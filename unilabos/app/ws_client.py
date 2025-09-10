#!/usr/bin/env python
# coding=utf-8
"""
WebSocket通信客户端和任务调度器

基于WebSocket协议的通信客户端实现，继承自BaseCommunicationClient。
包含WebSocketClient（连接管理）和TaskScheduler（任务调度）两个类。
"""

import json
import logging
import time
import uuid
import threading
import asyncio
import traceback
import websockets
import ssl as ssl_module
from dataclasses import dataclass
from typing import Optional, Dict, Any
from urllib.parse import urlparse
from unilabos.app.model import JobAddReq
from unilabos.ros.nodes.presets.host_node import HostNode
from unilabos.utils.type_check import serialize_result_info
from unilabos.app.communication import BaseCommunicationClient
from unilabos.config.config import WSConfig, HTTPConfig, BasicConfig
from unilabos.utils import logger


@dataclass
class QueueItem:
    """队列项数据结构"""

    task_type: str  # "query_action_status" 或 "job_call_back_status"
    device_id: str
    action_name: str
    task_id: str
    job_id: str
    device_action_key: str
    next_run_time: float  # 下次执行时间戳
    retry_count: int = 0  # 重试次数


class TaskScheduler:
    """
    任务调度器类

    负责任务队列管理、状态跟踪、业务逻辑处理等功能。
    """

    def __init__(self, message_sender: "WebSocketClient"):
        """初始化任务调度器"""
        self.message_sender = message_sender

        # 队列管理
        self.action_queue = []  # 任务队列
        self.action_queue_lock = threading.Lock()  # 队列锁

        # 任务状态跟踪
        self.active_jobs = {}  # job_id -> 任务信息
        self.cancel_events = {}  # job_id -> asyncio.Event for cancellation

        # 立即执行标记字典 - device_id+action_name -> timestamp
        self.immediate_execution_flags = {}  # 存储需要立即执行的设备动作组合
        self.immediate_execution_lock = threading.Lock()  # 立即执行标记锁

        # 队列处理器
        self.queue_processor_thread = None
        self.queue_running = False

    # 队列处理器相关方法
    def start(self) -> None:
        """启动任务调度器"""
        if self.queue_running:
            logger.warning("[TaskScheduler] Already running")
            return

        self.queue_running = True
        self.queue_processor_thread = threading.Thread(
            target=self._run_queue_processor, daemon=True, name="TaskScheduler"
        )
        self.queue_processor_thread.start()

    def stop(self) -> None:
        """停止任务调度器"""
        self.queue_running = False
        if self.queue_processor_thread and self.queue_processor_thread.is_alive():
            self.queue_processor_thread.join(timeout=5)
        logger.info("[TaskScheduler] Stopped")

    def _run_queue_processor(self):
        """在独立线程中运行队列处理器"""
        loop = asyncio.new_event_loop()
        try:
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._action_queue_processor())
        except Exception as e:
            logger.error(f"[TaskScheduler] Queue processor thread error: {str(e)}")
        finally:
            if loop:
                loop.close()

    async def _action_queue_processor(self) -> None:
        """队列处理器 - 从队列头部取出任务处理，保持顺序，使用list避免队尾排队问题"""
        logger.info("[TaskScheduler] Action queue processor started")

        try:
            while self.queue_running:
                try:
                    current_time = time.time()
                    items_to_process = []
                    items_to_requeue = []

                    # 使用锁安全地复制队列内容
                    with self.action_queue_lock:
                        if not self.action_queue:
                            # 队列为空，等待一段时间
                            pass
                        else:
                            # 复制队列内容以避免并发修改问题
                            items_to_process = self.action_queue.copy()
                            self.action_queue.clear()

                    if not items_to_process:
                        await asyncio.sleep(0.2)  # 队列为空时等待
                        continue

                    with self.immediate_execution_lock:
                        expired_keys = [k for k, v in self.immediate_execution_flags.items() if current_time > v]
                        for k in expired_keys:
                            del self.immediate_execution_flags[k]
                        immediate_execution = self.immediate_execution_flags.copy()
                    # 处理每个任务
                    for item in items_to_process:
                        try:
                            # 检查是否到了执行时间，是我们本地的执行时间，按顺序填入
                            if current_time < item.next_run_time and item.device_action_key not in immediate_execution:
                                # 还没到执行时间，保留在队列中（保持原有顺序）
                                items_to_requeue.append(item)
                                continue

                            # 执行相应的任务
                            should_continue = False
                            if item.task_type == "query_action_status":
                                should_continue = asyncio.run_coroutine_threadsafe(
                                    self._process_query_status_item(item), self.message_sender.event_loop
                                ).result()
                            elif item.task_type == "job_call_back_status":
                                should_continue = asyncio.run_coroutine_threadsafe(
                                    self._process_job_callback_item(item), self.message_sender.event_loop
                                ).result()
                            else:
                                logger.warning(f"[TaskScheduler] Unknown task type: {item.task_type}")
                                continue

                            # 如果需要继续，放入重新排队列表
                            if should_continue:
                                item.next_run_time = current_time + 10  # 10秒后再次执行
                                item.retry_count += 1
                                items_to_requeue.append(item)
                                logger.trace(  # type: ignore
                                    f"[TaskScheduler] Re-queued {item.job_id} {item.task_type} "
                                    f"for {item.device_action_key}"
                                )
                            else:
                                logger.debug(
                                    f"[TaskScheduler] Completed {item.job_id} {item.task_type} "
                                    f"for {item.device_action_key}"
                                )

                        except Exception as e:
                            logger.error(f"[TaskScheduler] Error processing item {item.task_type}: {str(e)}")

                    # 将需要重新排队的任务放回队列开头（保持原有顺序，确保优先于新任务执行）
                    if items_to_requeue and self.action_queue is not None:
                        with self.action_queue_lock:
                            self.action_queue = items_to_requeue + self.action_queue

                    await asyncio.sleep(0.1)  # 短暂等待避免过度占用CPU

                except Exception as e:
                    logger.error(f"[TaskScheduler] Error in queue processor: {str(e)}")
                    await asyncio.sleep(1)  # 错误后稍等再继续

        except asyncio.CancelledError:
            logger.info("[TaskScheduler] Action queue processor cancelled")
        except Exception as e:
            logger.error(f"[TaskScheduler] Fatal error in queue processor: {str(e)}")
        finally:
            logger.info("[TaskScheduler] Action queue processor stopped")

    # 队列处理方法
    async def _process_query_status_item(self, item: QueueItem) -> bool:
        """处理query_action_status类型的队列项，返回True表示需要继续，False表示可以停止"""
        try:
            # 检查设备状态
            host_node = HostNode.get_instance(0)
            if not host_node:
                logger.error("[TaskScheduler] HostNode instance not available in queue processor")
                return False

            action_jobs = len(host_node._device_action_status[item.device_action_key].job_ids)
            free = not bool(action_jobs)

            # 发送状态报告
            if free:
                # 设备空闲，发送最终状态并停止
                # 下面要增加和handle_query_state相同的逻辑
                host_node._device_action_status[item.device_action_key].job_ids[item.job_id] = time.time()
                await self._publish_device_action_state(
                    item.device_id, item.action_name, item.task_id, item.job_id, "query_action_status", True, 0
                )
                return False  # 停止继续监控
            else:
                # 设备忙碌，发送状态并继续监控
                await self._publish_device_action_state(
                    item.device_id, item.action_name, item.task_id, item.job_id, "query_action_status", False, 10
                )
                return True  # 继续监控

        except Exception as e:
            logger.error(f"[TaskScheduler] Error processing query status item: {str(e)}")
            return False  # 出错则停止

    async def _process_job_callback_item(self, item: QueueItem) -> bool:
        """处理job_call_back_status类型的队列项，返回True表示需要继续，False表示可以停止"""
        try:
            # 检查任务是否还在活跃列表中
            if item.job_id not in self.active_jobs:
                logger.debug(f"[TaskScheduler] Job {item.job_id} no longer active")
                return False

            # 检查是否收到取消信号
            if item.job_id in self.cancel_events and self.cancel_events[item.job_id].is_set():
                logger.info(f"[TaskScheduler] Job {item.job_id} cancelled via cancel event")
                return False

            # 检查设备状态
            host_node = HostNode.get_instance(0)
            if not host_node:
                logger.error(
                    f"[TaskScheduler] HostNode instance not available in job callback queue for job_id: {item.job_id}"
                )
                return False

            action_jobs = len(host_node._device_action_status[item.device_action_key].job_ids)
            free = not bool(action_jobs)

            # 发送job_call_back_status状态
            await self._publish_device_action_state(
                item.device_id, item.action_name, item.task_id, item.job_id, "job_call_back_status", free, 10
            )

            # 如果任务完成，停止监控
            if free:
                return False
            else:
                return True  # 继续监控

        except Exception as e:
            logger.error(f"[TaskScheduler] Error processing job callback item for job_id {item.job_id}: {str(e)}")
            return False  # 出错则停止

    # 消息发送方法
    async def _publish_device_action_state(
        self, device_id: str, action_name: str, task_id: str, job_id: str, typ: str, free: bool, need_more: int
    ) -> None:
        """发布设备动作状态"""
        message = {
            "action": "report_action_state",
            "data": {
                "type": typ,
                "device_id": device_id,
                "action_name": action_name,
                "task_id": task_id,
                "job_id": job_id,
                "free": free,
                "need_more": need_more,
            },
        }
        await self.message_sender.send_message(message)

    # 业务逻辑处理方法
    async def handle_query_state(self, data: Dict[str, str]) -> None:
        """处理query_action_state消息"""
        device_id = data.get("device_id", "")
        if not device_id:
            logger.error("[TaskScheduler] query_action_state missing device_id")
            return
        action_name = data.get("action_name", "")
        if not action_name:
            logger.error("[TaskScheduler] query_action_state missing action_name")
            return
        task_id = data.get("task_id", "")
        if not task_id:
            logger.error("[TaskScheduler] query_action_state missing task_id")
            return
        job_id = data.get("job_id", "")
        if not job_id:
            logger.error("[TaskScheduler] query_action_state missing job_id")
            return

        device_action_key = f"/devices/{device_id}/{action_name}"
        host_node = HostNode.get_instance(0)
        if not host_node:
            logger.error("[TaskScheduler] HostNode instance not available")
            return

        action_jobs = len(host_node._device_action_status[device_action_key].job_ids)
        free = not bool(action_jobs)

        # 如果设备空闲，立即响应free状态
        if free:
            await self._publish_device_action_state(
                device_id, action_name, task_id, job_id, "query_action_status", True, 0
            )
            logger.debug(f"[TaskScheduler] {job_id} Device {device_id}/{action_name} is free, responded immediately")
            host_node = HostNode.get_instance(0)
            if not host_node:
                logger.error(f"[TaskScheduler] HostNode instance not available for job_id: {job_id}")
                return
            host_node._device_action_status[device_action_key].job_ids[job_id] = time.time()
            return

        # 设备忙碌时，检查是否已有相同的轮询任务
        if self.action_queue is not None:
            with self.action_queue_lock:
                # 检查是否已存在相同job_id和task_id的轮询任务
                for existing_item in self.action_queue:
                    if (
                        existing_item.task_type == "query_action_status"
                        and existing_item.job_id == job_id
                        and existing_item.task_id == task_id
                        and existing_item.device_action_key == device_action_key
                    ):
                        logger.error(
                            f"[TaskScheduler] Duplicate query_action_state ignored: "
                            f"job_id={job_id}, task_id={task_id}, server error"
                        )
                        return

                # 没有重复，加入轮询队列
                queue_item = QueueItem(
                    task_type="query_action_status",
                    device_id=device_id,
                    action_name=action_name,
                    task_id=task_id,
                    job_id=job_id,
                    device_action_key=device_action_key,
                    next_run_time=time.time() + 10,  # 10秒后执行
                )
                self.action_queue.append(queue_item)
                logger.debug(
                    f"[TaskScheduler] {job_id} Device {device_id}/{action_name} is busy, "
                    f"added to polling queue {action_jobs}"
                )

                # 立即发送busy状态
                await self._publish_device_action_state(
                    device_id, action_name, task_id, job_id, "query_action_status", False, 10
                )
        else:
            logger.warning("[TaskScheduler] Action queue not available")

    async def handle_job_start(self, data: Dict[str, Any]):
        """处理作业启动消息"""
        try:
            req = JobAddReq(**data)
            device_action_key = f"/devices/{req.device_id}/{req.action}"

            logger.info(
                f"[TaskScheduler] Starting job with job_id: {req.job_id}, "
                f"device: {req.device_id}, action: {req.action}"
            )

            # 添加到活跃任务
            self.active_jobs[req.job_id] = {
                "device_id": req.device_id,
                "action_name": req.action,
                "task_id": data.get("task_id", ""),
                "start_time": time.time(),
                "device_action_key": device_action_key,
                "callback_started": False,  # 标记callback是否已启动
            }

            # 创建取消事件，todo：要移动到query_state中
            self.cancel_events[req.job_id] = asyncio.Event()
            # 启动callback定时发送
            await self._start_job_callback(req.job_id, req.device_id, req.action, req.task_id, device_action_key)

            # 创建兼容HostNode的QueueItem对象
            job_queue_item = QueueItem(
                task_type="job_call_back_status",
                device_id=req.device_id,
                action_name=req.action,
                task_id=req.task_id,
                job_id=req.job_id,
                device_action_key=device_action_key,
                next_run_time=time.time(),
            )
            try:

                host_node = HostNode.get_instance(0)
                if not host_node:
                    logger.error(f"[TaskScheduler] HostNode instance not available for job_id: {req.job_id}")
                    return
                host_node.send_goal(
                    job_queue_item,
                    action_type=req.action_type,
                    action_kwargs=req.action_args,
                    server_info=req.server_info,
                )
            except Exception as e:
                logger.error(f"[TaskScheduler] Exception during job start for job_id {req.job_id}: {str(e)}")
                traceback.print_exc()
                self.publish_job_status(
                    {}, job_queue_item, "failed", serialize_result_info(traceback.format_exc(), False, {})
                )
        except Exception as e:
            logger.error(f"[TaskScheduler] Error handling job start: {str(e)}")

    async def handle_cancel_action(self, data: Dict[str, Any]) -> None:
        """处理取消动作请求"""
        task_id = data.get("task_id")
        job_id = data.get("job_id")

        logger.debug(f"[TaskScheduler] Handling cancel action request - task_id: {task_id}, job_id: {job_id}")

        if not task_id and not job_id:
            logger.error("[TaskScheduler] cancel_action missing both task_id and job_id")
            return

        # 通过job_id取消
        if job_id:
            logger.info(f"[TaskScheduler] Cancelling job by job_id: {job_id}")
            # 设置取消事件
            if job_id in self.cancel_events:
                self.cancel_events[job_id].set()
                logger.debug(f"[TaskScheduler] Set cancel event for job_id: {job_id}")
            else:
                logger.warning(f"[TaskScheduler] Cancel event not found for job_id: {job_id}")

            # 停止job callback并发送取消状态
            if job_id in self.active_jobs:
                logger.debug(f"[TaskScheduler] Found active job for cancellation: {job_id}")
                # 调用HostNode的cancel_goal
                host_node = HostNode.get_instance(0)
                if host_node:
                    host_node.cancel_goal(job_id)
                    logger.info(f"[TaskScheduler] Cancelled goal in HostNode for job_id: {job_id}")
                else:
                    logger.error(f"[TaskScheduler] HostNode not available for cancel goal: {job_id}")

                # 停止callback并发送取消状态
                await self._stop_job_callback(job_id, "cancelled")
                logger.info(f"[TaskScheduler] Stopped job callback and sent cancel status for job_id: {job_id}")
            else:
                logger.warning(f"[TaskScheduler] Job not found in active jobs for cancellation: {job_id}")

        # 通过task_id取消（需要查找对应的job_id）
        if task_id and not job_id:
            logger.debug(f"[TaskScheduler] Cancelling jobs by task_id: {task_id}")
            jobs_to_cancel = []
            for jid, job_info in self.active_jobs.items():
                if job_info.get("task_id") == task_id:
                    jobs_to_cancel.append(jid)

            logger.debug(
                f"[TaskScheduler] Found {len(jobs_to_cancel)} jobs to cancel for task_id {task_id}: {jobs_to_cancel}"
            )

            for jid in jobs_to_cancel:
                logger.debug(f"[TaskScheduler] Recursively cancelling job_id: {jid} for task_id: {task_id}")
                # 递归调用自身来取消每个job
                await self.handle_cancel_action({"job_id": jid})

        logger.debug(f"[TaskScheduler] Completed cancel action handling - task_id: {task_id}, job_id: {job_id}")

    # job管理方法
    async def _start_job_callback(
        self, job_id: str, device_id: str, action_name: str, task_id: str, device_action_key: str
    ) -> None:
        """启动job的callback定时发送"""
        if job_id not in self.active_jobs:
            logger.debug(f"[TaskScheduler] Job not found in active jobs when starting callback: {job_id}")
            return

        # 检查是否已经启动过callback
        if self.active_jobs[job_id].get("callback_started", False):
            logger.warning(f"[TaskScheduler] Job callback already started for job_id: {job_id}")
            return

        # 标记callback已启动
        self.active_jobs[job_id]["callback_started"] = True

        # 将job_call_back_status任务放入队列
        queue_item = QueueItem(
            task_type="job_call_back_status",
            device_id=device_id,
            action_name=action_name,
            task_id=task_id,
            job_id=job_id,
            device_action_key=device_action_key,
            next_run_time=time.time() + 10,  # 10秒后开始报送
        )
        if self.action_queue is not None:
            with self.action_queue_lock:
                self.action_queue.append(queue_item)
        else:
            logger.debug(f"[TaskScheduler] Action queue not available for job callback: {job_id}")

    async def _stop_job_callback(self, job_id: str, final_status: str) -> None:
        """停止job的callback定时发送并发送最终结果"""
        logger.info(f"[TaskScheduler] Stopping job callback for job_id: {job_id} with final status: {final_status}")
        if job_id not in self.active_jobs:
            logger.debug(f"[TaskScheduler] Job {job_id} not found in active jobs when stopping callback")
            return

        job_info = self.active_jobs[job_id]
        device_id = job_info["device_id"]
        action_name = job_info["action_name"]
        task_id = job_info["task_id"]
        device_action_key = job_info["device_action_key"]

        logger.debug(
            f"[TaskScheduler] Job {job_id} details - device: {device_id}, action: {action_name}, task: {task_id}"
        )

        # 移除活跃任务和取消事件（这会让队列处理器自动停止callback）
        self.active_jobs.pop(job_id, None)
        self.cancel_events.pop(job_id, None)
        logger.debug(f"[TaskScheduler] Removed job {job_id} from active jobs and cancel events")

        # 发送最终的callback状态
        await self._publish_device_action_state(
            device_id, action_name, task_id, job_id, "job_call_back_status", True, 0
        )
        logger.debug(f"[TaskScheduler] Completed stopping job callback for {job_id} with final status: {final_status}")

    # 外部接口方法
    def publish_job_status(
        self, feedback_data: dict, item: "QueueItem", status: str, return_info: Optional[str] = None
    ) -> None:
        """发布作业状态，拦截最终结果（给HostNode调用的接口）"""
        if not self.message_sender.is_connected():
            logger.debug(f"[TaskScheduler] Not connected, cannot publish job status for job_id: {item.job_id}")
            return

        # 拦截最终结果状态
        if status in ["success", "failed"]:
            host_node = HostNode.get_instance(0)
            if host_node:
                host_node._device_action_status[item.device_action_key].job_ids.pop(item.job_id)
            logger.info(f"[TaskScheduler] Intercepting final status for job_id: {item.job_id} - {status}")
            # 给其他同名action至少执行一次的机会
            with self.immediate_execution_lock:
                self.immediate_execution_flags[item.device_action_key] = time.time() + 3

            # 检查是否在同一个事件循环中
            try:
                current_loop = asyncio.get_running_loop()
                if current_loop == self.message_sender.event_loop:
                    # 在同一个事件循环中，直接创建任务
                    asyncio.create_task(self._stop_job_callback(item.job_id, status))
                else:
                    # 不在同一个事件循环中，使用 run_coroutine_threadsafe
                    asyncio.run_coroutine_threadsafe(
                        self._stop_job_callback(item.job_id, status), self.message_sender.event_loop
                    )
            except RuntimeError:
                # 没有运行中的事件循环，使用 run_coroutine_threadsafe
                asyncio.run_coroutine_threadsafe(
                    self._stop_job_callback(item.job_id, status), self.message_sender.event_loop
                )

        # 执行结果信息上传
        message = {
            "action": "job_status",
            "data": {
                "job_id": item.job_id,
                "task_id": item.task_id,
                "device_id": item.device_id,
                "action_name": item.action_name,
                "status": status,
                "feedback_data": feedback_data,
                "return_info": return_info,
                "timestamp": time.time(),
            },
        }

        # 同样检查事件循环
        try:
            current_loop = asyncio.get_running_loop()
            if current_loop == self.message_sender.event_loop:
                # 在同一个事件循环中，直接创建任务
                asyncio.create_task(self.message_sender.send_message(message))
            else:
                # 不在同一个事件循环中，使用 run_coroutine_threadsafe
                asyncio.run_coroutine_threadsafe(
                    self.message_sender.send_message(message), self.message_sender.event_loop
                )
        except RuntimeError:
            # 没有运行中的事件循环，使用 run_coroutine_threadsafe
            asyncio.run_coroutine_threadsafe(self.message_sender.send_message(message), self.message_sender.event_loop)

        logger.trace(f"[TaskScheduler] Job status published: {item.job_id} - {status}")  # type: ignore

    def cancel_goal(self, job_id: str) -> None:
        """取消指定的任务（给外部调用的接口）"""
        logger.debug(f"[TaskScheduler] External cancel request for job_id: {job_id}")
        if job_id in self.cancel_events:
            logger.debug(f"[TaskScheduler] Found cancel event for job_id: {job_id}, processing cancellation")
            try:
                loop = asyncio.get_event_loop()
                loop.create_task(self.handle_cancel_action({"job_id": job_id}))
                logger.debug(f"[TaskScheduler] Scheduled cancel action for job_id: {job_id}")
            except RuntimeError:
                asyncio.run(self.handle_cancel_action({"job_id": job_id}))
                logger.debug(f"[TaskScheduler] Executed cancel action for job_id: {job_id}")
            logger.debug(f"[TaskScheduler] Initiated cancel for job_id: {job_id}")
        else:
            logger.debug(f"[TaskScheduler] Job {job_id} not found in cancel events for cancellation")


class WebSocketClient(BaseCommunicationClient):
    """
    WebSocket通信客户端类

    专注于WebSocket连接管理和消息传输。
    """

    def __init__(self):
        super().__init__()
        self.is_disabled = False
        self.client_id = f"{uuid.uuid4()}"

        # WebSocket连接相关
        self.websocket = None
        self.connection_loop = None
        self.event_loop: asyncio.AbstractEventLoop = None  # type: ignore
        self.connection_thread = None
        self.is_running = False
        self.connected = False

        # 消息处理
        self.reconnect_count = 0

        # 消息发送队列和处理器
        self.send_queue = None  # 延迟初始化
        self.send_queue_task = None  # 发送队列处理任务

        # 任务调度器
        self.task_scheduler = None

        # 构建WebSocket URL
        self._build_websocket_url()

        logger.info(f"[WebSocket] Client_id: {self.client_id}")

    # 初始化方法
    def _initialize_task_scheduler(self):
        """初始化任务调度器"""
        if not self.task_scheduler:
            self.task_scheduler = TaskScheduler(self)
            self.task_scheduler.start()
            logger.info("[WebSocket] Task scheduler initialized")

    def _build_websocket_url(self):
        """构建WebSocket连接URL"""
        if not HTTPConfig.remote_addr:
            self.websocket_url = None
            return

        # 解析服务器URL
        parsed = urlparse(HTTPConfig.remote_addr)

        # 根据SSL配置选择协议
        if parsed.scheme == "https":
            scheme = "wss"
        else:
            scheme = "ws"
        if ":" in parsed.netloc and parsed.port is not None:
            self.websocket_url = f"{scheme}://{parsed.hostname}:{parsed.port + 1}/api/v1/ws/schedule"
        else:
            self.websocket_url = f"{scheme}://{parsed.netloc}/api/v1/ws/schedule"
        logger.debug(f"[WebSocket] URL: {self.websocket_url}")

    # 连接管理方法
    def start(self) -> None:
        """启动WebSocket连接和任务调度器"""
        if self.is_disabled:
            logger.warning("[WebSocket] WebSocket is disabled, skipping connection.")
            return

        if not self.websocket_url:
            logger.error("[WebSocket] WebSocket URL not configured")
            return

        logger.info(f"[WebSocket] Starting connection to {self.websocket_url}")

        # 初始化任务调度器
        self._initialize_task_scheduler()

        self.is_running = True

        # 在单独线程中运行WebSocket连接
        self.connection_thread = threading.Thread(target=self._run_connection, daemon=True, name="WebSocketConnection")
        self.connection_thread.start()

    def stop(self) -> None:
        """停止WebSocket连接和任务调度器"""
        if self.is_disabled:
            return

        logger.info("[WebSocket] Stopping connection")
        self.is_running = False
        self.connected = False

        # 停止任务调度器
        if self.task_scheduler:
            self.task_scheduler.stop()

        if self.event_loop and self.event_loop.is_running():
            asyncio.run_coroutine_threadsafe(self._close_connection(), self.event_loop)

        if self.connection_thread and self.connection_thread.is_alive():
            self.connection_thread.join(timeout=5)

    def _run_connection(self):
        """在独立线程中运行WebSocket连接"""
        try:
            # 创建新的事件循环
            self.event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.event_loop)

            # 在正确的事件循环中创建发送队列
            self.send_queue = asyncio.Queue(maxsize=1000)  # 限制队列大小防止内存溢出

            # 运行连接逻辑
            self.event_loop.run_until_complete(self._connection_handler())
        except Exception as e:
            logger.error(f"[WebSocket] Connection thread error: {str(e)}")
            logger.error(traceback.format_exc())
        finally:
            if self.event_loop:
                self.event_loop.close()

    async def _connection_handler(self):
        """处理WebSocket连接和重连逻辑"""
        while self.is_running:
            try:
                # 构建SSL上下文
                ssl_context = None
                assert self.websocket_url is not None
                if self.websocket_url.startswith("wss://"):
                    ssl_context = ssl_module.create_default_context()
                ws_logger = logging.getLogger("websockets.client")
                ws_logger.setLevel(logging.INFO)
                async with websockets.connect(
                    self.websocket_url,
                    ssl=ssl_context,
                    ping_interval=WSConfig.ping_interval,
                    ping_timeout=10,
                    additional_headers={"Authorization": f"Lab {BasicConfig.auth_secret()}"},
                    logger=ws_logger,
                ) as websocket:
                    self.websocket = websocket
                    self.connected = True
                    self.reconnect_count = 0

                    logger.info(f"[WebSocket] Connected to {self.websocket_url}")

                    # 启动发送队列处理器
                    self.send_queue_task = asyncio.create_task(self._send_queue_processor())

                    try:
                        # 处理消息
                        await self._message_handler()
                    finally:
                        # 停止发送队列处理器
                        if self.send_queue_task and not self.send_queue_task.done():
                            self.send_queue_task.cancel()
                            try:
                                await self.send_queue_task
                            except asyncio.CancelledError:
                                pass

            except websockets.exceptions.ConnectionClosed:
                logger.warning("[WebSocket] Connection closed")
                self.connected = False
            except Exception as e:
                logger.error(f"[WebSocket] Connection error: {str(e)}")
                self.connected = False
            finally:
                # WebSocket连接结束时只需重置websocket对象
                self.websocket = None

            # 重连逻辑
            if self.is_running and self.reconnect_count < WSConfig.max_reconnect_attempts:
                self.reconnect_count += 1
                logger.info(
                    f"[WebSocket] Reconnecting in {WSConfig.reconnect_interval}s "
                    f"(attempt {self.reconnect_count}/{WSConfig.max_reconnect_attempts})"
                )
                await asyncio.sleep(WSConfig.reconnect_interval)
            elif self.reconnect_count >= WSConfig.max_reconnect_attempts:
                logger.error("[WebSocket] Max reconnection attempts reached")
                break
            else:
                self.reconnect_count -= 1

    async def _close_connection(self):
        """关闭WebSocket连接"""
        if self.websocket:
            await self.websocket.close()
            self.websocket = None

    async def _send_queue_processor(self):
        """处理发送队列中的消息"""
        logger.debug("[WebSocket] Send queue processor started")
        if not self.send_queue:
            logger.error("[WebSocket] Send queue not initialized")
            return

        try:
            while self.connected and self.websocket:
                try:
                    # 使用超时避免无限等待
                    message = await asyncio.wait_for(self.send_queue.get(), timeout=1.0)

                    # 批量处理：收集短时间内的多个消息
                    messages_to_send = [message]
                    batch_size = 0
                    max_batch_size = 10  # 最大批处理数量

                    # 尝试获取更多消息（非阻塞）
                    while batch_size < max_batch_size and not self.send_queue.empty():
                        try:
                            additional_msg = self.send_queue.get_nowait()
                            messages_to_send.append(additional_msg)
                            batch_size += 1
                        except asyncio.QueueEmpty:
                            break

                    # 发送消息
                    for msg in messages_to_send:
                        if self.websocket and self.connected:
                            try:
                                message_str = json.dumps(msg, ensure_ascii=False)
                                await self.websocket.send(message_str)
                                logger.trace(  # type: ignore
                                    f"[WebSocket] Message sent: {msg.get('action', 'unknown')}"
                                )
                            except Exception as e:
                                logger.error(f"[WebSocket] Failed to send message: {str(e)}")
                                # 如果发送失败，将消息重新放回队列（可选）
                                # await self.send_queue.put(msg)
                                break

                        # 在批量发送之间添加小延迟，避免过载
                        if batch_size > 5:
                            await asyncio.sleep(0.001)

                except asyncio.TimeoutError:
                    # 超时是正常的，继续循环
                    continue
                except asyncio.CancelledError:
                    logger.debug("[WebSocket] Send queue processor cancelled")
                    break
                except Exception as e:
                    logger.error(f"[WebSocket] Error in send queue processor: {str(e)}")
                    await asyncio.sleep(0.1)

        except Exception as e:
            logger.error(f"[WebSocket] Fatal error in send queue processor: {str(e)}")
        finally:
            logger.debug("[WebSocket] Send queue processor stopped")

    # 消息处理方法
    async def _message_handler(self):
        """处理接收到的消息"""
        if not self.websocket:
            logger.error("[WebSocket] WebSocket connection is None")
            return

        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    await self._process_message(data)
                except json.JSONDecodeError:
                    logger.error(f"[WebSocket] Invalid JSON received: {message}")
                except Exception as e:
                    logger.error(f"[WebSocket] Error processing message: {str(e)}")
        except websockets.exceptions.ConnectionClosed:
            logger.info("[WebSocket] Message handler stopped - connection closed")
        except Exception as e:
            logger.error(f"[WebSocket] Message handler error: {str(e)}")

    async def _process_message(self, input_message: Dict[str, Any]):
        """处理收到的消息"""
        message_type = input_message.get("action", "")
        data = input_message.get("data", {})

        if message_type == "pong":
            # 处理pong响应（WebSocket层面的连接管理）
            self._handle_pong_sync(data)
        elif self.task_scheduler:
            # 其他消息交给TaskScheduler处理
            if message_type == "job_start":
                await self.task_scheduler.handle_job_start(data)
            elif message_type == "query_action_state":
                await self.task_scheduler.handle_query_state(data)
            elif message_type == "cancel_action":
                await self.task_scheduler.handle_cancel_action(data)
            elif message_type == "":
                return
            else:
                logger.debug(f"[WebSocket] Unknown message: {input_message}")
        else:
            logger.warning(f"[WebSocket] Task scheduler not available for message: {message_type}")

    def _handle_pong_sync(self, pong_data: Dict[str, Any]):
        """同步处理pong响应"""
        host_node = HostNode.get_instance(0)
        if host_node:
            host_node.handle_pong_response(pong_data)

    # MessageSender接口实现
    async def send_message(self, message: Dict[str, Any]) -> None:
        """内部发送消息方法，将消息放入发送队列"""
        if not self.connected:
            logger.warning("[WebSocket] Not connected, cannot send message")
            return

        # 检查发送队列是否已初始化
        if not self.send_queue:
            logger.warning("[WebSocket] Send queue not initialized, cannot send message")
            return

        try:
            # 尝试将消息放入队列（非阻塞）
            self.send_queue.put_nowait(message)
            logger.trace(f"[WebSocket] Message queued: {message['action']}")  # type: ignore
        except asyncio.QueueFull:
            logger.error(f"[WebSocket] Send queue full, dropping message: {message['action']}")
            # 可选：在队列满时采取其他策略，如等待或丢弃旧消息

    def is_connected(self) -> bool:
        """检查是否已连接（TaskScheduler调用的接口）"""
        return self.connected and not self.is_disabled

    # 基类方法实现
    def publish_device_status(self, device_status: dict, device_id: str, property_name: str) -> None:
        """发布设备状态"""
        if self.is_disabled or not self.connected:
            return
        message = {
            "action": "device_status",
            "data": {
                "device_id": device_id,
                "data": {
                    "property_name": property_name,
                    "status": device_status.get(device_id, {}).get(property_name),
                    "timestamp": time.time(),
                },
            },
        }

        # 检查是否在同一个事件循环中
        try:
            current_loop = asyncio.get_running_loop()
            if current_loop == self.event_loop:
                # 在同一个事件循环中，直接创建任务
                asyncio.create_task(self.send_message(message))
            else:
                # 不在同一个事件循环中，使用 run_coroutine_threadsafe
                asyncio.run_coroutine_threadsafe(self.send_message(message), self.event_loop)
        except RuntimeError:
            # 没有运行中的事件循环，使用 run_coroutine_threadsafe
            asyncio.run_coroutine_threadsafe(self.send_message(message), self.event_loop)

        logger.debug(f"[WebSocket] Device status published: {device_id}.{property_name}")

    def publish_job_status(
        self, feedback_data: dict, item: "QueueItem", status: str, return_info: Optional[str] = None
    ) -> None:
        """发布作业状态（转发给TaskScheduler）"""
        if self.task_scheduler:
            self.task_scheduler.publish_job_status(feedback_data, item, status, return_info)
        else:
            logger.debug(f"[WebSocket] Task scheduler not available for job status: {item.job_id}")

    def send_ping(self, ping_id: str, timestamp: float) -> None:
        """发送ping消息"""
        if self.is_disabled or not self.connected:
            logger.warning("[WebSocket] Not connected, cannot send ping")
            return
        message = {"action": "ping", "data": {"ping_id": ping_id, "client_timestamp": timestamp}}

        # 检查是否在同一个事件循环中
        try:
            current_loop = asyncio.get_running_loop()
            if current_loop == self.event_loop:
                # 在同一个事件循环中，直接创建任务
                asyncio.create_task(self.send_message(message))
            else:
                # 不在同一个事件循环中，使用 run_coroutine_threadsafe
                asyncio.run_coroutine_threadsafe(self.send_message(message), self.event_loop)
        except RuntimeError:
            # 没有运行中的事件循环，使用 run_coroutine_threadsafe
            asyncio.run_coroutine_threadsafe(self.send_message(message), self.event_loop)

        logger.debug(f"[WebSocket] Ping sent: {ping_id}")

    def cancel_goal(self, job_id: str) -> None:
        """取消指定的任务（转发给TaskScheduler）"""
        logger.debug(f"[WebSocket] Received cancel goal request for job_id: {job_id}")
        if self.task_scheduler:
            self.task_scheduler.cancel_goal(job_id)
            logger.debug(f"[WebSocket] Forwarded cancel goal to TaskScheduler for job_id: {job_id}")
        else:
            logger.debug(f"[WebSocket] Task scheduler not available for cancel goal: {job_id}")
