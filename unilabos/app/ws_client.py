#!/usr/bin/env python
# coding=utf-8
"""
WebSocket通信客户端重构版本 v2

基于两线程架构的WebSocket客户端实现：
1. 消息处理线程 - 处理WebSocket消息，划分任务执行和任务队列
2. 队列处理线程 - 定时给发送队列推送消息，管理任务状态
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
from queue import Queue, Empty
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Callable, List, Set
from urllib.parse import urlparse
from enum import Enum
from unilabos.app.model import JobAddReq
from unilabos.ros.nodes.presets.host_node import HostNode
from unilabos.utils.type_check import serialize_result_info
from unilabos.app.communication import BaseCommunicationClient
from unilabos.config.config import WSConfig, HTTPConfig, BasicConfig
from unilabos.utils import logger


def format_job_log(job_id: str, task_id: str = "", device_id: str = "", action_name: str = "") -> str:
    """格式化job日志信息：jobid[:4]-taskid[:4] device_id/action_name"""
    job_part = f"{job_id[:4]}-{task_id[:4]}" if task_id else job_id[:4]
    device_part = f"{device_id}/{action_name}" if device_id and action_name else ""
    return f"{job_part} {device_part}".strip()


class JobStatus(Enum):
    """任务状态枚举"""

    QUEUE = "queue"  # 排队中
    READY = "ready"  # 已获得执行许可，等待开始
    STARTED = "started"  # 执行中
    ENDED = "ended"  # 已结束


@dataclass
class QueueItem:
    """队列项数据结构"""

    task_type: str  # "query_action_status" 或 "job_call_back_status"
    device_id: str
    action_name: str
    task_id: str
    job_id: str
    device_action_key: str
    next_run_time: float = 0  # 下次执行时间戳
    retry_count: int = 0  # 重试次数


@dataclass
class JobInfo:
    """任务信息数据结构"""

    job_id: str
    task_id: str
    device_id: str
    action_name: str
    device_action_key: str
    status: JobStatus
    start_time: float
    last_update_time: float = field(default_factory=time.time)
    ready_timeout: Optional[float] = None  # READY状态的超时时间

    def update_timestamp(self):
        """更新最后更新时间"""
        self.last_update_time = time.time()

    def set_ready_timeout(self, timeout_seconds: int = 10):
        """设置READY状态超时时间"""
        self.ready_timeout = time.time() + timeout_seconds

    def is_ready_timeout(self) -> bool:
        """检查READY状态是否超时"""
        return self.status == JobStatus.READY and self.ready_timeout is not None and time.time() > self.ready_timeout


@dataclass
class WebSocketMessage:
    """WebSocket消息数据结构"""

    action: str
    data: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)


class DeviceActionManager:
    """设备动作管理器 - 管理每个device_action_key的任务队列"""

    def __init__(self):
        self.device_queues: Dict[str, List[JobInfo]] = {}  # device_action_key -> job queue
        self.active_jobs: Dict[str, JobInfo] = {}  # device_action_key -> active job
        self.all_jobs: Dict[str, JobInfo] = {}  # job_id -> job_info
        self.lock = threading.RLock()

    def add_queue_request(self, job_info: JobInfo) -> bool:
        """
        添加队列请求
        返回True表示可以立即执行(free)，False表示需要排队(busy)
        """
        with self.lock:
            device_key = job_info.device_action_key

            # 总是将job添加到all_jobs中
            self.all_jobs[job_info.job_id] = job_info

            # 检查是否有正在执行或准备执行的任务
            if device_key in self.active_jobs:
                # 有正在执行或准备执行的任务，加入队列
                if device_key not in self.device_queues:
                    self.device_queues[device_key] = []
                job_info.status = JobStatus.QUEUE
                self.device_queues[device_key].append(job_info)
                job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
                logger.info(f"[DeviceActionManager] Job {job_log} queued for {device_key}")
                return False

            # 检查是否有排队的任务
            if device_key in self.device_queues and self.device_queues[device_key]:
                # 有排队的任务，加入队列末尾
                job_info.status = JobStatus.QUEUE
                self.device_queues[device_key].append(job_info)
                job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
                logger.info(f"[DeviceActionManager] Job {job_log} queued for {device_key}")
                return False

            # 没有正在执行或排队的任务，可以立即执行
            # 将其状态设为READY并占位，防止后续job也被判断为free
            job_info.status = JobStatus.READY
            job_info.update_timestamp()
            job_info.set_ready_timeout(10)  # 设置10秒超时
            self.active_jobs[device_key] = job_info
            job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
            logger.info(f"[DeviceActionManager] Job {job_log} can start immediately for {device_key}")
            return True

    def start_job(self, job_id: str) -> bool:
        """
        开始执行任务
        返回True表示成功开始，False表示失败
        """
        with self.lock:
            if job_id not in self.all_jobs:
                logger.error(f"[DeviceActionManager] Job {job_id[:4]} not found for start")
                return False

            job_info = self.all_jobs[job_id]
            device_key = job_info.device_action_key

            # 检查job的状态是否正确
            if job_info.status != JobStatus.READY:
                job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
                logger.error(f"[DeviceActionManager] Job {job_log} is not in READY status, current: {job_info.status}")
                return False

            # 检查设备上是否是这个job
            if device_key not in self.active_jobs or self.active_jobs[device_key].job_id != job_id:
                job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
                logger.error(f"[DeviceActionManager] Job {job_log} is not the active job for {device_key}")
                return False

            # 开始执行任务，将状态从READY转换为STARTED
            job_info.status = JobStatus.STARTED
            job_info.update_timestamp()
            job_info.ready_timeout = None  # 清除超时时间

            job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
            logger.info(f"[DeviceActionManager] Job {job_log} started for {device_key}")
            return True

    def end_job(self, job_id: str) -> Optional[JobInfo]:
        """
        结束任务，返回下一个可以执行的任务(如果有的话)
        """
        with self.lock:
            if job_id not in self.all_jobs:
                logger.warning(f"[DeviceActionManager] Job {job_id[:4]} not found for end")
                return None

            job_info = self.all_jobs[job_id]
            device_key = job_info.device_action_key

            # 移除活跃任务
            if device_key in self.active_jobs and self.active_jobs[device_key].job_id == job_id:
                del self.active_jobs[device_key]
                job_info.status = JobStatus.ENDED
                job_info.update_timestamp()
                # 从all_jobs中移除已结束的job
                del self.all_jobs[job_id]
                job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
                logger.info(f"[DeviceActionManager] Job {job_log} ended for {device_key}")
            else:
                job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
                logger.warning(f"[DeviceActionManager] Job {job_log} was not active for {device_key}")

            # 检查队列中是否有等待的任务
            if device_key in self.device_queues and self.device_queues[device_key]:
                next_job = self.device_queues[device_key].pop(0)  # FIFO
                # 将下一个job设置为READY状态并放入active_jobs
                next_job.status = JobStatus.READY
                next_job.update_timestamp()
                next_job.set_ready_timeout(10)  # 设置10秒超时
                self.active_jobs[device_key] = next_job
                next_job_log = format_job_log(
                    next_job.job_id, next_job.task_id, next_job.device_id, next_job.action_name
                )
                logger.info(f"[DeviceActionManager] Next job {next_job_log} can start for {device_key}")
                return next_job

            return None

    def get_active_jobs(self) -> List[JobInfo]:
        """获取所有正在执行的任务"""
        with self.lock:
            return list(self.active_jobs.values())

    def get_queued_jobs(self) -> List[JobInfo]:
        """获取所有排队中的任务"""
        with self.lock:
            queued = []
            for queue in self.device_queues.values():
                queued.extend(queue)
            return queued

    def get_job_info(self, job_id: str) -> Optional[JobInfo]:
        """获取任务信息"""
        with self.lock:
            return self.all_jobs.get(job_id)

    def cancel_job(self, job_id: str) -> bool:
        """取消单个任务"""
        with self.lock:
            if job_id not in self.all_jobs:
                logger.warning(f"[DeviceActionManager] Job {job_id[:4]} not found for cancel")
                return False

            job_info = self.all_jobs[job_id]
            device_key = job_info.device_action_key

            # 如果是正在执行的任务
            if (
                device_key in self.active_jobs and self.active_jobs[device_key].job_id == job_id
            ):  # 后面需要和cancel_goal进行联动，而不是在这里进行处理，现在默认等待这个job结束
                # del self.active_jobs[device_key]
                # job_info.status = JobStatus.ENDED
                # # 从all_jobs中移除
                # del self.all_jobs[job_id]
                # job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
                # logger.info(f"[DeviceActionManager] Active job {job_log} cancelled for {device_key}")

                # # 启动下一个任务
                # if device_key in self.device_queues and self.device_queues[device_key]:
                #     next_job = self.device_queues[device_key].pop(0)
                #     # 将下一个job设置为READY状态并放入active_jobs
                #     next_job.status = JobStatus.READY
                #     next_job.update_timestamp()
                #     next_job.set_ready_timeout(10)
                #     self.active_jobs[device_key] = next_job
                #     next_job_log = format_job_log(next_job.job_id, next_job.task_id,
                #                                   next_job.device_id, next_job.action_name)
                #     logger.info(f"[DeviceActionManager] Next job {next_job_log} can start after cancel")
                #     return True
                pass

            # 如果是排队中的任务
            elif device_key in self.device_queues:
                original_length = len(self.device_queues[device_key])
                self.device_queues[device_key] = [j for j in self.device_queues[device_key] if j.job_id != job_id]
                if len(self.device_queues[device_key]) < original_length:
                    job_info.status = JobStatus.ENDED
                    # 从all_jobs中移除
                    del self.all_jobs[job_id]
                    job_log = format_job_log(
                        job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name
                    )
                    logger.info(f"[DeviceActionManager] Queued job {job_log} cancelled for {device_key}")
                    return True

            job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
            logger.warning(f"[DeviceActionManager] Job {job_log} not found in active or queued jobs")
            return False

    def cancel_jobs_by_task_id(self, task_id: str) -> List[str]:
        """按task_id取消所有相关任务，返回被取消的job_id列表"""
        cancelled_job_ids = []

        # 首先找到所有属于该task_id的job
        jobs_to_cancel = []
        with self.lock:
            jobs_to_cancel = [job_info for job_info in self.all_jobs.values() if job_info.task_id == task_id]

        if not jobs_to_cancel:
            logger.warning(f"[DeviceActionManager] No jobs found for task_id: {task_id}")
            return cancelled_job_ids

        logger.info(f"[DeviceActionManager] Found {len(jobs_to_cancel)} jobs to cancel for task_id: {task_id}")

        # 逐个取消job
        for job_info in jobs_to_cancel:
            if self.cancel_job(job_info.job_id):
                cancelled_job_ids.append(job_info.job_id)

        logger.info(
            f"[DeviceActionManager] Successfully cancelled {len(cancelled_job_ids)} " f"jobs for task_id: {task_id}"
        )

        return cancelled_job_ids

    def check_ready_timeouts(self) -> List[JobInfo]:
        """检查READY状态超时的任务，仅检测不处理"""
        timeout_jobs = []

        with self.lock:
            # 统计READY状态的任务数量
            ready_jobs_count = sum(1 for job in self.active_jobs.values() if job.status == JobStatus.READY)
            if ready_jobs_count > 0:
                logger.trace(f"[DeviceActionManager] Checking {ready_jobs_count} READY jobs for timeout")  # type: ignore  # noqa: E501

            # 找到所有超时的READY任务（只检测，不处理）
            for job_info in self.active_jobs.values():
                if job_info.is_ready_timeout():
                    timeout_jobs.append(job_info)
                    job_log = format_job_log(
                        job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name
                    )
                    logger.warning(f"[DeviceActionManager] Job {job_log} READY timeout detected")

        return timeout_jobs


class MessageProcessor:
    """消息处理线程 - 处理WebSocket消息，划分任务执行和任务队列"""

    def __init__(self, websocket_url: str, send_queue: Queue, device_manager: DeviceActionManager):
        self.websocket_url = websocket_url
        self.send_queue = send_queue
        self.device_manager = device_manager
        self.queue_processor = None  # 延迟设置
        self.websocket_client = None  # 延迟设置

        # WebSocket连接
        self.websocket = None
        self.connected = False

        # 线程控制
        self.is_running = False
        self.thread = None
        self.reconnect_count = 0

        logger.info(f"[MessageProcessor] Initialized for URL: {websocket_url}")

    def set_queue_processor(self, queue_processor: "QueueProcessor"):
        """设置队列处理器引用"""
        self.queue_processor = queue_processor

    def set_websocket_client(self, websocket_client: "WebSocketClient"):
        """设置WebSocket客户端引用"""
        self.websocket_client = websocket_client

    def start(self) -> None:
        """启动消息处理线程"""
        if self.is_running:
            logger.warning("[MessageProcessor] Already running")
            return

        self.is_running = True
        self.thread = threading.Thread(target=self._run, daemon=True, name="MessageProcessor")
        self.thread.start()
        logger.info("[MessageProcessor] Started")

    def stop(self) -> None:
        """停止消息处理线程"""
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        logger.info("[MessageProcessor] Stopped")

    def _run(self):
        """运行消息处理主循环"""
        loop = asyncio.new_event_loop()
        try:
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._connection_handler())
        except Exception as e:
            logger.error(f"[MessageProcessor] Thread error: {str(e)}")
            logger.error(traceback.format_exc())
        finally:
            if loop:
                loop.close()

    async def _connection_handler(self):
        """处理WebSocket连接和重连逻辑"""
        while self.is_running:
            try:
                # 构建SSL上下文
                ssl_context = None
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

                    logger.info(f"[MessageProcessor] Connected to {self.websocket_url}")

                    # 启动发送协程
                    send_task = asyncio.create_task(self._send_handler())

                    try:
                        # 接收消息循环
                        await self._message_handler()
                    finally:
                        send_task.cancel()
                        try:
                            await send_task
                        except asyncio.CancelledError:
                            pass
                        self.connected = False

            except websockets.exceptions.ConnectionClosed:
                logger.warning("[MessageProcessor] Connection closed")
                self.connected = False
            except Exception as e:
                logger.error(f"[MessageProcessor] Connection error: {str(e)}")
                logger.error(traceback.format_exc())
                self.connected = False
            finally:
                self.websocket = None

            # 重连逻辑
            if self.is_running and self.reconnect_count < WSConfig.max_reconnect_attempts:
                self.reconnect_count += 1
                logger.info(
                    f"[MessageProcessor] Reconnecting in {WSConfig.reconnect_interval}s "
                    f"(attempt {self.reconnect_count}/{WSConfig.max_reconnect_attempts})"
                )
                await asyncio.sleep(WSConfig.reconnect_interval)
            elif self.reconnect_count >= WSConfig.max_reconnect_attempts:
                logger.error("[MessageProcessor] Max reconnection attempts reached")
                break
            else:
                self.reconnect_count -= 1

    async def _message_handler(self):
        """处理接收到的消息"""
        if not self.websocket:
            logger.error("[MessageProcessor] WebSocket connection is None")
            return

        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    await self._process_message(data)
                except json.JSONDecodeError:
                    logger.error(f"[MessageProcessor] Invalid JSON received: {message}")
                except Exception as e:
                    logger.error(f"[MessageProcessor] Error processing message: {str(e)}")
                    logger.error(traceback.format_exc())

        except websockets.exceptions.ConnectionClosed:
            logger.info("[MessageProcessor] Message handler stopped - connection closed")
        except Exception as e:
            logger.error(f"[MessageProcessor] Message handler error: {str(e)}")
            logger.error(traceback.format_exc())

    async def _send_handler(self):
        """处理发送队列中的消息"""
        logger.debug("[MessageProcessor] Send handler started")

        try:
            while self.connected and self.websocket:
                try:
                    # 从发送队列获取消息（非阻塞）
                    messages_to_send = []
                    max_batch = 10

                    while len(messages_to_send) < max_batch:
                        try:
                            message = self.send_queue.get_nowait()
                            messages_to_send.append(message)
                        except Empty:
                            break

                    if not messages_to_send:
                        await asyncio.sleep(0.1)
                        continue

                    # 批量发送消息
                    for msg in messages_to_send:
                        if not self.connected or not self.websocket:
                            break

                        try:
                            message_str = json.dumps(msg, ensure_ascii=False)
                            await self.websocket.send(message_str)
                            logger.trace(f"[MessageProcessor] Message sent: {msg.get('action', 'unknown')}")  # type: ignore  # noqa: E501
                        except Exception as e:
                            logger.error(f"[MessageProcessor] Failed to send message: {str(e)}")
                            logger.error(traceback.format_exc())
                            break

                    # 批量发送后短暂等待
                    if len(messages_to_send) > 5:
                        await asyncio.sleep(0.001)

                except Exception as e:
                    logger.error(f"[MessageProcessor] Error in send handler: {str(e)}")
                    logger.error(traceback.format_exc())
                    await asyncio.sleep(0.1)

        except asyncio.CancelledError:
            logger.debug("[MessageProcessor] Send handler cancelled")
        except Exception as e:
            logger.error(f"[MessageProcessor] Fatal error in send handler: {str(e)}")
            logger.error(traceback.format_exc())
        finally:
            logger.debug("[MessageProcessor] Send handler stopped")

    async def _process_message(self, data: Dict[str, Any]):
        """处理收到的消息"""
        message_type = data.get("action", "")
        message_data = data.get("data", {})

        logger.debug(f"[MessageProcessor] Processing message: {message_type}")

        try:
            if message_type == "pong":
                self._handle_pong(message_data)
            elif message_type == "query_action_state":
                await self._handle_query_action_state(message_data)
            elif message_type == "job_start":
                await self._handle_job_start(message_data)
            elif message_type == "cancel_action" or message_type == "cancel_task":
                await self._handle_cancel_action(message_data)
            elif message_type == "":
                return
            else:
                logger.debug(f"[MessageProcessor] Unknown message type: {message_type}")

        except Exception as e:
            logger.error(f"[MessageProcessor] Error processing message {message_type}: {str(e)}")
            logger.error(traceback.format_exc())

    def _handle_pong(self, pong_data: Dict[str, Any]):
        """处理pong响应"""
        host_node = HostNode.get_instance(0)
        if host_node:
            host_node.handle_pong_response(pong_data)

    async def _handle_query_action_state(self, data: Dict[str, Any]):
        """处理query_action_state消息"""
        device_id = data.get("device_id", "")
        action_name = data.get("action_name", "")
        task_id = data.get("task_id", "")
        job_id = data.get("job_id", "")

        if not all([device_id, action_name, task_id, job_id]):
            logger.error("[MessageProcessor] Missing required fields in query_action_state")
            return

        device_action_key = f"/devices/{device_id}/{action_name}"

        # 创建任务信息
        job_info = JobInfo(
            job_id=job_id,
            task_id=task_id,
            device_id=device_id,
            action_name=action_name,
            device_action_key=device_action_key,
            status=JobStatus.QUEUE,
            start_time=time.time(),
        )

        # 添加到设备管理器
        can_start_immediately = self.device_manager.add_queue_request(job_info)

        job_log = format_job_log(job_id, task_id, device_id, action_name)
        if can_start_immediately:
            # 可以立即开始
            await self._send_action_state_response(
                device_id, action_name, task_id, job_id, "query_action_status", True, 0
            )
            logger.info(f"[MessageProcessor] Job {job_log} can start immediately")
        else:
            # 需要排队
            await self._send_action_state_response(
                device_id, action_name, task_id, job_id, "query_action_status", False, 10
            )
            logger.info(f"[MessageProcessor] Job {job_log} queued")

            # 通知QueueProcessor有新的队列更新
            if self.queue_processor:
                self.queue_processor.notify_queue_update()

    async def _handle_job_start(self, data: Dict[str, Any]):
        """处理job_start消息"""
        try:
            req = JobAddReq(**data)

            job_log = format_job_log(req.job_id, req.task_id, req.device_id, req.action)
            success = self.device_manager.start_job(req.job_id)
            if not success:
                logger.error(f"[MessageProcessor] Failed to start job {job_log}")
                return

            logger.info(f"[MessageProcessor] Starting job {job_log}")

            # 创建HostNode任务
            device_action_key = f"/devices/{req.device_id}/{req.action}"
            queue_item = QueueItem(
                task_type="job_call_back_status",
                device_id=req.device_id,
                action_name=req.action,
                task_id=req.task_id,
                job_id=req.job_id,
                device_action_key=device_action_key,
            )

            # 提交给HostNode执行
            host_node = HostNode.get_instance(0)
            if not host_node:
                logger.error(f"[MessageProcessor] HostNode instance not available for job_id: {req.job_id}")
                return

            host_node.send_goal(
                queue_item,
                action_type=req.action_type,
                action_kwargs=req.action_args,
                server_info=req.server_info,
            )

        except Exception as e:
            logger.error(f"[MessageProcessor] Error handling job start: {str(e)}")
            traceback.print_exc()

            # job_start出错时，需要通过正确的publish_job_status方法来处理
            if "req" in locals() and "queue_item" in locals():
                job_log = format_job_log(req.job_id, req.task_id, req.device_id, req.action)
                logger.info(f"[MessageProcessor] Publishing failed status for job {job_log}")

                if self.websocket_client:
                    # 使用完整的错误信息，与原版本一致
                    self.websocket_client.publish_job_status(
                        {}, queue_item, "failed", serialize_result_info(traceback.format_exc(), False, {})
                    )
                else:
                    # 备用方案：直接发送消息，但使用完整的错误信息
                    message = {
                        "action": "job_status",
                        "data": {
                            "job_id": req.job_id,
                            "task_id": req.task_id,
                            "device_id": req.device_id,
                            "action_name": req.action,
                            "status": "failed",
                            "feedback_data": {},
                            "return_info": serialize_result_info(traceback.format_exc(), False, {}),
                            "timestamp": time.time(),
                        },
                    }
                    self.send_message(message)

                    # 手动调用job结束逻辑
                    next_job = self.device_manager.end_job(req.job_id)
                    if next_job:
                        # 通知下一个任务可以开始
                        await self._send_action_state_response(
                            next_job.device_id,
                            next_job.action_name,
                            next_job.task_id,
                            next_job.job_id,
                            "query_action_status",
                            True,
                            0,
                        )
                        next_job_log = format_job_log(
                            next_job.job_id, next_job.task_id, next_job.device_id, next_job.action_name
                        )
                        logger.info(f"[MessageProcessor] Started next job {next_job_log} after error")

                        # 通知QueueProcessor有队列更新
                        if self.queue_processor:
                            self.queue_processor.notify_queue_update()
            else:
                logger.warning("[MessageProcessor] Failed to publish job error status - missing req or queue_item")

    async def _handle_cancel_action(self, data: Dict[str, Any]):
        """处理cancel_action/cancel_task消息"""
        task_id = data.get("task_id")
        job_id = data.get("job_id")

        logger.info(f"[MessageProcessor] Cancel request - task_id: {task_id}, job_id: {job_id}")

        if job_id:
            # 获取job信息用于日志
            job_info = self.device_manager.get_job_info(job_id)
            job_log = format_job_log(
                job_id,
                job_info.task_id if job_info else "",
                job_info.device_id if job_info else "",
                job_info.action_name if job_info else "",
            )

            # 按job_id取消单个job
            success = self.device_manager.cancel_job(job_id)
            if success:
                # 通知HostNode取消
                host_node = HostNode.get_instance(0)
                if host_node:
                    host_node.cancel_goal(job_id)
                logger.info(f"[MessageProcessor] Job {job_log} cancelled")

                # 通知QueueProcessor有队列更新
                if self.queue_processor:
                    self.queue_processor.notify_queue_update()
            else:
                logger.warning(f"[MessageProcessor] Failed to cancel job {job_log}")

        elif task_id:
            # 按task_id取消所有相关job
            cancelled_job_ids = self.device_manager.cancel_jobs_by_task_id(task_id)
            if cancelled_job_ids:
                # 通知HostNode取消所有job
                host_node = HostNode.get_instance(0)
                if host_node:
                    for cancelled_job_id in cancelled_job_ids:
                        host_node.cancel_goal(cancelled_job_id)

                logger.info(f"[MessageProcessor] Cancelled {len(cancelled_job_ids)} jobs for task_id: {task_id}")

                # 通知QueueProcessor有队列更新
                if self.queue_processor:
                    self.queue_processor.notify_queue_update()
            else:
                logger.warning(f"[MessageProcessor] Failed to cancel any jobs for task_id: {task_id}")
        else:
            logger.warning("[MessageProcessor] Cancel request missing both task_id and job_id")

    async def _send_action_state_response(
        self, device_id: str, action_name: str, task_id: str, job_id: str, typ: str, free: bool, need_more: int
    ):
        """发送动作状态响应"""
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

        try:
            self.send_queue.put_nowait(message)
        except Exception:
            logger.warning("[MessageProcessor] Send queue full, dropping message")

    def send_message(self, message: Dict[str, Any]) -> bool:
        """发送消息到队列"""
        try:
            self.send_queue.put_nowait(message)
            return True
        except Exception:
            logger.warning(f"[MessageProcessor] Failed to queue message: {message.get('action', 'unknown')}")
            return False

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.connected


class QueueProcessor:
    """队列处理线程 - 定时给发送队列推送消息，管理任务状态"""

    def __init__(self, device_manager: DeviceActionManager, message_processor: MessageProcessor):
        self.device_manager = device_manager
        self.message_processor = message_processor
        self.websocket_client = None  # 延迟设置

        # 线程控制
        self.is_running = False
        self.thread = None

        # 事件通知机制
        self.queue_update_event = threading.Event()

        logger.info("[QueueProcessor] Initialized")

    def set_websocket_client(self, websocket_client: "WebSocketClient"):
        """设置WebSocket客户端引用"""
        self.websocket_client = websocket_client

    def start(self) -> None:
        """启动队列处理线程"""
        if self.is_running:
            logger.warning("[QueueProcessor] Already running")
            return

        self.is_running = True
        self.thread = threading.Thread(target=self._run, daemon=True, name="QueueProcessor")
        self.thread.start()
        logger.info("[QueueProcessor] Started")

    def stop(self) -> None:
        """停止队列处理线程"""
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        logger.info("[QueueProcessor] Stopped")

    def _run(self):
        """运行队列处理主循环"""
        logger.debug("[QueueProcessor] Queue processor started")

        while self.is_running:
            try:
                # 检查READY状态超时的任务
                timeout_jobs = self.device_manager.check_ready_timeouts()
                if timeout_jobs:
                    logger.info(f"[QueueProcessor] Found {len(timeout_jobs)} READY jobs that timed out")
                    # 为超时的job发布失败状态，通过正常job完成流程处理
                    for timeout_job in timeout_jobs:
                        timeout_item = QueueItem(
                            task_type="job_call_back_status",
                            device_id=timeout_job.device_id,
                            action_name=timeout_job.action_name,
                            task_id=timeout_job.task_id,
                            job_id=timeout_job.job_id,
                            device_action_key=timeout_job.device_action_key,
                        )
                        # 发布超时失败状态，这会触发正常的job完成流程
                        if self.websocket_client:
                            job_log = format_job_log(
                                timeout_job.job_id, timeout_job.task_id, timeout_job.device_id, timeout_job.action_name
                            )
                            logger.info(f"[QueueProcessor] Publishing timeout failure for job {job_log}")
                            self.websocket_client.publish_job_status(
                                {},
                                timeout_item,
                                "failed",
                                serialize_result_info("Job READY state timeout after 10 seconds", False, {}),
                            )

                    # 立即触发状态更新
                    self.notify_queue_update()

                # 发送正在执行任务的running状态
                self._send_running_status()

                # 发送排队任务的busy状态
                self._send_busy_status()

                # 等待10秒或者等待事件通知
                self.queue_update_event.wait(timeout=10)
                self.queue_update_event.clear()  # 清除事件

            except Exception as e:
                logger.error(f"[QueueProcessor] Error in queue processor: {str(e)}")
                logger.error(traceback.format_exc())
                time.sleep(1)

        logger.debug("[QueueProcessor] Queue processor stopped")

    def notify_queue_update(self):
        """通知队列有更新，触发立即检查"""
        self.queue_update_event.set()

    def _send_running_status(self):
        """发送正在执行任务的running状态"""
        if not self.message_processor.is_connected():
            return

        active_jobs = self.device_manager.get_active_jobs()
        for job_info in active_jobs:
            # 只给真正在执行的job发送running状态，READY状态的job不需要
            if job_info.status != JobStatus.STARTED:
                continue

            message = {
                "action": "report_action_state",
                "data": {
                    "type": "job_call_back_status",
                    "device_id": job_info.device_id,
                    "action_name": job_info.action_name,
                    "task_id": job_info.task_id,
                    "job_id": job_info.job_id,
                    "free": False,
                    "need_more": 10,
                },
            }
            self.message_processor.send_message(message)
            job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
            logger.trace(f"[QueueProcessor] Sent running status for job {job_log}")  # type: ignore

    def _send_busy_status(self):
        """发送排队任务的busy状态"""
        if not self.message_processor.is_connected():
            return

        queued_jobs = self.device_manager.get_queued_jobs()
        if not queued_jobs:
            return

        logger.debug(f"[QueueProcessor] Sending busy status for {len(queued_jobs)} queued jobs")

        for job_info in queued_jobs:
            message = {
                "action": "report_action_state",
                "data": {
                    "type": "query_action_status",
                    "device_id": job_info.device_id,
                    "action_name": job_info.action_name,
                    "task_id": job_info.task_id,
                    "job_id": job_info.job_id,
                    "free": False,
                    "need_more": 10,
                },
            }
            success = self.message_processor.send_message(message)
            job_log = format_job_log(job_info.job_id, job_info.task_id, job_info.device_id, job_info.action_name)
            if success:
                logger.debug(f"[QueueProcessor] Sent busy/need_more for queued job {job_log}")
            else:
                logger.warning(f"[QueueProcessor] Failed to send busy status for job {job_log}")

    def handle_job_completed(self, job_id: str, status: str) -> None:
        """处理任务完成"""
        # 获取job信息用于日志
        job_info = self.device_manager.get_job_info(job_id)
        job_log = format_job_log(
            job_id,
            job_info.task_id if job_info else "",
            job_info.device_id if job_info else "",
            job_info.action_name if job_info else "",
        )

        logger.info(f"[QueueProcessor] Job {job_log} completed with status: {status}")

        # 结束任务，获取下一个可执行的任务
        next_job = self.device_manager.end_job(job_id)

        if next_job and self.message_processor.is_connected():
            # 通知下一个任务可以开始
            message = {
                "action": "report_action_state",
                "data": {
                    "type": "query_action_status",
                    "device_id": next_job.device_id,
                    "action_name": next_job.action_name,
                    "task_id": next_job.task_id,
                    "job_id": next_job.job_id,
                    "free": True,
                    "need_more": 0,
                },
            }
            self.message_processor.send_message(message)
            next_job_log = format_job_log(next_job.job_id, next_job.task_id, next_job.device_id, next_job.action_name)
            logger.info(f"[QueueProcessor] Notified next job {next_job_log} can start")

            # 立即触发下一轮状态检查
            self.notify_queue_update()


class WebSocketClient(BaseCommunicationClient):
    """
    重构后的WebSocket客户端 v2

    采用两线程架构：
    - 消息处理线程：处理WebSocket消息，划分任务执行和任务队列
    - 队列处理线程：定时给发送队列推送消息，管理任务状态
    """

    def __init__(self):
        super().__init__()
        self.is_disabled = False
        self.client_id = f"{uuid.uuid4()}"

        # 核心组件
        self.device_manager = DeviceActionManager()
        self.send_queue = Queue(maxsize=1000)

        # 构建WebSocket URL
        self.websocket_url = self._build_websocket_url()

        # 两个核心线程
        self.message_processor = MessageProcessor(self.websocket_url, self.send_queue, self.device_manager)
        self.queue_processor = QueueProcessor(self.device_manager, self.message_processor)

        # 设置相互引用
        self.message_processor.set_queue_processor(self.queue_processor)
        self.message_processor.set_websocket_client(self)
        self.queue_processor.set_websocket_client(self)

        logger.info(f"[WebSocketClient] Client_id: {self.client_id}")

    def _build_websocket_url(self) -> Optional[str]:
        """构建WebSocket连接URL"""
        if not HTTPConfig.remote_addr:
            return None

        parsed = urlparse(HTTPConfig.remote_addr)

        if parsed.scheme == "https":
            scheme = "wss"
        else:
            scheme = "ws"

        if ":" in parsed.netloc and parsed.port is not None:
            url = f"{scheme}://{parsed.hostname}:{parsed.port + 1}/api/v1/ws/schedule"
        else:
            url = f"{scheme}://{parsed.netloc}/api/v1/ws/schedule"

        logger.debug(f"[WebSocketClient] URL: {url}")
        return url

    def start(self) -> None:
        """启动WebSocket客户端"""
        if self.is_disabled:
            logger.warning("[WebSocketClient] WebSocket is disabled, skipping connection.")
            return

        if not self.websocket_url:
            logger.error("[WebSocketClient] WebSocket URL not configured")
            return

        logger.info(f"[WebSocketClient] Starting connection to {self.websocket_url}")

        # 启动两个核心线程
        self.message_processor.start()
        self.queue_processor.start()

        logger.info("[WebSocketClient] All threads started")

    def stop(self) -> None:
        """停止WebSocket客户端"""
        if self.is_disabled:
            return

        logger.info("[WebSocketClient] Stopping connection")

        # 停止两个核心线程
        self.message_processor.stop()
        self.queue_processor.stop()

        logger.info("[WebSocketClient] All threads stopped")

    # BaseCommunicationClient接口实现
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.message_processor.is_connected() and not self.is_disabled

    def publish_device_status(self, device_status: dict, device_id: str, property_name: str) -> None:
        """发布设备状态"""
        if self.is_disabled or not self.is_connected():
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
        self.message_processor.send_message(message)
        logger.trace(f"[WebSocketClient] Device status published: {device_id}.{property_name}")

    def publish_job_status(
        self, feedback_data: dict, item: QueueItem, status: str, return_info: Optional[dict] = None
    ) -> None:
        """发布作业状态，拦截最终结果（给HostNode调用的接口）"""
        if not self.is_connected():
            logger.debug(f"[WebSocketClient] Not connected, cannot publish job status for job_id: {item.job_id}")
            return

        # 拦截最终结果状态，与原版本逻辑一致
        if status in ["success", "failed"]:
            host_node = HostNode.get_instance(0)
            if host_node:
                # 从HostNode的device_action_status中移除job_id
                try:
                    host_node._device_action_status[item.device_action_key].job_ids.pop(item.job_id, None)
                except (KeyError, AttributeError):
                    logger.warning(f"[WebSocketClient] Failed to remove job {item.job_id} from HostNode status")

            logger.info(f"[WebSocketClient] Intercepting final status for job_id: {item.job_id} - {status}")

            # 通知队列处理器job完成（包括timeout的job）
            self.queue_processor.handle_job_completed(item.job_id, status)

        # 发送job状态消息
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
        self.message_processor.send_message(message)

        job_log = format_job_log(item.job_id, item.task_id, item.device_id, item.action_name)
        logger.debug(f"[WebSocketClient] Job status published: {job_log} - {status}")

    def send_ping(self, ping_id: str, timestamp: float) -> None:
        """发送ping消息"""
        if self.is_disabled or not self.is_connected():
            logger.warning("[WebSocketClient] Not connected, cannot send ping")
            return

        message = {"action": "ping", "data": {"ping_id": ping_id, "client_timestamp": timestamp}}
        self.message_processor.send_message(message)
        logger.debug(f"[WebSocketClient] Ping sent: {ping_id}")

    def cancel_goal(self, job_id: str) -> None:
        """取消指定的任务"""
        # 获取job信息用于日志
        job_info = self.device_manager.get_job_info(job_id)
        job_log = format_job_log(
            job_id,
            job_info.task_id if job_info else "",
            job_info.device_id if job_info else "",
            job_info.action_name if job_info else "",
        )

        logger.debug(f"[WebSocketClient] Cancel goal request for job: {job_log}")
        success = self.device_manager.cancel_job(job_id)
        if success:
            logger.info(f"[WebSocketClient] Job {job_log} cancelled successfully")
        else:
            logger.warning(f"[WebSocketClient] Failed to cancel job {job_log}")
