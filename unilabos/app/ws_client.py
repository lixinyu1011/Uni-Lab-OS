#!/usr/bin/env python
# coding=utf-8
"""
WebSocket通信客户端

基于WebSocket协议的通信客户端实现，继承自BaseCommunicationClient。
"""

import json
import time
import uuid
import threading
import asyncio
import traceback
from typing import Optional, Dict, Any
from urllib.parse import urlparse
from unilabos.app.controler import job_add
from unilabos.app.model import JobAddReq
from unilabos.ros.nodes.presets.host_node import HostNode

try:
    import websockets
    import ssl as ssl_module

    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

from unilabos.app.communication import BaseCommunicationClient
from unilabos.config.config import WSConfig, HTTPConfig, BasicConfig
from unilabos.utils import logger


class WebSocketClient(BaseCommunicationClient):
    """
    WebSocket通信客户端类

    实现基于WebSocket协议的实时通信功能。
    """

    def __init__(self):
        super().__init__()

        if not HAS_WEBSOCKETS:
            logger.error("[WebSocket] websockets库未安装，WebSocket功能不可用")
            self.is_disabled = True
            return

        self.is_disabled = False
        self.client_id = f"{uuid.uuid4()}"

        # WebSocket连接相关
        self.websocket = None
        self.connection_loop = None
        self.event_loop = None
        self.connection_thread = None
        self.is_running = False
        self.connected = False

        # 消息处理
        self.message_queue = asyncio.Queue() if not self.is_disabled else None
        self.reconnect_count = 0

        # 构建WebSocket URL
        self._build_websocket_url()

        logger.info(f"[WebSocket] Client_id: {self.client_id}")

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
        self.websocket_url = f"{scheme}://{parsed.netloc}/api/v1/lab"

        logger.debug(f"[WebSocket] URL: {self.websocket_url}")

    def start(self) -> None:
        """启动WebSocket连接"""
        if self.is_disabled:
            logger.warning("[WebSocket] WebSocket is disabled, skipping connection.")
            return

        if not self.websocket_url:
            logger.error("[WebSocket] WebSocket URL not configured")
            return

        logger.info(f"[WebSocket] Starting connection to {self.websocket_url}")

        self.is_running = True

        # 在单独线程中运行WebSocket连接
        self.connection_thread = threading.Thread(target=self._run_connection, daemon=True, name="WebSocketConnection")
        self.connection_thread.start()

    def stop(self) -> None:
        """停止WebSocket连接"""
        if self.is_disabled:
            return

        logger.info("[WebSocket] Stopping connection")
        self.is_running = False
        self.connected = False

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

                async with websockets.connect(
                    self.websocket_url,
                    ssl=ssl_context,
                    ping_interval=WSConfig.ping_interval,
                    ping_timeout=10,
                    additional_headers={"Authorization": f"Bearer {BasicConfig.auth_secret()}"},
                ) as websocket:
                    self.websocket = websocket
                    self.connected = True
                    self.reconnect_count = 0

                    logger.info(f"[WebSocket] Connected to {self.websocket_url}")
                    # 处理消息
                    await self._message_handler()

            except websockets.exceptions.ConnectionClosed:
                logger.warning("[WebSocket] Connection closed")
                self.connected = False
            except Exception as e:
                logger.error(f"[WebSocket] Connection error: {str(e)}")
                self.connected = False

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

    async def _close_connection(self):
        """关闭WebSocket连接"""
        if self.websocket:
            await self.websocket.close()
            self.websocket = None

    async def _send_message(self, message: Dict[str, Any]):
        """发送消息"""
        if not self.connected or not self.websocket:
            logger.warning("[WebSocket] Not connected, cannot send message")
            return

        try:
            message_str = json.dumps(message, ensure_ascii=False)
            await self.websocket.send(message_str)
            logger.debug(f"[WebSocket] Message sent: {message['type']}")
        except Exception as e:
            logger.error(f"[WebSocket] Failed to send message: {str(e)}")

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
        message_type = input_message.get("type", "")
        data = input_message.get("data", {})
        if message_type == "job_start":
            # 处理作业启动消息
            await self._handle_job_start(data)
        elif message_type == "pong":
            # 处理pong响应
            self._handle_pong_sync(data)
        else:
            logger.debug(f"[WebSocket] Unknown message type: {message_type}")

    async def _handle_job_start(self, data: Dict[str, Any]):
        """处理作业启动消息"""
        try:
            job_req = JobAddReq(**data.get("job_data", {}))
            job_add(job_req)
            job_id = getattr(job_req, "id", "unknown")
            logger.info(f"[WebSocket] Job started: {job_id}")
        except Exception as e:
            logger.error(f"[WebSocket] Error handling job start: {str(e)}")

    def _handle_pong_sync(self, pong_data: Dict[str, Any]):
        """同步处理pong响应"""
        host_node = HostNode.get_instance(0)
        if host_node:
            host_node.handle_pong_response(pong_data)

    # 实现抽象基类的方法
    def publish_device_status(self, device_status: dict, device_id: str, property_name: str) -> None:
        """发布设备状态"""
        if self.is_disabled or not self.connected:
            return
        message = {
            "type": "device_status",
            "data": {
                "device_id": device_id,
                "property_name": property_name,
                "status": device_status.get(device_id, {}).get(property_name),
                "timestamp": time.time(),
            },
        }
        if self.event_loop:
            asyncio.run_coroutine_threadsafe(self._send_message(message), self.event_loop)
        logger.debug(f"[WebSocket] Device status published: {device_id}.{property_name}")

    def publish_job_status(
        self, feedback_data: dict, job_id: str, status: str, return_info: Optional[str] = None
    ) -> None:
        """发布作业状态"""
        if self.is_disabled or not self.connected:
            logger.warning("[WebSocket] Not connected, cannot publish job status")
            return
        message = {
            "type": "job_status",
            "data": {
                "job_id": job_id,
                "status": status,
                "feedback_data": feedback_data,
                "return_info": return_info,
                "timestamp": time.time(),
            },
        }
        if self.event_loop:
            asyncio.run_coroutine_threadsafe(self._send_message(message), self.event_loop)
        logger.debug(f"[WebSocket] Job status published: {job_id} - {status}")

    def send_ping(self, ping_id: str, timestamp: float) -> None:
        """发送ping消息"""
        if self.is_disabled or not self.connected:
            logger.warning("[WebSocket] Not connected, cannot send ping")
            return
        message = {"type": "ping", "data": {"ping_id": ping_id, "client_timestamp": timestamp}}
        if self.event_loop:
            asyncio.run_coroutine_threadsafe(self._send_message(message), self.event_loop)
        logger.debug(f"[WebSocket] Ping sent: {ping_id}")

    @property
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.connected and not self.is_disabled
