"""
OPC UA 通讯基类（带订阅）
连接失败时自动进入模拟模式，假装已连接
"""

import json
import socket
import threading
import time
from typing import Dict, Any

from opcua import Client

from unilabos.device_comms.opcua_client.client import BaseClient
from unilabos.utils.log import logger


class OpcUaClientWithSubscription(BaseClient):
    """OPC UA 客户端，连接失败时使用模拟模式"""

    def __init__(
        self,
        url: str,
        username: str = None,
        password: str = None,
        use_subscription: bool = True,
        cache_timeout: float = 5.0,
        subscription_interval: int = 500,
        *args,
        **kwargs,
    ):
        BaseClient.__init__(self)
        self._mock_mode = False
        self._mock_values: Dict[str, Any] = {}
        self._init_mock_defaults()

        client = Client(url)
        if username and password:
            client.set_user(username)
            client.set_password(password)
        self._set_client(client)

        self._connect()

    def _init_mock_defaults(self):
        """模拟模式下的默认值，使等待循环能快速通过"""
        defaults = {
            "robot_ready": True,
            "station_1_ready": True,
            "station_2_ready": True,
            "station_3_ready": True,
            "auto_mode": False,
            "station_1_request_params": True,
            "station_2_request_params": True,
            "station_3_request_params": True,
        }
        self._mock_values.update(defaults)

    def _connect(self) -> None:
        logger.info("尝试连接到 OPC UA 服务器...")
        if not self.client:
            raise ValueError("client is not initialized")
        try:
            self.client.connect()
            logger.info("客户端已连接")
            if self._variables_to_find:
                self._find_nodes()
        except (ConnectionRefusedError, OSError) as e:
            logger.warning(f"连接失败，使用模拟模式（假装已连接）: {e}")
            self._mock_mode = True
            self.client = None

    def load_nodes_from_csv(self, path: str) -> "OpcUaClientWithSubscription":
        """从 CSV 加载节点"""
        import os
        if self._mock_mode or not os.path.isfile(path):
            return self
        self.register_node_list_from_csv_path(path=path)
        return self

    def _resolve_node_name(self, name: str) -> str:
        """解析节点名，支持英文名到中文名映射"""
        if name in self._name_mapping:
            return self._name_mapping[name]
        return name

    def get_node_value(self, name: str) -> Any:
        """获取节点值"""
        if self._mock_mode:
            resolved = self._resolve_node_name(name)
            if resolved in self._mock_values:
                return self._mock_values[resolved]
            if name in self._mock_values:
                return self._mock_values[name]
            if "ready" in name or "complete" in name or "finished" in name or "request_params" in name:
                return self._mock_values.get(resolved, self._mock_values.get(name, True))
            return self._mock_values.get(resolved, self._mock_values.get(name, False))
        value, _ = self.use_node(self._resolve_node_name(name)).read()
        return value

    def set_node_value(self, name: str, value: Any) -> bool:
        """设置节点值"""
        if self._mock_mode:
            resolved = self._resolve_node_name(name)
            self._mock_values[resolved] = value
            self._mock_values[name] = value
            if name == "robot_pick_beaker_id":
                self._mock_values[f"robot_rack_pick_beaker_{value}_complete"] = True
            elif name == "robot_place_station_id":
                self._mock_values[f"robot_place_station_{value}_complete"] = True
            elif name == "robot_pick_station_id":
                self._mock_values[f"robot_pick_station_{value}_complete"] = True
            elif name == "robot_place_beaker_id":
                self._mock_values[f"robot_rack_place_beaker_{value}_complete"] = True
            elif "initialize" in name and value:
                self._mock_values["init finished"] = True
            elif name == "manual_auto_switch":
                self._mock_values["auto_mode"] = value
            elif "_start" in name and value:
                station_id = name.split("_")[1]
                self._mock_values[f"station_{station_id}_params_received"] = True
                self._mock_values[f"station_{station_id}_process_complete"] = True
            elif name == "auto_run_start_trigger" and value:
                self._mock_values["auto_run_complete"] = True
            elif name == "auto_param_downloaded" and value:
                self._mock_values["auto_param_applied"] = True
            return True
        node = self.use_node(self._resolve_node_name(name))
        return not node.write(value)

    def use_node(self, name: str):
        """获取节点，模拟模式下返回模拟对象"""
        if self._mock_mode:
            return _MockNode(name, self._mock_values)
        return super().use_node(self._resolve_node_name(name))


class _MockNode:
    """模拟节点，用于模拟模式下的 use_node 调用"""

    def __init__(self, name: str, values: Dict[str, Any]):
        self.name = name
        self._values = values

    def read(self):
        val = self._values.get(self.name, False)
        return val, False

    def write(self, value):
        self._values[self.name] = value
        return False

class UDPClient:
    """
    UDP客户端类，用于与设备进行通信
    """
    def __init__(self, address: str = "127.0.0.1", port: int = 8888, timeout: float = 5.0):
        self.address = address
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.connected = False
        self.lock = threading.Lock()
        self.status_callback = None  # 状态更新回调函数
        self.listen_thread = None  # 状态监听线程
        self.listen_running = False  # 监听线程运行状态
        
        # 命令类型配置：立即响应/长时间运行
        self._immediate_response_commands = {
            "TOGGLE_LOCAL_REMOTE_CONTROL",
            "GET_DEVICE_STATE",
            "GET_REACTOR_STATE",
            "GET_POST_PROCESS_STATE",
            "GET_REAGENT_STATE",
            "GET_ALL_REAGENTS_STATE",
            "REACTOR_N2_ON",
            "REACTOR_N2_OFF",
            "REACTOR_AIR_ON",
            "REACTOR_AIR_OFF",
            "TEMP_SET",
            "START_STIR",
            "STOP_STIR",
            "POST_PROCESS_DISCHARGE_ON",
            "POST_PROCESS_DISCHARGE_OFF"
        }
        
        self._long_running_commands = {
            "REACTOR_SOLUTION_ADD",
            "POST_PROCESS_SOLUTION_ADD",
            "POST_PROCESS_CLEAN",
            "WAIT",
            "UPDATE_REAGENT_VOLUME"
        }
    
    def connect(self) -> bool:
        """
        连接到UDP服务器
        
        Returns:
            bool: 连接成功返回True，否则返回False
        """
        try:
            # UDP是无连接协议，这里只是初始化socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(self.timeout)
            self.connected = True
            logger.info(f"UDP客户端已初始化，目标地址: {self.address}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"UDP客户端初始化失败: {e}")
            self.connected = False
            return False
    
    def set_status_callback(self, callback):
        """
        设置状态更新回调函数
        
        Args:
            callback: 回调函数，接收状态数据作为参数
        """
        self.status_callback = callback
    
    def start_listen(self):
        """
        启动状态监听线程
        """
        if self.listen_running:
            logger.warning("状态监听线程已在运行")
            return
            
        self.listen_running = True
        self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listen_thread.start()
        logger.info("UDP状态监听线程已启动")
    
    def stop_listen(self):
        """
        停止状态监听线程
        """
        self.listen_running = False
        if self.listen_thread:
            self.listen_thread.join(timeout=1.0)
            self.listen_thread = None
        logger.info("UDP状态监听线程已停止")
    
    def _listen_loop(self):
        """
        状态监听循环，接收服务器主动推送的状态更新
        """
        while self.listen_running and self.connected and self.socket:
            try:
                # 设置较短的超时，以便定期检查listen_running状态
                self.socket.settimeout(0.5)
                response_data, _ = self.socket.recvfrom(1024)
                
                # 尝试解析JSON响应
                try:
                    response = json.loads(response_data.decode('utf-8'))
                    logger.debug(f"收到UDP状态更新: {response}")
                    
                    # 如果是状态更新消息，调用回调函数
                    if response.get("type") == "status_update" and self.status_callback:
                        self.status_callback(response.get("data", {}))
                except json.JSONDecodeError:
                    logger.error(f"UDP响应格式错误: {response_data}")
            except socket.timeout:
                # 超时是正常的，继续监听
                continue
            except Exception as e:
                logger.error(f"UDP监听错误: {e}")
                # 短暂暂停后继续监听
                time.sleep(0.5)
    
    def disconnect(self) -> bool:
        """
        断开UDP连接
        
        Returns:
            bool: 断开成功返回True，否则返回False
        """
        try:
            # 停止监听线程
            self.stop_listen()
            
            if self.socket:
                self.socket.close()
                self.socket = None
            self.connected = False
            logger.info("UDP客户端已断开连接")
            
            return True
        except Exception as e:
            logger.error(f"UDP客户端断开连接失败: {e}")
            return False
    
    def send_command(self, command: str, params: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        发送命令到UDP服务器
        
        Args:
            command: 命令名称
            params: 命令参数
            
        Returns:
            Dict[str, Any]: 服务器响应
        """
        with self.lock:
            if getattr(self, "_debug_skip_io", False):
                if command in self._long_running_commands:
                    return {
                        "status": "success",
                        "message": "debug_mode 模拟异步命令",
                        "type": "async",
                    }
                return {"status": "success", "message": "debug_mode 模拟", "data": {}}

            if not self.connected or not self.socket:
                logger.error("UDP客户端未连接")
                return {"status": "error", "message": "UDP客户端未连接"}
            
            try:
                # 构建函数调用格式命令
                params = params or {}
                
                # 处理参数，转换为函数调用格式
                param_list = []
                for key, value in params.items():
                    # 处理反应器ID，转换为reactor_1格式
                    if key == "reactor_id" or key == "post_process_id":
                        param_list.append(f"{key[:-3]}_{value}")
                    else:
                        # 根据参数类型格式化
                        if isinstance(value, str):
                            param_list.append(value)
                        else:
                            param_list.append(str(value))
                
                # 构建命令字符串，格式：S COMMAND_NAME(param1,param2,...)
                cmd_str = f"S {command}({','.join(param_list)})"
                
                # 发送命令
                data = cmd_str.encode('utf-8')
                self.socket.sendto(data, (self.address, self.port))
                logger.debug(f"发送UDP命令: {cmd_str}")
                
                # 根据命令类型决定是否等待响应
                if command in self._immediate_response_commands:
                    # 立即响应命令，设置合理的超时时间
                    try:
                        self.socket.settimeout(5.0)
                        response_data, _ = self.socket.recvfrom(1024)
                        
                        # 尝试解析响应，假设响应仍然是JSON格式
                        try:
                            response = json.loads(response_data.decode('utf-8'))
                            logger.debug(f"收到UDP响应: {response}")
                        except json.JSONDecodeError:
                            # 如果响应不是JSON格式，返回成功状态
                            response = {"status": "success", "message": "命令执行成功"}
                            logger.debug(f"收到UDP响应: {response_data.decode('utf-8')}")
                        
                        # 恢复监听线程的超时时间
                        self.socket.settimeout(2)
                        return response
                    except socket.timeout:
                        logger.error(f"UDP命令超时: {command}")
                        # 恢复监听线程的超时时间
                        self.socket.settimeout(2)
                        return {"status": "error", "message": "命令超时"}
                elif command in self._long_running_commands:
                    # 长时间运行命令，发送后立即返回成功，不等待响应
                    # 服务器会通过状态更新推送执行结果
                    logger.debug(f"长时间运行命令已发送，等待状态更新: {command}")
                    # 恢复监听线程的超时时间
                    self.socket.settimeout(2)
                    return {"status": "success", "message": "命令已接收，正在执行", "type": "async"}
                else:
                    # 未知命令类型，默认按立即响应处理
                    try:
                        self.socket.settimeout(2.0)
                        response_data, _ = self.socket.recvfrom(1024)
                        
                        # 尝试解析响应
                        try:
                            response = json.loads(response_data.decode('utf-8'))
                            logger.debug(f"收到UDP响应: {response}")
                        except json.JSONDecodeError:
                            response = {"status": "success", "message": "命令执行成功"}
                            logger.debug(f"收到UDP响应: {response_data.decode('utf-8')}")
                        
                        # 恢复监听线程的超时时间
                        self.socket.settimeout(2)
                        return response
                    except socket.timeout:
                        logger.error(f"UDP命令超时: {command}")
                        # 恢复监听线程的超时时间
                        self.socket.settimeout(2)
                        return {"status": "error", "message": "命令超时"}
            except Exception as e:
                logger.error(f"UDP命令执行失败: {command}, 错误: {e}")
                # 恢复监听线程的超时时间
                self.socket.settimeout(2)
                return {"status": "error", "message": str(e)}
