"""
OPC UA 通讯基类（带订阅）
连接失败时自动进入模拟模式，假装已连接
"""

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

