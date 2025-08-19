"""
工作站通信基类
Workstation Communication Base Class

从具体设备驱动中抽取通用通信模式
"""
import json
import time
import threading
from typing import Dict, Any, Optional, Callable, Union, List
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from unilabos.device_comms.modbus_plc.client import TCPClient as ModbusTCPClient
from unilabos.device_comms.modbus_plc.node.modbus import DataType, WorderOrder
from unilabos.utils.log import logger


class CommunicationProtocol(Enum):
    """通信协议类型"""
    MODBUS_TCP = "modbus_tcp"
    MODBUS_RTU = "modbus_rtu"
    SERIAL = "serial"
    ETHERNET = "ethernet"


@dataclass
class CommunicationConfig:
    """通信配置"""
    protocol: CommunicationProtocol
    host: str
    port: int
    timeout: float = 5.0
    retry_count: int = 3
    extra_params: Dict[str, Any] = None


class WorkstationCommunicationBase(ABC):
    """工作站通信基类
    
    定义工作站通信的标准接口：
    1. 状态查询 - 定期获取设备状态
    2. 命令下发 - 发送控制指令
    3. 数据采集 - 收集生产数据
    4. 紧急控制 - 单点调试控制
    """

    def __init__(self, communication_config: CommunicationConfig):
        self.config = communication_config
        self.client = None
        self.is_connected = False
        self.last_status = {}
        self.data_export_thread = None
        self.data_export_running = False
        
        # 状态缓存
        self._status_cache = {}
        self._last_update_time = 0
        self._cache_timeout = 1.0  # 缓存1秒
        
        self._initialize_communication()

    @abstractmethod
    def _initialize_communication(self):
        """初始化通信连接"""
        pass

    @abstractmethod
    def _load_address_mapping(self) -> Dict[str, Any]:
        """加载地址映射表"""
        pass

    def connect(self) -> bool:
        """建立连接"""
        try:
            if self.config.protocol == CommunicationProtocol.MODBUS_TCP:
                self.client = ModbusTCPClient(
                    addr=self.config.host,
                    port=self.config.port
                )
                self.client.client.connect()
                
                # 等待连接建立
                count = 100
                while count > 0:
                    count -= 1
                    if self.client.client.is_socket_open():
                        self.is_connected = True
                        logger.info(f"工作站通信连接成功: {self.config.host}:{self.config.port}")
                        return True
                    time.sleep(0.1)
                
                if not self.client.client.is_socket_open():
                    raise ConnectionError(f"无法连接到工作站: {self.config.host}:{self.config.port}")
                    
            else:
                raise NotImplementedError(f"协议 {self.config.protocol} 暂未实现")
                
        except Exception as e:
            logger.error(f"工作站通信连接失败: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """断开连接"""
        try:
            if self.client and hasattr(self.client, 'client'):
                self.client.client.close()
            self.is_connected = False
            logger.info("工作站通信连接已断开")
        except Exception as e:
            logger.error(f"断开连接时出错: {e}")

    # ============ 标准工作流接口 ============
    
    def start_workflow(self, workflow_type: str, parameters: Dict[str, Any] = None) -> bool:
        """启动工作流"""
        try:
            if not self.is_connected:
                logger.error("通信未连接，无法启动工作流")
                return False
            
            logger.info(f"启动工作流: {workflow_type}, 参数: {parameters}")
            return self._execute_start_workflow(workflow_type, parameters or {})
            
        except Exception as e:
            logger.error(f"启动工作流失败: {e}")
            return False

    def stop_workflow(self, emergency: bool = False) -> bool:
        """停止工作流"""
        try:
            if not self.is_connected:
                logger.error("通信未连接，无法停止工作流")
                return False
            
            logger.info(f"停止工作流 (紧急: {emergency})")
            return self._execute_stop_workflow(emergency)
            
        except Exception as e:
            logger.error(f"停止工作流失败: {e}")
            return False

    def get_workflow_status(self) -> Dict[str, Any]:
        """获取工作流状态"""
        try:
            if not self.is_connected:
                return {"error": "通信未连接"}
            
            return self._query_workflow_status()
            
        except Exception as e:
            logger.error(f"查询工作流状态失败: {e}")
            return {"error": str(e)}

    # ============ 设备状态查询接口 ============
    
    def get_device_status(self, force_refresh: bool = False) -> Dict[str, Any]:
        """获取设备状态（带缓存）"""
        current_time = time.time()
        
        if not force_refresh and (current_time - self._last_update_time) < self._cache_timeout:
            return self._status_cache
        
        try:
            if not self.is_connected:
                return {"error": "通信未连接"}
            
            status = self._query_device_status()
            self._status_cache = status
            self._last_update_time = current_time
            return status
            
        except Exception as e:
            logger.error(f"查询设备状态失败: {e}")
            return {"error": str(e)}

    def get_production_data(self) -> Dict[str, Any]:
        """获取生产数据"""
        try:
            if not self.is_connected:
                return {"error": "通信未连接"}
            
            return self._query_production_data()
            
        except Exception as e:
            logger.error(f"查询生产数据失败: {e}")
            return {"error": str(e)}

    # ============ 单点控制接口（调试用）============
    
    def write_register(self, register_name: str, value: Any, data_type: DataType = None, word_order: WorderOrder = None) -> bool:
        """写寄存器（单点控制）"""
        try:
            if not self.is_connected:
                logger.error("通信未连接，无法写寄存器")
                return False
            
            return self._write_single_register(register_name, value, data_type, word_order)
            
        except Exception as e:
            logger.error(f"写寄存器失败: {e}")
            return False

    def read_register(self, register_name: str, count: int = 1, data_type: DataType = None, word_order: WorderOrder = None) -> tuple:
        """读寄存器（单点控制）"""
        try:
            if not self.is_connected:
                logger.error("通信未连接，无法读寄存器")
                return None, True
            
            return self._read_single_register(register_name, count, data_type, word_order)
            
        except Exception as e:
            logger.error(f"读寄存器失败: {e}")
            return None, True

    # ============ 数据导出功能 ============
    
    def start_data_export(self, file_path: str, export_interval: float = 1.0) -> bool:
        """开始数据导出"""
        try:
            if self.data_export_running:
                logger.warning("数据导出已在运行")
                return False
            
            self.data_export_file = file_path
            self.data_export_interval = export_interval
            self.data_export_running = True
            
            # 创建CSV文件并写入表头
            self._initialize_export_file(file_path)
            
            # 启动数据收集线程
            self.data_export_thread = threading.Thread(target=self._data_export_worker)
            self.data_export_thread.daemon = True
            self.data_export_thread.start()
            
            logger.info(f"数据导出已启动: {file_path}")
            return True
            
        except Exception as e:
            logger.error(f"启动数据导出失败: {e}")
            return False

    def stop_data_export(self) -> bool:
        """停止数据导出"""
        try:
            if not self.data_export_running:
                logger.warning("数据导出未运行")
                return False
            
            self.data_export_running = False
            
            if self.data_export_thread and self.data_export_thread.is_alive():
                self.data_export_thread.join(timeout=5.0)
            
            logger.info("数据导出已停止")
            return True
            
        except Exception as e:
            logger.error(f"停止数据导出失败: {e}")
            return False

    def _data_export_worker(self):
        """数据导出工作线程"""
        while self.data_export_running:
            try:
                data = self.get_production_data()
                self._append_to_export_file(data)
                time.sleep(self.data_export_interval)
            except Exception as e:
                logger.error(f"数据导出工作线程错误: {e}")

    # ============ 抽象方法 - 子类必须实现 ============
    
    @abstractmethod
    def _execute_start_workflow(self, workflow_type: str, parameters: Dict[str, Any]) -> bool:
        """执行启动工作流命令"""
        pass

    @abstractmethod
    def _execute_stop_workflow(self, emergency: bool) -> bool:
        """执行停止工作流命令"""
        pass

    @abstractmethod
    def _query_workflow_status(self) -> Dict[str, Any]:
        """查询工作流状态"""
        pass

    @abstractmethod
    def _query_device_status(self) -> Dict[str, Any]:
        """查询设备状态"""
        pass

    @abstractmethod
    def _query_production_data(self) -> Dict[str, Any]:
        """查询生产数据"""
        pass

    @abstractmethod
    def _write_single_register(self, register_name: str, value: Any, data_type: DataType, word_order: WorderOrder) -> bool:
        """写单个寄存器"""
        pass

    @abstractmethod
    def _read_single_register(self, register_name: str, count: int, data_type: DataType, word_order: WorderOrder) -> tuple:
        """读单个寄存器"""
        pass

    @abstractmethod
    def _initialize_export_file(self, file_path: str):
        """初始化导出文件"""
        pass

    @abstractmethod
    def _append_to_export_file(self, data: Dict[str, Any]):
        """追加数据到导出文件"""
        pass


class CoinCellCommunication(WorkstationCommunicationBase):
    """纽扣电池组装系统通信类
    
    从 coin_cell_assembly_system 抽取的通信功能
    """

    def __init__(self, communication_config: CommunicationConfig, csv_path: str = "./coin_cell_assembly.csv"):
        self.csv_path = csv_path
        super().__init__(communication_config)

    def _initialize_communication(self):
        """初始化通信连接"""
        # 加载节点映射
        try:
            nodes = self.client.load_csv(self.csv_path) if self.client else []
            if self.client:
                self.client.register_node_list(nodes)
        except Exception as e:
            logger.error(f"加载节点映射失败: {e}")

    def _load_address_mapping(self) -> Dict[str, Any]:
        """加载地址映射表"""
        # 从CSV文件加载地址映射
        return {}

    def _execute_start_workflow(self, workflow_type: str, parameters: Dict[str, Any]) -> bool:
        """执行启动工作流命令"""
        if workflow_type == "battery_manufacturing":
            # 发送电池制造启动命令
            return self._start_battery_manufacturing(parameters)
        else:
            logger.error(f"不支持的工作流类型: {workflow_type}")
            return False

    def _start_battery_manufacturing(self, parameters: Dict[str, Any]) -> bool:
        """启动电池制造工作流"""
        try:
            # 1. 设置参数
            if "electrolyte_num" in parameters:
                self.client.use_node('REG_MSG_ELECTROLYTE_NUM').write(parameters["electrolyte_num"])
            
            if "electrolyte_volume" in parameters:
                self.client.use_node('REG_MSG_ELECTROLYTE_VOLUME').write(
                    parameters["electrolyte_volume"], 
                    data_type=DataType.FLOAT32, 
                    word_order=WorderOrder.LITTLE
                )
            
            if "assembly_pressure" in parameters:
                self.client.use_node('REG_MSG_ASSEMBLY_PRESSURE').write(
                    parameters["assembly_pressure"], 
                    data_type=DataType.FLOAT32, 
                    word_order=WorderOrder.LITTLE
                )
            
            # 2. 发送启动命令
            self.client.use_node('COIL_SYS_START_CMD').write(True)
            
            # 3. 确认启动成功
            time.sleep(0.5)
            status, read_err = self.client.use_node('COIL_SYS_START_STATUS').read(1)
            return not read_err and status[0]
            
        except Exception as e:
            logger.error(f"启动电池制造工作流失败: {e}")
            return False

    def _execute_stop_workflow(self, emergency: bool) -> bool:
        """执行停止工作流命令"""
        try:
            if emergency:
                # 紧急停止
                self.client.use_node('COIL_SYS_RESET_CMD').write(True)
            else:
                # 正常停止
                self.client.use_node('COIL_SYS_STOP_CMD').write(True)
            
            time.sleep(0.5)
            status, read_err = self.client.use_node('COIL_SYS_STOP_STATUS').read(1)
            return not read_err and status[0]
            
        except Exception as e:
            logger.error(f"停止工作流失败: {e}")
            return False

    def _query_workflow_status(self) -> Dict[str, Any]:
        """查询工作流状态"""
        try:
            status = {}
            
            # 读取系统状态
            start_status, _ = self.client.use_node('COIL_SYS_START_STATUS').read(1)
            stop_status, _ = self.client.use_node('COIL_SYS_STOP_STATUS').read(1)
            auto_status, _ = self.client.use_node('COIL_SYS_AUTO_STATUS').read(1)
            init_status, _ = self.client.use_node('COIL_SYS_INIT_STATUS').read(1)
            
            status.update({
                "is_running": start_status[0] if start_status else False,
                "is_stopped": stop_status[0] if stop_status else False,
                "is_auto_mode": auto_status[0] if auto_status else False,
                "is_initialized": init_status[0] if init_status else False,
            })
            
            return status
            
        except Exception as e:
            logger.error(f"查询工作流状态失败: {e}")
            return {"error": str(e)}

    def _query_device_status(self) -> Dict[str, Any]:
        """查询设备状态"""
        try:
            status = {}
            
            # 读取位置信息
            x_pos, _ = self.client.use_node('REG_DATA_AXIS_X_POS').read(2, word_order=WorderOrder.LITTLE)
            y_pos, _ = self.client.use_node('REG_DATA_AXIS_Y_POS').read(2, word_order=WorderOrder.LITTLE)
            z_pos, _ = self.client.use_node('REG_DATA_AXIS_Z_POS').read(2, word_order=WorderOrder.LITTLE)
            
            # 读取环境数据
            pressure, _ = self.client.use_node('REG_DATA_GLOVE_BOX_PRESSURE').read(2, word_order=WorderOrder.LITTLE)
            o2_content, _ = self.client.use_node('REG_DATA_GLOVE_BOX_O2_CONTENT').read(2, word_order=WorderOrder.LITTLE)
            water_content, _ = self.client.use_node('REG_DATA_GLOVE_BOX_WATER_CONTENT').read(2, word_order=WorderOrder.LITTLE)
            
            status.update({
                "axis_position": {
                    "x": x_pos[0] if x_pos else 0.0,
                    "y": y_pos[0] if y_pos else 0.0,
                    "z": z_pos[0] if z_pos else 0.0,
                },
                "environment": {
                    "glove_box_pressure": pressure[0] if pressure else 0.0,
                    "o2_content": o2_content[0] if o2_content else 0.0,
                    "water_content": water_content[0] if water_content else 0.0,
                }
            })
            
            return status
            
        except Exception as e:
            logger.error(f"查询设备状态失败: {e}")
            return {"error": str(e)}

    def _query_production_data(self) -> Dict[str, Any]:
        """查询生产数据"""
        try:
            data = {}
            
            # 读取生产统计
            coin_cell_num, _ = self.client.use_node('REG_DATA_ASSEMBLY_COIN_CELL_NUM').read(1)
            assembly_time, _ = self.client.use_node('REG_DATA_ASSEMBLY_TIME').read(2, word_order=WorderOrder.LITTLE)
            voltage, _ = self.client.use_node('REG_DATA_OPEN_CIRCUIT_VOLTAGE').read(2, word_order=WorderOrder.LITTLE)
            
            # 读取当前产品信息
            coin_cell_code, _ = self.client.use_node('REG_DATA_COIN_CELL_CODE').read(20)  # 假设是字符串
            electrolyte_code, _ = self.client.use_node('REG_DATA_ELECTROLYTE_CODE').read(20)
            
            data.update({
                "production_count": coin_cell_num[0] if coin_cell_num else 0,
                "assembly_time": assembly_time[0] if assembly_time else 0.0,
                "open_circuit_voltage": voltage[0] if voltage else 0.0,
                "current_battery_code": self._decode_string(coin_cell_code) if coin_cell_code else "",
                "current_electrolyte_code": self._decode_string(electrolyte_code) if electrolyte_code else "",
                "timestamp": time.time(),
            })
            
            return data
            
        except Exception as e:
            logger.error(f"查询生产数据失败: {e}")
            return {"error": str(e)}

    def _write_single_register(self, register_name: str, value: Any, data_type: DataType = None, word_order: WorderOrder = None) -> bool:
        """写单个寄存器"""
        try:
            kwargs = {"value": value}
            if data_type:
                kwargs["data_type"] = data_type
            if word_order:
                kwargs["word_order"] = word_order
            
            result = self.client.use_node(register_name).write(**kwargs)
            return result
            
        except Exception as e:
            logger.error(f"写寄存器 {register_name} 失败: {e}")
            return False

    def _read_single_register(self, register_name: str, count: int = 1, data_type: DataType = None, word_order: WorderOrder = None) -> tuple:
        """读单个寄存器"""
        try:
            kwargs = {"count": count}
            if data_type:
                kwargs["data_type"] = data_type
            if word_order:
                kwargs["word_order"] = word_order
            
            value, error = self.client.use_node(register_name).read(**kwargs)
            return value, error
            
        except Exception as e:
            logger.error(f"读寄存器 {register_name} 失败: {e}")
            return None, True

    def _initialize_export_file(self, file_path: str):
        """初始化导出文件"""
        import csv
        try:
            with open(file_path, 'w', newline='', encoding='utf-8') as csvfile:
                fieldnames = [
                    'timestamp', 'production_count', 'assembly_time', 
                    'open_circuit_voltage', 'battery_code', 'electrolyte_code',
                    'axis_x', 'axis_y', 'axis_z', 'glove_box_pressure', 
                    'o2_content', 'water_content'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
        except Exception as e:
            logger.error(f"初始化导出文件失败: {e}")

    def _append_to_export_file(self, data: Dict[str, Any]):
        """追加数据到导出文件"""
        import csv
        try:
            with open(self.data_export_file, 'a', newline='', encoding='utf-8') as csvfile:
                fieldnames = [
                    'timestamp', 'production_count', 'assembly_time', 
                    'open_circuit_voltage', 'battery_code', 'electrolyte_code',
                    'axis_x', 'axis_y', 'axis_z', 'glove_box_pressure', 
                    'o2_content', 'water_content'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                row = {
                    'timestamp': data.get('timestamp', time.time()),
                    'production_count': data.get('production_count', 0),
                    'assembly_time': data.get('assembly_time', 0.0),
                    'open_circuit_voltage': data.get('open_circuit_voltage', 0.0),
                    'battery_code': data.get('current_battery_code', ''),
                    'electrolyte_code': data.get('current_electrolyte_code', ''),
                }
                
                # 添加位置数据
                axis_pos = data.get('axis_position', {})
                row.update({
                    'axis_x': axis_pos.get('x', 0.0),
                    'axis_y': axis_pos.get('y', 0.0),
                    'axis_z': axis_pos.get('z', 0.0),
                })
                
                # 添加环境数据
                env = data.get('environment', {})
                row.update({
                    'glove_box_pressure': env.get('glove_box_pressure', 0.0),
                    'o2_content': env.get('o2_content', 0.0),
                    'water_content': env.get('water_content', 0.0),
                })
                
                writer.writerow(row)
                
        except Exception as e:
            logger.error(f"追加数据到导出文件失败: {e}")

    def _decode_string(self, data_list: List[int]) -> str:
        """将寄存器数据解码为字符串"""
        try:
            # 假设每个寄存器包含2个字符（16位）
            chars = []
            for value in data_list:
                if value == 0:
                    break
                chars.append(chr(value & 0xFF))
                if (value >> 8) & 0xFF != 0:
                    chars.append(chr((value >> 8) & 0xFF))
            return ''.join(chars).rstrip('\x00')
        except:
            return ""
