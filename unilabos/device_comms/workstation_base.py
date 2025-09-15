"""
工作站基类
Workstation Base Class

集成通信、物料管理和工作流的工作站基类
融合子设备管理、动态工作流注册等高级功能
"""
import asyncio
import json
import time
import traceback
from typing import Dict, Any, List, Optional, Union, Callable
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
from unilabos_msgs.srv import SerialCommand
from unilabos_msgs.msg import Resource

from unilabos.ros.nodes.presets.protocol_node import ROS2ProtocolNode
from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.device_comms.workstation_communication import WorkstationCommunicationBase, CommunicationConfig
from unilabos.device_comms.workstation_material_management import MaterialManagementBase
from unilabos.ros.msgs.message_converter import convert_to_ros_msg, convert_from_ros_msg
from unilabos.utils.log import logger
from unilabos.utils.type_check import serialize_result_info


class DeviceType(Enum):
    """设备类型枚举"""
    LOGICAL = "logical"      # 逻辑设备
    COMMUNICATION = "communication"  # 通信设备 (modbus/opcua/serial)
    PROTOCOL = "protocol"    # 协议设备


@dataclass
class CommunicationInterface:
    """通信接口配置"""
    device_id: str          # 通信设备ID
    read_method: str        # 读取方法名
    write_method: str       # 写入方法名
    protocol_type: str      # 协议类型 (modbus/opcua/serial)
    config: Dict[str, Any]  # 协议特定配置


@dataclass
class WorkflowStep:
    """工作流步骤定义"""
    device_id: str
    action_name: str
    action_kwargs: Dict[str, Any]
    depends_on: Optional[List[str]] = None  # 依赖的步骤ID
    step_id: Optional[str] = None
    timeout: Optional[float] = None
    retry_count: int = 0


@dataclass
class WorkflowDefinition:
    """工作流定义"""
    name: str
    description: str
    steps: List[WorkflowStep]
    input_schema: Dict[str, Any]
    output_schema: Dict[str, Any]
    metadata: Dict[str, Any]


class WorkflowStatus(Enum):
    """工作流状态"""
    IDLE = "idle"
    INITIALIZING = "initializing"  
    RUNNING = "running"
    PAUSED = "paused"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"
    COMPLETED = "completed"


@dataclass
class WorkflowInfo:
    """工作流信息"""
    name: str
    description: str
    estimated_duration: float  # 预估持续时间（秒）
    required_materials: List[str]  # 所需物料类型
    output_product: str  # 输出产品类型
    parameters_schema: Dict[str, Any]  # 参数架构


class WorkstationBase(ROS2ProtocolNode, ABC):
    """工作站基类
    
    提供工作站的核心功能：
    1. 通信转发 - 与PLC/设备的通信接口
    2. 物料管理 - 基于PyLabRobot的物料系统
    3. 工作流控制 - 支持动态注册和静态预定义工作流
    4. 子设备管理 - 继承自ROS2ProtocolNode的设备管理能力
    5. 状态监控 - 设备状态和生产数据监控
    6. 调试接口 - 单点控制和紧急操作
    """

    def __init__(
        self,
        device_id: str,
        children: Dict[str, Dict[str, Any]],
        protocol_type: Union[str, List[str]],
        resource_tracker: DeviceNodeResourceTracker,
        communication_config: CommunicationConfig,
        deck_config: Optional[Dict[str, Any]] = None,
        communication_interfaces: Optional[Dict[str, CommunicationInterface]] = None,
        *args,
        **kwargs,
    ):
        # 保存工作站特定配置
        self.communication_config = communication_config
        self.deck_config = deck_config or {"size_x": 1000.0, "size_y": 1000.0, "size_z": 500.0}
        self.communication_interfaces = communication_interfaces or {}
        
        # 工作流状态 - 支持静态和动态工作流
        self.current_workflow_status = WorkflowStatus.IDLE
        self.current_workflow_info = None
        self.workflow_start_time = None
        self.workflow_parameters = {}
        
        # 支持的工作流（静态预定义）
        self.supported_workflows: Dict[str, WorkflowInfo] = {}
        
        # 动态注册的工作流
        self.registered_workflows: Dict[str, WorkflowDefinition] = {}
        self._workflow_action_servers: Dict[str, ActionServer] = {}
        
        # 初始化基类 - ROS2ProtocolNode会处理子设备初始化
        super().__init__(
            device_id=device_id,
            children=children,
            protocol_type=protocol_type,
            resource_tracker=resource_tracker,
            *args,
            **kwargs
        )
        
        # 工作站特有的设备分类 (基于已初始化的sub_devices)
        self.communication_devices: Dict[str, Any] = {}
        self.logical_devices: Dict[str, Any] = {}
        self._classify_devices()
        
        # 初始化工作站模块
        self.communication: WorkstationCommunicationBase = self._create_communication_module()
        self.material_management: MaterialManagementBase = self._create_material_management_module()
        
        # 设置工作站特定的通信接口
        self._setup_workstation_communication_interfaces()
        
        # 注册支持的工作流
        self._register_supported_workflows()
        
        # 创建工作站ROS服务
        self._create_workstation_services()
        
        # 启动状态监控
        self._start_status_monitoring()
        
        logger.info(f"增强工作站基类 {device_id} 初始化完成")

    def _classify_devices(self):
        """基于已初始化的设备进行分类"""
        for device_id, device in self.sub_devices.items():
            device_config = self.children.get(device_id, {})
            device_type = DeviceType(device_config.get("device_type", "logical"))
            
            if device_type == DeviceType.COMMUNICATION:
                self.communication_devices[device_id] = device
                logger.info(f"通信设备 {device_id} 已分类")
            elif device_type == DeviceType.LOGICAL:
                self.logical_devices[device_id] = device
                logger.info(f"逻辑设备 {device_id} 已分类")

    def _setup_workstation_communication_interfaces(self):
        """设置工作站特定的通信接口代理"""
        for logical_device_id, logical_device in self.logical_devices.items():
            # 检查是否有配置的通信接口
            interface_config = self.communication_interfaces.get(logical_device_id)
            if not interface_config:
                continue
            
            comm_device = self.communication_devices.get(interface_config.device_id)
            if not comm_device:
                logger.error(f"通信设备 {interface_config.device_id} 不存在")
                continue
            
            # 设置工作站级别的通信代理
            self._setup_workstation_hardware_proxy(
                logical_device, 
                comm_device, 
                interface_config
            )

    def _setup_workstation_hardware_proxy(self, logical_device, comm_device, interface: CommunicationInterface):
        """为逻辑设备设置工作站级通信代理"""
        try:
            # 获取通信设备的读写方法
            read_func = getattr(comm_device.driver_instance, interface.read_method, None)
            write_func = getattr(comm_device.driver_instance, interface.write_method, None)
            
            if read_func:
                setattr(logical_device.driver_instance, 'comm_read', read_func)
            if write_func:
                setattr(logical_device.driver_instance, 'comm_write', write_func)
            
            # 设置通信配置
            setattr(logical_device.driver_instance, 'comm_config', interface.config)
            setattr(logical_device.driver_instance, 'comm_protocol', interface.protocol_type)
            
            logger.info(f"为逻辑设备 {logical_device.device_id} 设置工作站通信代理 -> {comm_device.device_id}")
            
        except Exception as e:
            logger.error(f"设置工作站通信代理失败: {e}")

    @abstractmethod
    def _create_communication_module(self) -> WorkstationCommunicationBase:
        """创建通信模块 - 子类必须实现"""
        pass

    @abstractmethod
    def _create_material_management_module(self) -> MaterialManagementBase:
        """创建物料管理模块 - 子类必须实现"""
        pass

    @abstractmethod
    def _register_supported_workflows(self):
        """注册支持的工作流 - 子类必须实现"""
        pass

    def _create_workstation_services(self):
        """创建工作站ROS服务"""
        self._workstation_services = {
            # 动态工作流管理服务
            "register_workflow": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/register_workflow",
                self._handle_register_workflow,
                callback_group=self.callback_group,
            ),
            "unregister_workflow": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/unregister_workflow",
                self._handle_unregister_workflow,
                callback_group=self.callback_group,
            ),
            "list_workflows": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/list_workflows",
                self._handle_list_workflows,
                callback_group=self.callback_group,
            ),
            
            # 增强物料管理服务
            "create_resource": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/create_resource",
                self._handle_create_resource,
                callback_group=self.callback_group,
            ),
            "delete_resource": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/delete_resource",
                self._handle_delete_resource,
                callback_group=self.callback_group,
            ),
            "update_resource": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/update_resource",
                self._handle_update_resource,
                callback_group=self.callback_group,
            ),
            "get_resource": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/get_resource",
                self._handle_get_resource,
                callback_group=self.callback_group,
            ),
            
            # 工作站特有服务
            "start_workflow": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/start_workflow",
                self._handle_start_workflow,
                callback_group=self.callback_group,
            ),
            "stop_workflow": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/stop_workflow",
                self._handle_stop_workflow,
                callback_group=self.callback_group,
            ),
            "get_workflow_status": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/get_workflow_status",
                self._handle_get_workflow_status,
                callback_group=self.callback_group,
            ),
            "get_device_status": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/get_device_status",
                self._handle_get_device_status,
                callback_group=self.callback_group,
            ),
            "get_production_data": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/get_production_data",
                self._handle_get_production_data,
                callback_group=self.callback_group,
            ),
            "get_material_inventory": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/get_material_inventory",
                self._handle_get_material_inventory,
                callback_group=self.callback_group,
            ),
            "find_materials": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/find_materials",
                self._handle_find_materials,
                callback_group=self.callback_group,
            ),
            "write_register": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/write_register",
                self._handle_write_register,
                callback_group=self.callback_group,
            ),
            "read_register": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/read_register",
                self._handle_read_register,
                callback_group=self.callback_group,
            ),
            "emergency_stop": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/emergency_stop",
                self._handle_emergency_stop,
                callback_group=self.callback_group,
            ),
        }

    def _start_status_monitoring(self):
        """启动状态监控"""
        # 这里可以启动定期状态查询线程
        # 目前简化为按需查询
        pass

    # ============ 工作流控制接口 ============
    
    def _handle_start_workflow(self, request, response):
        """处理启动工作流请求"""
        try:
            import json
            
            # 解析请求参数
            params = json.loads(request.data) if request.data else {}
            workflow_type = params.get("workflow_type", "")
            workflow_parameters = params.get("parameters", {})
            
            if not workflow_type:
                response.success = False
                response.message = "缺少工作流类型参数"
                return response
            
            if workflow_type not in self.supported_workflows:
                response.success = False
                response.message = f"不支持的工作流类型: {workflow_type}"
                return response
            
            if self.current_workflow_status != WorkflowStatus.IDLE:
                response.success = False
                response.message = f"当前状态不允许启动工作流: {self.current_workflow_status.value}"
                return response
            
            # 启动工作流
            success = self.start_workflow(workflow_type, workflow_parameters)
            
            response.success = success
            response.message = "工作流启动成功" if success else "工作流启动失败"
            response.data = json.dumps({
                "workflow_type": workflow_type,
                "status": self.current_workflow_status.value,
                "estimated_duration": self.supported_workflows[workflow_type].estimated_duration
            })
            
        except Exception as e:
            logger.error(f"处理启动工作流请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    def _handle_stop_workflow(self, request, response):
        """处理停止工作流请求"""
        try:
            import json
            
            params = json.loads(request.data) if request.data else {}
            emergency = params.get("emergency", False)
            
            success = self.stop_workflow(emergency)
            
            response.success = success
            response.message = "工作流停止成功" if success else "工作流停止失败"
            response.data = json.dumps({
                "status": self.current_workflow_status.value,
                "emergency": emergency
            })
            
        except Exception as e:
            logger.error(f"处理停止工作流请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    def _handle_get_workflow_status(self, request, response):
        """处理获取工作流状态请求"""
        try:
            import json
            import time
            
            status_info = {
                "status": self.current_workflow_status.value,
                "workflow_info": self.current_workflow_info.name if self.current_workflow_info else None,
                "start_time": self.workflow_start_time,
                "parameters": self.workflow_parameters,
                "supported_workflows": {
                    name: {
                        "description": info.description,
                        "estimated_duration": info.estimated_duration,
                        "required_materials": info.required_materials,
                        "output_product": info.output_product
                    }
                    for name, info in self.supported_workflows.items()
                }
            }
            
            # 如果工作流正在运行，添加进度信息
            if self.current_workflow_status == WorkflowStatus.RUNNING and self.workflow_start_time:
                elapsed_time = time.time() - self.workflow_start_time
                estimated_duration = self.current_workflow_info.estimated_duration if self.current_workflow_info else 0
                progress = min(elapsed_time / estimated_duration * 100, 99) if estimated_duration > 0 else 0
                
                status_info.update({
                    "elapsed_time": elapsed_time,
                    "estimated_remaining": max(estimated_duration - elapsed_time, 0),
                    "progress_percent": progress
                })
            
            # 查询PLC状态
            plc_status = self.communication.get_workflow_status()
            status_info["plc_status"] = plc_status
            
            response.success = True
            response.message = "获取状态成功"
            response.data = json.dumps(status_info)
            
        except Exception as e:
            logger.error(f"处理获取工作流状态请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    # ============ 设备状态接口 ============
    
    def _handle_get_device_status(self, request, response):
        """处理获取设备状态请求"""
        try:
            import json
            
            # 从通信模块获取设备状态
            device_status = self.communication.get_device_status()
            
            response.success = True
            response.message = "获取设备状态成功"
            response.data = json.dumps(device_status)
            
        except Exception as e:
            logger.error(f"处理获取设备状态请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    def _handle_get_production_data(self, request, response):
        """处理获取生产数据请求"""
        try:
            import json
            
            # 从通信模块获取生产数据
            production_data = self.communication.get_production_data()
            
            response.success = True
            response.message = "获取生产数据成功"
            response.data = json.dumps(production_data)
            
        except Exception as e:
            logger.error(f"处理获取生产数据请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    # ============ 物料管理接口 ============
    
    def _handle_get_material_inventory(self, request, response):
        """处理获取物料库存请求"""
        try:
            import json
            
            # 从物料管理模块获取库存
            inventory = self.material_management.get_material_inventory()
            deck_state = self.material_management.get_deck_state()
            
            result = {
                "inventory": inventory,
                "deck_state": deck_state
            }
            
            response.success = True
            response.message = "获取物料库存成功"
            response.data = json.dumps(result)
            
        except Exception as e:
            logger.error(f"处理获取物料库存请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    def _handle_find_materials(self, request, response):
        """处理查找物料请求"""
        try:
            import json
            
            params = json.loads(request.data) if request.data else {}
            material_type = params.get("material_type", "")
            category = params.get("category", "")
            name_pattern = params.get("name_pattern", "")
            
            found_materials = []
            
            if material_type:
                materials = self.material_management.find_materials_by_type(material_type)
                found_materials.extend([self.material_management.convert_to_unilab_format(m) for m in materials])
            
            if category:
                materials = self.material_management.resource_tracker.find_by_category(category)
                found_materials.extend([self.material_management.convert_to_unilab_format(m) for m in materials])
            
            if name_pattern:
                materials = self.material_management.resource_tracker.find_by_name_pattern(name_pattern)
                found_materials.extend([self.material_management.convert_to_unilab_format(m) for m in materials])
            
            response.success = True
            response.message = f"找到 {len(found_materials)} 个物料"
            response.data = json.dumps({"materials": found_materials})
            
        except Exception as e:
            logger.error(f"处理查找物料请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    # ============ 调试控制接口 ============
    
    def _handle_write_register(self, request, response):
        """处理写寄存器请求"""
        try:
            import json
            from unilabos.device_comms.modbus_plc.node.modbus import DataType, WorderOrder
            
            params = json.loads(request.data) if request.data else {}
            register_name = params.get("register_name", "")
            value = params.get("value")
            data_type_str = params.get("data_type", "")
            word_order_str = params.get("word_order", "")
            
            if not register_name or value is None:
                response.success = False
                response.message = "缺少寄存器名称或值"
                return response
            
            # 转换数据类型和字节序
            data_type = DataType[data_type_str] if data_type_str else None
            word_order = WorderOrder[word_order_str] if word_order_str else None
            
            success = self.communication.write_register(register_name, value, data_type, word_order)
            
            response.success = success
            response.message = "写寄存器成功" if success else "写寄存器失败"
            response.data = json.dumps({
                "register_name": register_name,
                "value": value,
                "data_type": data_type_str,
                "word_order": word_order_str
            })
            
        except Exception as e:
            logger.error(f"处理写寄存器请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    def _handle_read_register(self, request, response):
        """处理读寄存器请求"""
        try:
            import json
            from unilabos.device_comms.modbus_plc.node.modbus import DataType, WorderOrder
            
            params = json.loads(request.data) if request.data else {}
            register_name = params.get("register_name", "")
            count = params.get("count", 1)
            data_type_str = params.get("data_type", "")
            word_order_str = params.get("word_order", "")
            
            if not register_name:
                response.success = False
                response.message = "缺少寄存器名称"
                return response
            
            # 转换数据类型和字节序
            data_type = DataType[data_type_str] if data_type_str else None
            word_order = WorderOrder[word_order_str] if word_order_str else None
            
            value, error = self.communication.read_register(register_name, count, data_type, word_order)
            
            response.success = not error
            response.message = "读寄存器成功" if not error else "读寄存器失败"
            response.data = json.dumps({
                "register_name": register_name,
                "value": value,
                "error": error,
                "data_type": data_type_str,
                "word_order": word_order_str
            })
            
        except Exception as e:
            logger.error(f"处理读寄存器请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    def _handle_emergency_stop(self, request, response):
        """处理紧急停止请求"""
        try:
            import json
            
            # 立即停止工作流
            success = self.stop_workflow(emergency=True)
            
            # 更新状态
            if success:
                self.current_workflow_status = WorkflowStatus.STOPPED
            
            response.success = success
            response.message = "紧急停止成功" if success else "紧急停止失败"
            response.data = json.dumps({
                "status": self.current_workflow_status.value,
                "timestamp": time.time()
            })
            
        except Exception as e:
            logger.error(f"处理紧急停止请求失败: {e}")
            response.success = False
            response.message = f"处理请求失败: {str(e)}"
        
        return response

    # ============ 工作流控制方法 ============
    
    def start_workflow(self, workflow_type: str, parameters: Dict[str, Any] = None) -> bool:
        """启动工作流"""
        try:
            if workflow_type not in self.supported_workflows:
                logger.error(f"不支持的工作流类型: {workflow_type}")
                return False
            
            if self.current_workflow_status != WorkflowStatus.IDLE:
                logger.error(f"当前状态不允许启动工作流: {self.current_workflow_status}")
                return False
            
            # 更新状态
            self.current_workflow_status = WorkflowStatus.INITIALIZING
            self.current_workflow_info = self.supported_workflows[workflow_type]
            self.workflow_parameters = parameters or {}
            
            # 通过通信模块启动工作流
            success = self.communication.start_workflow(workflow_type, self.workflow_parameters)
            
            if success:
                self.current_workflow_status = WorkflowStatus.RUNNING
                self.workflow_start_time = time.time()
                logger.info(f"工作流启动成功: {workflow_type}")
            else:
                self.current_workflow_status = WorkflowStatus.ERROR
                logger.error(f"工作流启动失败: {workflow_type}")
            
            return success
            
        except Exception as e:
            logger.error(f"启动工作流失败: {e}")
            self.current_workflow_status = WorkflowStatus.ERROR
            return False

    def stop_workflow(self, emergency: bool = False) -> bool:
        """停止工作流"""
        try:
            if self.current_workflow_status in [WorkflowStatus.IDLE, WorkflowStatus.STOPPED]:
                logger.warning("没有正在运行的工作流")
                return True
            
            # 更新状态
            self.current_workflow_status = WorkflowStatus.STOPPING
            
            # 通过通信模块停止工作流
            success = self.communication.stop_workflow(emergency)
            
            if success:
                self.current_workflow_status = WorkflowStatus.STOPPED
                logger.info(f"工作流停止成功 (紧急: {emergency})")
            else:
                self.current_workflow_status = WorkflowStatus.ERROR
                logger.error(f"工作流停止失败 (紧急: {emergency})")
            
            return success
            
        except Exception as e:
            logger.error(f"停止工作流失败: {e}")
            self.current_workflow_status = WorkflowStatus.ERROR
            return False

    # ============ 状态属性 ============
    
    @property
    def is_busy(self) -> bool:
        """是否忙碌"""
        return self.current_workflow_status in [
            WorkflowStatus.INITIALIZING,
            WorkflowStatus.RUNNING,
            WorkflowStatus.STOPPING
        ]

    @property
    def is_ready(self) -> bool:
        """是否就绪"""
        return self.current_workflow_status == WorkflowStatus.IDLE

    @property
    def has_error(self) -> bool:
        """是否有错误"""
        return self.current_workflow_status == WorkflowStatus.ERROR

    @property
    def communication_status(self) -> Dict[str, Any]:
        """通信状态"""
        return {
            "is_connected": self.communication.is_connected,
            "host": self.communication.config.host,
            "port": self.communication.config.port,
            "protocol": self.communication.config.protocol.value
        }

    @property
    def material_status(self) -> Dict[str, Any]:
        """物料状态"""
        return {
            "total_resources": len(self.material_management.plr_resources),
            "inventory": self.material_management.get_material_inventory(),
            "deck_size": {
                "x": self.material_management.plr_deck.size_x,
                "y": self.material_management.plr_deck.size_y,
                "z": self.material_management.plr_deck.size_z
            }
        }

    # ============ 增强物料管理接口 ============
    
    def _handle_create_resource(self, request, response):
        """处理创建物料请求"""
        try:
            data = json.loads(request.data) if request.data else {}
            result = self.create_resource(
                resource_data=data.get("resource_data"),
                parent_id=data.get("parent_id"),
                location=data.get("location"),
                metadata=data.get("metadata", {})
            )
            response.success = True
            response.message = "创建物料成功"
            response.data = serialize_result_info("", True, result)
        except Exception as e:
            error_msg = f"创建物料失败: {e}\n{traceback.format_exc()}"
            logger.error(error_msg)
            response.success = False
            response.message = error_msg
            response.data = serialize_result_info(error_msg, False, {})
        return response

    def create_resource(self, resource_data: Dict[str, Any], parent_id: Optional[str] = None, 
                       location: Optional[Dict[str, float]] = None, metadata: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """创建物料资源"""
        try:
            # 验证资源数据
            if not self._validate_resource_data(resource_data):
                raise ValueError("无效的资源数据")
            
            # 添加到本地资源跟踪器
            resource = convert_to_ros_msg(Resource, resource_data)
            self.resource_tracker.add_resource(resource)
            
            # 如果有父节点，建立关联
            if parent_id:
                self._link_resource_to_parent(resource_data["id"], parent_id, location)
            
            # 同步到全局资源管理器
            self._sync_resource_to_global(resource, "create")
            
            logger.info(f"物料 {resource_data['id']} 创建成功")
            return {"resource_id": resource_data["id"], "status": "created"}
            
        except Exception as e:
            logger.error(f"创建物料失败: {e}")
            raise

    def _handle_delete_resource(self, request, response):
        """处理删除物料请求"""
        try:
            data = json.loads(request.data) if request.data else {}
            result = self.delete_resource(data.get("resource_id"))
            response.success = True
            response.message = "删除物料成功"
            response.data = serialize_result_info("", True, result)
        except Exception as e:
            error_msg = f"删除物料失败: {e}\n{traceback.format_exc()}"
            logger.error(error_msg)
            response.success = False
            response.message = error_msg
            response.data = serialize_result_info(error_msg, False, {})
        return response

    def delete_resource(self, resource_id: str) -> Dict[str, Any]:
        """删除物料资源"""
        try:
            # 从本地资源跟踪器删除
            resources = self.resource_tracker.figure_resource({"id": resource_id})
            if not resources:
                raise ValueError(f"资源 {resource_id} 不存在")
            
            # 同步到全局资源管理器
            self._sync_resource_to_global(resources[0], "delete")
            
            logger.info(f"物料 {resource_id} 删除成功")
            return {"resource_id": resource_id, "status": "deleted"}
            
        except Exception as e:
            logger.error(f"删除物料失败: {e}")
            raise

    def _handle_update_resource(self, request, response):
        """处理更新物料请求"""
        try:
            data = json.loads(request.data) if request.data else {}
            result = self.update_resource(
                resource_id=data.get("resource_id"),
                updates=data.get("updates", {})
            )
            response.success = True
            response.message = "更新物料成功"
            response.data = serialize_result_info("", True, result)
        except Exception as e:
            error_msg = f"更新物料失败: {e}\n{traceback.format_exc()}"
            logger.error(error_msg)
            response.success = False
            response.message = error_msg
            response.data = serialize_result_info(error_msg, False, {})
        return response

    def update_resource(self, resource_id: str, updates: Dict[str, Any]) -> Dict[str, Any]:
        """更新物料资源"""
        try:
            # 查找资源
            resources = self.resource_tracker.figure_resource({"id": resource_id})
            if not resources:
                raise ValueError(f"资源 {resource_id} 不存在")
            
            resource = resources[0]
            
            # 更新资源数据
            if isinstance(resource, Resource):
                if "data" in updates:
                    current_data = json.loads(resource.data) if resource.data else {}
                    current_data.update(updates["data"])
                    resource.data = json.dumps(current_data)
                
                for key, value in updates.items():
                    if key != "data" and hasattr(resource, key):
                        setattr(resource, key, value)
            
            # 同步到全局资源管理器
            self._sync_resource_to_global(resource, "update")
            
            logger.info(f"物料 {resource_id} 更新成功")
            return {"resource_id": resource_id, "status": "updated"}
            
        except Exception as e:
            logger.error(f"更新物料失败: {e}")
            raise

    def _handle_get_resource(self, request, response):
        """处理获取物料请求"""
        try:
            data = json.loads(request.data) if request.data else {}
            result = self.get_resource(
                resource_id=data.get("resource_id"),
                with_children=data.get("with_children", False)
            )
            response.success = True
            response.message = "获取物料成功"
            response.data = serialize_result_info("", True, result)
        except Exception as e:
            error_msg = f"获取物料失败: {e}\n{traceback.format_exc()}"
            logger.error(error_msg)
            response.success = False
            response.message = error_msg
            response.data = serialize_result_info(error_msg, False, {})
        return response

    def get_resource(self, resource_id: str, with_children: bool = False) -> Dict[str, Any]:
        """获取物料资源"""
        try:
            resources = self.resource_tracker.figure_resource({"id": resource_id})
            if not resources:
                raise ValueError(f"资源 {resource_id} 不存在")
            
            resource = resources[0]
            
            # 转换为字典格式
            if isinstance(resource, Resource):
                result = convert_from_ros_msg(resource)
            else:
                result = resource
            
            # 如果需要包含子资源
            if with_children:
                children = self._get_child_resources(resource_id)
                result["children"] = children
            
            return result
            
        except Exception as e:
            logger.error(f"获取物料失败: {e}")
            raise

    # ============ 动态工作流管理接口 ============
    
    def _handle_register_workflow(self, request, response):
        """处理注册工作流请求"""
        try:
            data = json.loads(request.data) if request.data else {}
            result = self.register_workflow(
                workflow_name=data.get("workflow_name"),
                workflow_definition=data.get("workflow_definition"),
                action_type=data.get("action_type")
            )
            response.success = True
            response.message = "注册工作流成功"
            response.data = serialize_result_info("", True, result)
        except Exception as e:
            error_msg = f"注册工作流失败: {e}\n{traceback.format_exc()}"
            logger.error(error_msg)
            response.success = False
            response.message = error_msg
            response.data = serialize_result_info(error_msg, False, {})
        return response

    def register_workflow(self, workflow_name: str, workflow_definition: Dict[str, Any], 
                         action_type: Optional[str] = None) -> Dict[str, Any]:
        """注册工作流并创建对应的ROS Action"""
        try:
            # 验证工作流定义
            if not self._validate_workflow_definition(workflow_definition):
                raise ValueError("无效的工作流定义")
            
            # 创建工作流定义对象
            workflow = WorkflowDefinition(
                name=workflow_name,
                description=workflow_definition.get("description", ""),
                steps=[WorkflowStep(**step) for step in workflow_definition.get("steps", [])],
                input_schema=workflow_definition.get("input_schema", {}),
                output_schema=workflow_definition.get("output_schema", {}),
                metadata=workflow_definition.get("metadata", {})
            )
            
            # 存储工作流定义
            self.registered_workflows[workflow_name] = workflow
            
            # 创建对应的ROS Action Server
            self._create_workflow_action_server(workflow_name, workflow, action_type)
            
            logger.info(f"工作流 {workflow_name} 注册成功")
            return {"workflow_name": workflow_name, "status": "registered"}
            
        except Exception as e:
            logger.error(f"注册工作流失败: {e}")
            raise

    def _create_workflow_action_server(self, workflow_name: str, workflow: WorkflowDefinition, action_type: Optional[str]):
        """为工作流创建ROS Action Server"""
        try:
            # 如果没有指定action_type，使用默认的SendCmd
            if not action_type:
                from unilabos_msgs.action import SendCmd
                action_type_class = SendCmd
            else:
                # 动态导入指定的action类型
                action_type_class = self._import_action_type(action_type)
            
            # 创建Action Server
            self._workflow_action_servers[workflow_name] = ActionServer(
                self,
                action_type_class,
                workflow_name,
                execute_callback=self._create_workflow_execute_callback(workflow),
                callback_group=ReentrantCallbackGroup(),
            )
            
            logger.info(f"为工作流 {workflow_name} 创建Action Server")
            
        except Exception as e:
            logger.error(f"创建工作流Action Server失败: {e}")
            raise

    def _create_workflow_execute_callback(self, workflow: WorkflowDefinition):
        """创建工作流执行回调"""
        async def execute_workflow(goal_handle: ServerGoalHandle):
            execution_error = ""
            execution_success = False
            workflow_return_value = None
            
            try:
                logger.info(f"开始执行工作流: {workflow.name}")
                
                # 解析输入参数
                goal = goal_handle.request
                workflow_kwargs = self._parse_workflow_input(goal, workflow.input_schema)
                
                # 执行工作流步骤
                step_results = []
                for step in workflow.steps:
                    # 检查依赖
                    if step.depends_on:
                        self._wait_for_dependencies(step.depends_on, step_results)
                    
                    # 执行步骤
                    step_result = await self._execute_workflow_step(step, workflow_kwargs)
                    step_results.append(step_result)
                    
                    # 发布反馈
                    feedback = self._create_workflow_feedback(workflow, step_results)
                    goal_handle.publish_feedback(feedback)
                
                execution_success = True
                workflow_return_value = {
                    "workflow_name": workflow.name,
                    "steps_completed": len(step_results),
                    "results": step_results
                }
                
                goal_handle.succeed()
                
            except Exception as e:
                execution_error = traceback.format_exc()
                execution_success = False
                logger.error(f"工作流执行失败: {e}")
                goal_handle.abort()
            
            # 创建结果
            result = goal_handle._action_type.Result()
            result.success = execution_success
            
            # 如果有return_info字段，设置详细信息
            if hasattr(result, 'return_info'):
                result.return_info = serialize_result_info(execution_error, execution_success, workflow_return_value)
            
            return result
        
        return execute_workflow

    async def _execute_workflow_step(self, step: WorkflowStep, workflow_kwargs: Dict[str, Any]) -> Dict[str, Any]:
        """执行单个工作流步骤 - 使用父类的execute_single_action方法"""
        try:
            # 替换参数中的变量
            resolved_kwargs = self._resolve_step_kwargs(step.action_kwargs, workflow_kwargs)
            
            # 使用父类的execute_single_action方法执行动作
            result = await self.execute_single_action(
                device_id=step.device_id,
                action_name=step.action_name,
                action_kwargs=resolved_kwargs
            )
            
            return {
                "step_id": step.step_id or f"{step.device_id}_{step.action_name}",
                "device_id": step.device_id,
                "action_name": step.action_name,
                "status": "success",
                "result": result
            }
            
        except Exception as e:
            logger.error(f"步骤执行失败: {step.step_id}, 错误: {e}")
            return {
                "step_id": step.step_id or f"{step.device_id}_{step.action_name}",
                "device_id": step.device_id,
                "action_name": step.action_name,
                "status": "failed",
                "error": str(e)
            }

    def _handle_unregister_workflow(self, request, response):
        """处理注销工作流请求"""
        try:
            data = json.loads(request.data) if request.data else {}
            workflow_name = data.get("workflow_name")
            
            if workflow_name in self.registered_workflows:
                del self.registered_workflows[workflow_name]
                
                if workflow_name in self._workflow_action_servers:
                    # 销毁Action Server
                    del self._workflow_action_servers[workflow_name]
                
                result = {"workflow_name": workflow_name, "status": "unregistered"}
                response.success = True
                response.message = "注销工作流成功"
                response.data = serialize_result_info("", True, result)
            else:
                raise ValueError(f"工作流 {workflow_name} 不存在")
                
        except Exception as e:
            error_msg = f"注销工作流失败: {e}"
            logger.error(error_msg)
            response.success = False
            response.message = error_msg
            response.data = serialize_result_info(error_msg, False, {})
        return response

    def _handle_list_workflows(self, request, response):
        """处理列出工作流请求"""
        try:
            # 静态预定义工作流
            static_workflows = []
            for name, workflow in self.supported_workflows.items():
                static_workflows.append({
                    "name": name,
                    "type": "static",
                    "description": workflow.description,
                    "estimated_duration": workflow.estimated_duration,
                    "required_materials": workflow.required_materials,
                    "output_product": workflow.output_product
                })
            
            # 动态注册工作流
            dynamic_workflows = []
            for name, workflow in self.registered_workflows.items():
                dynamic_workflows.append({
                    "name": name,
                    "type": "dynamic",
                    "description": workflow.description,
                    "step_count": len(workflow.steps),
                    "metadata": workflow.metadata
                })
            
            result = {
                "static_workflows": static_workflows,
                "dynamic_workflows": dynamic_workflows,
                "total_count": len(static_workflows) + len(dynamic_workflows)
            }
            response.success = True
            response.message = "列出工作流成功"
            response.data = serialize_result_info("", True, result)
        except Exception as e:
            error_msg = f"列出工作流失败: {e}"
            logger.error(error_msg)
            response.success = False
            response.message = error_msg
            response.data = serialize_result_info(error_msg, False, {})
        return response

    # ============ 辅助方法 ============
    
    def _validate_resource_data(self, resource_data: Dict[str, Any]) -> bool:
        """验证资源数据"""
        required_fields = ["id", "name", "type"]
        return all(field in resource_data for field in required_fields)

    def _validate_workflow_definition(self, workflow_def: Dict[str, Any]) -> bool:
        """验证工作流定义"""
        required_fields = ["steps"]
        return all(field in workflow_def for field in required_fields)

    def _sync_resource_to_global(self, resource: Resource, operation: str):
        """同步资源到全局管理器"""
        # 实现与全局资源管理器的同步逻辑
        pass

    def _link_resource_to_parent(self, resource_id: str, parent_id: str, location: Optional[Dict[str, float]]):
        """将资源链接到父节点"""
        # 实现资源父子关系的建立逻辑
        pass

    def _get_child_resources(self, resource_id: str) -> List[Dict[str, Any]]:
        """获取子资源"""
        # 实现获取子资源的逻辑
        return []

    def _import_action_type(self, action_type: str):
        """动态导入Action类型"""
        # 实现动态导入逻辑
        from unilabos_msgs.action import SendCmd
        return SendCmd

    def _parse_workflow_input(self, goal, input_schema: Dict[str, Any]) -> Dict[str, Any]:
        """解析工作流输入"""
        # 根据input_schema解析goal中的参数
        return {}

    def _wait_for_dependencies(self, dependencies: List[str], completed_steps: List[Dict[str, Any]]):
        """等待依赖步骤完成"""
        # 实现依赖等待逻辑
        pass

    def _resolve_step_kwargs(self, action_kwargs: Dict[str, Any], workflow_kwargs: Dict[str, Any]) -> Dict[str, Any]:
        """解析步骤参数中的变量"""
        # 实现参数变量替换逻辑
        return action_kwargs

    def _create_workflow_feedback(self, workflow: WorkflowDefinition, step_results: List[Dict[str, Any]]):
        """创建工作流反馈"""
        # 创建反馈消息
        return None

    # ============ 增强状态属性 ============
    
    @property
    def communication_device_count(self) -> int:
        """通信设备数量"""
        return len(self.communication_devices)

    @property
    def logical_device_count(self) -> int:
        """逻辑设备数量"""
        return len(self.logical_devices)

    @property
    def active_dynamic_workflows(self) -> int:
        """活跃动态工作流数量"""
        return len([server for server in self._workflow_action_servers.values() if server])

    @property
    def total_workflow_count(self) -> int:
        """总工作流数量"""
        return len(self.supported_workflows) + len(self.registered_workflows)

    @property
    def workstation_resource_count(self) -> int:
        """工作站资源数量"""
        return len(self.resource_tracker.figure_resource({}))
    
    @property
    def workstation_status_summary(self) -> Dict[str, Any]:
        """工作站状态摘要"""
        return {
            "workflow_status": self.current_workflow_status.value,
            "is_busy": self.is_busy,
            "is_ready": self.is_ready,
            "has_error": self.has_error,
            "total_devices": len(self.sub_devices),
            "communication_devices": self.communication_device_count,
            "logical_devices": self.logical_device_count,
            "total_workflows": self.total_workflow_count,
            "active_workflows": self.active_dynamic_workflows,
            "total_resources": self.workstation_resource_count,
            "communication_status": self.communication_status,
            "material_status": self.material_status
        }
