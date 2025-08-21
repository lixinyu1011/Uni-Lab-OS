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

from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.device_comms.workstation_material_management import MaterialManagementBase
from unilabos.device_comms.workstation_http_service import (
    WorkstationHTTPService, WorkstationReportRequest, MaterialUsage
)
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


@dataclass
class CommunicationConfig:
    """通信配置"""
    protocol: str
    host: str
    port: int
    timeout: float = 5.0
    retry_count: int = 3
    extra_params: Dict[str, Any] = None


class WorkstationBase(ABC):
    """工作站基类
    
    提供工作站的核心功能：
    1. 物料管理 - 基于PyLabRobot的物料系统
    2. 工作流控制 - 支持动态注册和静态预定义工作流
    3. 状态监控 - 设备状态和生产数据监控
    4. HTTP服务 - 接收外部报送和状态查询
    
    注意：子设备管理和通信转发功能已移入ROS2ProtocolNode
    """

    def __init__(
        self,
        device_id: str,
        deck_config: Optional[Dict[str, Any]] = None,
        http_service_config: Optional[Dict[str, Any]] = None,  # HTTP服务配置
        *args,
        **kwargs,
    ):
        # 保存工作站基本配置
        self.device_id = device_id
        self.deck_config = deck_config or {"size_x": 1000.0, "size_y": 1000.0, "size_z": 500.0}
        
        # HTTP服务配置 - 现在专门用于报送接收
        self.http_service_config = http_service_config or {
            "enabled": True,
            "host": "127.0.0.1", 
            "port": 8081  # 默认使用8081端口作为报送接收服务
        }
        
        # 错误处理和动作追踪
        self.current_action_context = None  # 当前正在执行的动作上下文
        self.error_history = []  # 错误历史记录
        self.action_results = {}  # 动作结果缓存
        
        # 工作流状态 - 支持静态和动态工作流
        self.current_workflow_status = WorkflowStatus.IDLE
        self.current_workflow_info = None
        self.workflow_start_time = None
        self.workflow_parameters = {}
        
        # 支持的工作流（静态预定义）
        self.supported_workflows: Dict[str, WorkflowInfo] = {}
        
        # 动态注册的工作流
        self.registered_workflows: Dict[str, WorkflowDefinition] = {}
        
        # 初始化工作站模块
        self.material_management: MaterialManagementBase = self._create_material_management_module()
        
        # 注册支持的工作流
        self._register_supported_workflows()
        
        # 启动HTTP报送接收服务
        self.http_service = None
        self._start_http_service()
        
        logger.info(f"工作站基类 {device_id} 初始化完成")

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
    def _start_http_service(self):
        """启动HTTP报送接收服务"""
        if self.http_service_config.get("enabled", True):
            try:
                self.http_service = WorkstationHTTPService(
                    host=self.http_service_config.get("host", "127.0.0.1"),
                    port=self.http_service_config.get("port", 8081),
                    workstation_handler=self
                )
                logger.info(f"HTTP报送接收服务已启动: {self.http_service_config['host']}:{self.http_service_config['port']}")
            except Exception as e:
                logger.error(f"启动HTTP报送接收服务失败: {e}")
        else:
            logger.info("HTTP报送接收服务已禁用")

    def _stop_http_service(self):
        """停止HTTP报送接收服务"""
        if self.http_service:
            try:
                self.http_service.stop()
                logger.info("HTTP报送接收服务已停止")
            except Exception as e:
                logger.error(f"停止HTTP报送接收服务失败: {e}")

    # ============ 核心业务方法 ============

    def start_workflow(self, workflow_type: str, parameters: Dict[str, Any] = None) -> bool:
        """启动工作流 - 业务逻辑层"""
        try:
            if self.current_workflow_status != WorkflowStatus.IDLE:
                logger.warning(f"工作流 {workflow_type} 启动失败：当前状态为 {self.current_workflow_status}")
                return False
            
            # 设置工作流状态
            self.current_workflow_status = WorkflowStatus.INITIALIZING
            self.workflow_parameters = parameters or {}
            self.workflow_start_time = time.time()
            
            # 执行具体的工作流启动逻辑
            success = self._execute_start_workflow(workflow_type, parameters or {})
            
            if success:
                self.current_workflow_status = WorkflowStatus.RUNNING
                logger.info(f"工作流 {workflow_type} 启动成功")
            else:
                self.current_workflow_status = WorkflowStatus.ERROR
                logger.error(f"工作流 {workflow_type} 启动失败")
            
            return success
            
        except Exception as e:
            self.current_workflow_status = WorkflowStatus.ERROR
            logger.error(f"启动工作流失败: {e}")
            return False

    def stop_workflow(self, emergency: bool = False) -> bool:
        """停止工作流 - 业务逻辑层"""
        try:
            if self.current_workflow_status in [WorkflowStatus.IDLE, WorkflowStatus.STOPPED]:
                logger.warning("没有正在运行的工作流")
                return True
            
            self.current_workflow_status = WorkflowStatus.STOPPING
            
            # 执行具体的工作流停止逻辑
            success = self._execute_stop_workflow(emergency)
            
            if success:
                self.current_workflow_status = WorkflowStatus.STOPPED
                logger.info(f"工作流停止成功 (紧急: {emergency})")
            else:
                self.current_workflow_status = WorkflowStatus.ERROR
                logger.error(f"工作流停止失败")
            
            return success
            
        except Exception as e:
            self.current_workflow_status = WorkflowStatus.ERROR
            logger.error(f"停止工作流失败: {e}")
            return False

    # ============ 抽象方法 - 子类必须实现具体的工作流控制 ============
    
    @abstractmethod
    def _execute_start_workflow(self, workflow_type: str, parameters: Dict[str, Any]) -> bool:
        """执行启动工作流的具体逻辑 - 子类实现"""
        pass

    @abstractmethod  
    def _execute_stop_workflow(self, emergency: bool) -> bool:
        """执行停止工作流的具体逻辑 - 子类实现"""
        pass

    # ============ 状态属性 ============
    
    @property
    def workflow_status(self) -> WorkflowStatus:
        """获取当前工作流状态"""
        return self.current_workflow_status

    @property
    def is_busy(self) -> bool:
        """检查工作站是否忙碌"""
        return self.current_workflow_status in [
            WorkflowStatus.INITIALIZING,
            WorkflowStatus.RUNNING,
            WorkflowStatus.STOPPING
        ]

    @property
    def workflow_runtime(self) -> float:
        """获取工作流运行时间（秒）"""
        if self.workflow_start_time is None:
            return 0.0
        return time.time() - self.workflow_start_time

    @property
    def error_count(self) -> int:
        """获取错误计数"""
        return len(self.error_history)

    @property
    def last_error(self) -> Optional[Dict[str, Any]]:
        """获取最后一个错误"""
        return self.error_history[-1] if self.error_history else None

    def _start_http_service(self):
        """启动HTTP报送接收服务"""
        try:
            if not self.http_service_config.get("enabled", True):
                logger.info("HTTP报送接收服务已禁用")
                return
            
            host = self.http_service_config.get("host", "127.0.0.1")
            port = self.http_service_config.get("port", 8081)
            
            self.http_service = WorkstationHTTPService(
                workstation_handler=self,
                host=host,
                port=port
            )
            
            logger.info(f"工作站 {self.device_id} HTTP报送接收服务启动成功: {host}:{port}")
            
        except Exception as e:
            logger.error(f"启动HTTP报送接收服务失败: {e}")
            self.http_service = None

    def _stop_http_service(self):
        """停止HTTP报送接收服务"""
        try:
            if self.http_service:
                self.http_service.stop()
                self.http_service = None
                logger.info("HTTP报送接收服务已停止")
        except Exception as e:
            logger.error(f"停止HTTP报送接收服务失败: {e}")
            logger.error(f"停止HTTP报送接收服务失败: {e}")

    # ============ 报送处理方法 ============

    # ============ 报送处理方法 ============

    def process_material_change_report(self, report) -> Dict[str, Any]:
        """处理物料变更报送"""
        try:
            logger.info(f"处理物料变更报送: {report.workstation_id} -> {report.resource_id} ({report.change_type})")
            
            result = {
                'processed': True,
                'resource_id': report.resource_id,
                'change_type': report.change_type,
                'timestamp': time.time()
            }
            
            # 更新本地物料管理系统
            if hasattr(self, 'material_management'):
                try:
                    self.material_management.sync_external_material_change(report)
                except Exception as e:
                    logger.warning(f"同步物料变更到本地管理系统失败: {e}")
            
            return result
            
        except Exception as e:
            logger.error(f"处理物料变更报送失败: {e}")
            return {'processed': False, 'error': str(e)}

    def process_step_finish_report(self, request: WorkstationReportRequest) -> Dict[str, Any]:
        """处理步骤完成报送（统一LIMS协议规范）"""
        try:
            data = request.data
            logger.info(f"处理步骤完成报送: {data['orderCode']} - {data['stepName']}")
            
            result = {
                'processed': True,
                'order_code': data['orderCode'],
                'step_id': data['stepId'],
                'timestamp': time.time()
            }
            
            return result
            
        except Exception as e:
            logger.error(f"处理步骤完成报送失败: {e}")
            return {'processed': False, 'error': str(e)}

    def process_sample_finish_report(self, request: WorkstationReportRequest) -> Dict[str, Any]:
        """处理样品完成报送"""
        try:
            data = request.data
            logger.info(f"处理样品完成报送: {data['sampleId']}")
            
            result = {
                'processed': True,
                'sample_id': data['sampleId'],
                'timestamp': time.time()
            }
            
            return result
            
        except Exception as e:
            logger.error(f"处理样品完成报送失败: {e}")
            return {'processed': False, 'error': str(e)}

    def process_order_finish_report(self, request: WorkstationReportRequest, used_materials: List[MaterialUsage]) -> Dict[str, Any]:
        """处理订单完成报送"""
        try:
            data = request.data
            logger.info(f"处理订单完成报送: {data['orderCode']}")
            
            result = {
                'processed': True,
                'order_code': data['orderCode'],
                'used_materials': len(used_materials),
                'timestamp': time.time()
            }
            
            return result
            
        except Exception as e:
            logger.error(f"处理订单完成报送失败: {e}")
            return {'processed': False, 'error': str(e)}

    def handle_external_error(self, error_request):
        """处理外部错误报告"""
        try:
            logger.error(f"收到外部错误报告: {error_request}")
            
            # 记录错误
            error_record = {
                'timestamp': time.time(),
                'error_type': error_request.get('error_type', 'unknown'),
                'error_message': error_request.get('message', ''),
                'source': error_request.get('source', 'external'),
                'context': error_request.get('context', {})
            }
            
            self.error_history.append(error_record)
            
            # 处理紧急停止情况
            if error_request.get('emergency_stop', False):
                self._trigger_emergency_stop(error_record['error_message'])
            
            return {'processed': True, 'error_id': len(self.error_history)}
            
        except Exception as e:
            logger.error(f"处理外部错误失败: {e}")
            return {'processed': False, 'error': str(e)}

    def _trigger_emergency_stop(self, reason: str):
        """触发紧急停止"""
        logger.critical(f"触发紧急停止: {reason}")
        self.stop_workflow(emergency=True)

    def __del__(self):
        """清理资源"""
        try:
            self._stop_http_service()
        except:
            pass
