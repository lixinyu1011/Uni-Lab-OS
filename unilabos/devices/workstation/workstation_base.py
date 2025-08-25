"""
工作站基类
Workstation Base Class - 单接口模式

基于单一硬件接口的简化工作站架构
支持直接模式和代理模式的自动工作流执行器选择
"""
import time
import traceback
from typing import Dict, Any, List, Optional, Union, TYPE_CHECKING
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

if TYPE_CHECKING:
    from unilabos.ros.nodes.presets.protocol_node import ROS2WorkstationNode

from unilabos.devices.work_station.workstation_material_management import MaterialManagementBase
from unilabos.devices.work_station.workstation_http_service import (
    WorkstationHTTPService, WorkstationReportRequest, MaterialUsage
)
from unilabos.utils.log import logger


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


class WorkstationBase(ABC):
    """工作站基类 - 单接口模式
    
    核心设计原则：
    1. 每个工作站只有一个 hardware_interface
    2. 根据接口类型自动选择工作流执行器
    3. 支持直接模式和代理模式
    4. 统一的设备操作接口
    """

    def __init__(
        self,
        device_id: str,
        deck_config: Optional[Dict[str, Any]] = None,
        http_service_config: Optional[Dict[str, Any]] = None,
        *args,
        **kwargs,
    ):
        # 基本配置
        self.device_id = device_id
        self.deck_config = deck_config or {"size_x": 1000.0, "size_y": 1000.0, "size_z": 500.0}
        
        # HTTP服务配置
        self.http_service_config = http_service_config or {
            "enabled": True,
            "host": "127.0.0.1", 
            "port": 8081
        }
        
        # 单一硬件接口 - 可以是具体客户端对象或代理字符串
        self.hardware_interface: Union[Any, str] = None
        
        # 协议节点引用（用于代理模式）
        self._protocol_node: Optional['ROS2WorkstationNode'] = None
        
        # 工作流执行器（基于通信接口类型自动选择）
        self.workflow_executor: Optional['WorkflowExecutor'] = None
        
        # 工作流状态
        self.current_workflow_status = WorkflowStatus.IDLE
        self.current_workflow_info = None
        self.workflow_start_time = None
        self.workflow_parameters = {}
        
        # 错误处理
        self.error_history = []
        self.action_results = {}
        
        # 支持的工作流（静态预定义）
        self.supported_workflows: Dict[str, WorkflowInfo] = {}
        
        # 初始化工作站模块
        self.material_management: MaterialManagementBase = self._create_material_management_module()
        
        # 注册支持的工作流
        self._register_supported_workflows()
        
        # 启动HTTP报送接收服务
        self.http_service = None
        self._start_http_service()
        
        logger.info(f"工作站 {device_id} 初始化完成（单接口模式）")

    def set_hardware_interface(self, hardware_interface: Union[Any, str]):
        """设置硬件接口"""
        self.hardware_interface = hardware_interface
        
        # 根据接口类型自动创建工作流执行器
        self._setup_workflow_executor()
        
        logger.info(f"工作站 {self.device_id} 硬件接口设置: {type(hardware_interface).__name__}")

    def set_protocol_node(self, protocol_node: 'ROS2WorkstationNode'):
        """设置协议节点引用（用于代理模式）"""
        self._protocol_node = protocol_node
        logger.info(f"工作站 {self.device_id} 关联协议节点")

    def _setup_workflow_executor(self):
        """根据硬件接口类型自动设置工作流执行器"""
        if self.hardware_interface is None:
            return
        
        # 动态导入工作流执行器类
        try:
            from unilabos.devices.work_station.workflow_executors import (
                ProxyWorkflowExecutor, ModbusWorkflowExecutor, 
                HttpWorkflowExecutor, PyLabRobotWorkflowExecutor
            )
        except ImportError:
            logger.warning("工作流执行器模块未找到，将使用基础执行器")
            self.workflow_executor = None
            return
        
        # 检查是否为代理字符串
        if isinstance(self.hardware_interface, str) and self.hardware_interface.startswith("proxy:"):
            self.workflow_executor = ProxyWorkflowExecutor(self)
            logger.info(f"工作站 {self.device_id} 使用代理工作流执行器")
            
        # 检查是否为Modbus客户端
        elif hasattr(self.hardware_interface, 'write_register') and hasattr(self.hardware_interface, 'read_register'):
            self.workflow_executor = ModbusWorkflowExecutor(self)
            logger.info(f"工作站 {self.device_id} 使用Modbus工作流执行器")
            
        # 检查是否为HTTP客户端
        elif hasattr(self.hardware_interface, 'post') or hasattr(self.hardware_interface, 'get'):
            self.workflow_executor = HttpWorkflowExecutor(self)
            logger.info(f"工作站 {self.device_id} 使用HTTP工作流执行器")
            
        # 检查是否为PyLabRobot设备
        elif hasattr(self.hardware_interface, 'transfer_liquid') or hasattr(self.hardware_interface, 'pickup_tips'):
            self.workflow_executor = PyLabRobotWorkflowExecutor(self)
            logger.info(f"工作站 {self.device_id} 使用PyLabRobot工作流执行器")
            
        else:
            logger.warning(f"工作站 {self.device_id} 无法识别硬件接口类型: {type(self.hardware_interface)}")
            self.workflow_executor = None

    # ============ 统一的设备操作接口 ============
    
    def call_device_method(self, method: str, *args, **kwargs) -> Any:
        """调用设备方法的统一接口"""
        # 1. 代理模式：通过协议节点转发
        if isinstance(self.hardware_interface, str) and self.hardware_interface.startswith("proxy:"):
            if not self._protocol_node:
                raise RuntimeError("代理模式需要设置protocol_node")
            
            device_id = self.hardware_interface[6:]  # 移除 "proxy:" 前缀
            return self._protocol_node.call_device_method(device_id, method, *args, **kwargs)
        
        # 2. 直接模式：直接调用硬件接口方法
        elif self.hardware_interface and hasattr(self.hardware_interface, method):
            return getattr(self.hardware_interface, method)(*args, **kwargs)
        
        else:
            raise AttributeError(f"硬件接口不支持方法: {method}")

    def get_device_status(self) -> Dict[str, Any]:
        """获取设备状态"""
        try:
            return self.call_device_method('get_status')
        except AttributeError:
            # 如果设备不支持get_status方法，返回基础状态
            return {
                "status": "unknown",
                "interface_type": type(self.hardware_interface).__name__,
                "timestamp": time.time()
            }

    def is_device_available(self) -> bool:
        """检查设备是否可用"""
        try:
            self.get_device_status()
            return True
        except:
            return False

    # ============ 工作流控制接口 ============

    def execute_workflow(self, workflow_name: str, parameters: Dict[str, Any]) -> bool:
        """执行工作流 - 委托给工作流执行器"""
        if not self.workflow_executor:
            logger.error(f"工作站 {self.device_id} 工作流执行器未初始化")
            return False
        
        try:
            # 设置工作流状态
            self.current_workflow_status = WorkflowStatus.INITIALIZING
            self.workflow_parameters = parameters
            self.workflow_start_time = time.time()
            
            # 委托给工作流执行器
            success = self.workflow_executor.execute_workflow(workflow_name, parameters)
            
            if success:
                self.current_workflow_status = WorkflowStatus.RUNNING
                logger.info(f"工作站 {self.device_id} 工作流 {workflow_name} 启动成功")
            else:
                self.current_workflow_status = WorkflowStatus.ERROR
                logger.error(f"工作站 {self.device_id} 工作流 {workflow_name} 启动失败")
            
            return success
            
        except Exception as e:
            self.current_workflow_status = WorkflowStatus.ERROR
            logger.error(f"工作站 {self.device_id} 执行工作流失败: {e}")
            return False

    def start_workflow(self, workflow_type: str, parameters: Dict[str, Any] = None) -> bool:
        """启动工作流 - 兼容旧接口"""
        return self.execute_workflow(workflow_type, parameters or {})

    def stop_workflow(self, emergency: bool = False) -> bool:
        """停止工作流"""
        if not self.workflow_executor:
            logger.warning(f"工作站 {self.device_id} 工作流执行器未初始化")
            return True
        
        try:
            if self.current_workflow_status in [WorkflowStatus.IDLE, WorkflowStatus.STOPPED]:
                logger.warning(f"工作站 {self.device_id} 没有正在运行的工作流")
                return True
            
            self.current_workflow_status = WorkflowStatus.STOPPING
            
            # 委托给工作流执行器
            success = self.workflow_executor.stop_workflow(emergency)
            
            if success:
                self.current_workflow_status = WorkflowStatus.STOPPED
                logger.info(f"工作站 {self.device_id} 工作流停止成功 (紧急: {emergency})")
            else:
                self.current_workflow_status = WorkflowStatus.ERROR
                logger.error(f"工作站 {self.device_id} 工作流停止失败")
            
            return success
            
        except Exception as e:
            self.current_workflow_status = WorkflowStatus.ERROR
            logger.error(f"工作站 {self.device_id} 停止工作流失败: {e}")
            return False

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

    # ============ 抽象方法 - 子类必须实现 ============
    
    @abstractmethod
    def _create_material_management_module(self) -> MaterialManagementBase:
        """创建物料管理模块 - 子类必须实现"""
        pass

    @abstractmethod
    def _register_supported_workflows(self):
        """注册支持的工作流 - 子类必须实现"""
        pass

    # ============ HTTP服务管理 ============

    def _start_http_service(self):
        """启动HTTP报送接收服务"""
        try:
            if not self.http_service_config.get("enabled", True):
                logger.info(f"工作站 {self.device_id} HTTP报送接收服务已禁用")
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
            logger.error(f"工作站 {self.device_id} 启动HTTP报送接收服务失败: {e}")
            self.http_service = None

    def _stop_http_service(self):
        """停止HTTP报送接收服务"""
        try:
            if self.http_service:
                self.http_service.stop()
                self.http_service = None
                logger.info(f"工作站 {self.device_id} HTTP报送接收服务已停止")
        except Exception as e:
            logger.error(f"工作站 {self.device_id} 停止HTTP报送接收服务失败: {e}")

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
