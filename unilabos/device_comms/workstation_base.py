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
        http_service_config: Optional[Dict[str, Any]] = None,  # 新增：HTTP服务配置
        *args,
        **kwargs,
    ):
        # 保存工作站特定配置
        self.communication_config = communication_config
        self.deck_config = deck_config or {"size_x": 1000.0, "size_y": 1000.0, "size_z": 500.0}
        self.communication_interfaces = communication_interfaces or {}
        
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
        self._workflow_action_servers: Dict[str, ActionServer] = {}
        
        # 初始化基类 - ROS2ProtocolNode会处理所有设备管理
        super().__init__(
            device_id=device_id,
            children=children,
            protocol_type=protocol_type,
            resource_tracker=resource_tracker,
            workstation_config={
                'communication_interfaces': communication_interfaces,
                'deck_config': self.deck_config
            },
            *args,
            **kwargs
        )
        
        # 使用父类的设备分类结果（不再重复分类）
        # self.communication_devices 和 self.logical_devices 由 ROS2ProtocolNode 提供
        
        # 初始化工作站模块
        self.communication: WorkstationCommunicationBase = self._create_communication_module()
        self.material_management: MaterialManagementBase = self._create_material_management_module()
        
        # 注册支持的工作流
        self._register_supported_workflows()
        
        # 创建工作站ROS服务
        self._create_workstation_services()
        
        # 启动状态监控
        self._start_status_monitoring()
        
        # 启动HTTP报送接收服务
        self.http_service = None
        self._start_http_service()
        
        logger.info(f"增强工作站基类 {device_id} 初始化完成")

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

    def _start_http_service(self):
        """启动HTTP报送接收服务"""
        try:
            if not self.http_service_config.get("enabled", True):
                logger.info("HTTP报送接收服务已禁用")
                return
            
            host = self.http_service_config.get("host", "127.0.0.1")
            port = self.http_service_config.get("port", 8081)
            
            self.http_service = WorkstationHTTPService(
                workstation_instance=self,
                host=host,
                port=port
            )
            
            self.http_service.start()
            logger.info(f"工作站 {self.device_id} HTTP报送接收服务启动成功: {self.http_service.service_url}")
            
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

    # ============ 报送处理方法 ============

    def process_material_change_report(self, report) -> Dict[str, Any]:
        """处理物料变更报送 - 同步到 ResourceTracker 并发送 ROS2 更新"""
        try:
            logger.info(f"处理物料变更报送: {report.workstation_id} -> {report.resource_id} ({report.change_type})")
            
            # 增加接收计数
            self._reports_received_count = getattr(self, '_reports_received_count', 0) + 1
            
            # 准备变更数据
            changes = {
                'workstation_id': report.workstation_id,
                'timestamp': report.timestamp,
                'change_type': report.change_type,
                'resource_id': report.resource_id
            }
            
            # 添加额外的变更信息
            if hasattr(report, 'new_location'):
                changes['location'] = {
                    'x': getattr(report.new_location, 'x', 0),
                    'y': getattr(report.new_location, 'y', 0),
                    'z': getattr(report.new_location, 'z', 0)
                }
            
            if hasattr(report, 'quantity'):
                changes['quantity'] = report.quantity
            
            if hasattr(report, 'status'):
                changes['status'] = report.status
            
            # 同步到 ResourceTracker
            sync_success = self.resource_tracker.update_material_state(
                report.resource_id, 
                changes, 
                report.change_type
            )
            
            result = {
                'processed': True,
                'resource_id': report.resource_id,
                'change_type': report.change_type,
                'next_actions': [],
                'tracker_sync': sync_success
            }
            
            # 发送 ROS2 ResourceUpdate 请求到 host node
            if sync_success:
                try:
                    self._send_resource_update_to_host(report.resource_id, changes)
                    result['ros_update_sent'] = True
                    result['next_actions'].append('ros_update_completed')
                except Exception as e:
                    logger.warning(f"发送ROS2资源更新失败: {e}")
                    result['ros_update_sent'] = False
                    result['warnings'] = [f"ROS2更新失败: {str(e)}"]
            
            # 根据变更类型处理
            if report.change_type == 'created':
                result['next_actions'].append('sync_to_global_registry')
                self._handle_material_created(report)
                
            elif report.change_type == 'updated':
                result['next_actions'].append('update_local_state')
                self._handle_material_updated(report)
                
            elif report.change_type == 'moved':
                result['next_actions'].append('update_location_tracking')
                self._handle_material_moved(report)
                
            elif report.change_type == 'consumed':
                result['next_actions'].append('update_inventory')
                self._handle_material_consumed(report)
                
            elif report.change_type == 'completed':
                result['next_actions'].append('trigger_next_workflow')
                self._handle_material_completed(report)
            
            # 更新本地物料管理系统（如果存在）
            if hasattr(self, 'material_management'):
                try:
                    self.material_management.sync_external_material_change(report)
                except Exception as e:
                    logger.warning(f"同步物料变更到本地管理系统失败: {e}")
            
            return result
            
        except Exception as e:
            logger.error(f"处理物料变更报送失败: {e}")
            return {
                'processed': False,
                'error': str(e),
                'next_actions': ['retry_processing']
            }

    def process_workflow_status_report(self, workstation_id: str, workflow_id: str, 
                                     status: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """处理工作流状态报送"""
        try:
            logger.info(f"处理工作流状态报送: {workstation_id} -> {workflow_id} ({status})")
            
            # 增加接收计数
            self._reports_received_count = getattr(self, '_reports_received_count', 0) + 1
            
            result = {
                'processed': True,
                'workflow_id': workflow_id,
                'status': status
            }
            
            # 这里可以添加工作流状态同步逻辑
            # 例如：更新本地工作流状态、触发后续动作等
            
            return result
            
        except Exception as e:
            logger.error(f"处理工作流状态报送失败: {e}")
            return {'processed': False, 'error': str(e)}

    # ============ 统一报送处理方法（基于LIMS协议规范） ============

    def process_step_finish_report(self, request: WorkstationReportRequest) -> Dict[str, Any]:
        """处理步骤完成报送（统一LIMS协议规范）- 同步到 ResourceTracker"""
        try:
            data = request.data
            logger.info(f"处理步骤完成报送: {data['orderCode']} - {data['stepName']} (步骤ID: {data['stepId']})")
            
            # 增加接收计数
            self._reports_received_count = getattr(self, '_reports_received_count', 0) + 1
            
            # 同步步骤信息到 ResourceTracker
            step_changes = {
                'order_code': data['orderCode'],
                'order_name': data.get('orderName', ''),
                'step_name': data['stepName'],
                'step_id': data['stepId'],
                'sample_id': data['sampleId'],
                'start_time': data['startTime'],
                'end_time': data['endTime'],
                'execution_status': data.get('executionStatus', 'completed'),
                'status': 'step_completed',
                'last_updated': request.request_time
            }
            
            # 更新 ResourceTracker 中的样本状态
            sample_sync_success = False
            if data['sampleId']:
                sample_sync_success = self.resource_tracker.update_material_state(
                    data['sampleId'],
                    {
                        'current_step': data['stepName'],
                        'step_status': 'completed',
                        'last_step_time': data['endTime'],
                        'execution_status': data.get('executionStatus', 'completed')
                    },
                    'step_finished'
                )
            
            result = {
                'processed': True,
                'order_code': data['orderCode'],
                'step_id': data['stepId'],
                'step_name': data['stepName'],
                'sample_id': data['sampleId'],
                'start_time': data['startTime'],
                'end_time': data['endTime'],
                'execution_status': data.get('executionStatus', 'completed'),
                'next_actions': [],
                'sample_sync': sample_sync_success
            }
            
            # 发送 ROS2 ResourceUpdate 到 host node
            if sample_sync_success and data['sampleId']:
                try:
                    self._send_resource_update_to_host(data['sampleId'], step_changes)
                    result['ros_update_sent'] = True
                    result['next_actions'].append('ros_step_update_completed')
                except Exception as e:
                    logger.warning(f"发送ROS2步骤完成更新失败: {e}")
                    result['ros_update_sent'] = False
                    result['warnings'] = [f"ROS2更新失败: {str(e)}"]
            
            # 处理步骤完成逻辑
            try:
                # 更新步骤状态
                result['next_actions'].append('update_step_status')
                
                # 检查是否触发后续步骤
                result['next_actions'].append('check_next_step')
                
                # 更新通量进度
                result['next_actions'].append('update_sample_progress')
                
                # 记录步骤完成事件
                self._record_step_completion(data)
                
            except Exception as e:
                logger.warning(f"步骤完成处理过程中出现警告: {e}")
                result['warnings'] = result.get('warnings', []) + [str(e)]
            
            return result
            
        except Exception as e:
            logger.error(f"处理步骤完成报送失败: {e}")
            return {
                'processed': False,
                'error': str(e),
                'next_actions': ['retry_processing']
            }

    def process_sample_finish_report(self, request: WorkstationReportRequest) -> Dict[str, Any]:
        """处理通量完成报送（统一LIMS协议规范）"""
        try:
            data = request.data
            logger.info(f"处理通量完成报送: {data['orderCode']} - 通量ID: {data['sampleId']} (状态: {data['Status']})")
            
            # 增加接收计数
            self._reports_received_count = getattr(self, '_reports_received_count', 0) + 1
            
            result = {
                'processed': True,
                'order_code': data['orderCode'],
                'sample_id': data['sampleId'],
                'status': data['Status'],
                'start_time': data['startTime'],
                'end_time': data['endTime'],
                'next_actions': []
            }
            
            # 根据通量状态处理
            status = int(data['Status'])
            if status == 20:  # 完成
                result['next_actions'].extend(['update_sample_completed', 'check_order_completion'])
                self._record_sample_completion(data, 'completed')
            elif status == -2:  # 异常停止
                result['next_actions'].extend(['log_sample_error', 'trigger_error_handling'])
                self._record_sample_completion(data, 'error')
            elif status == -3:  # 人工停止或取消
                result['next_actions'].extend(['log_sample_cancelled', 'update_order_status'])
                self._record_sample_completion(data, 'cancelled')
            elif status == 10:  # 开始
                result['next_actions'].append('update_sample_started')
                self._record_sample_start(data)
            elif status == 2:  # 进样
                result['next_actions'].append('update_sample_intake')
                self._record_sample_intake(data)
            
            return result
            
        except Exception as e:
            logger.error(f"处理通量完成报送失败: {e}")
            return {
                'processed': False,
                'error': str(e),
                'next_actions': ['retry_processing']
            }

    def process_order_finish_report(self, request: WorkstationReportRequest, used_materials: List[MaterialUsage]) -> Dict[str, Any]:
        """处理任务完成报送（统一LIMS协议规范）"""
        try:
            data = request.data
            logger.info(f"处理任务完成报送: {data['orderCode']} - {data['orderName']} (状态: {data['status']})")
            
            # 增加接收计数
            self._reports_received_count = getattr(self, '_reports_received_count', 0) + 1
            
            result = {
                'processed': True,
                'order_code': data['orderCode'],
                'order_name': data['orderName'],
                'status': data['status'],
                'start_time': data['startTime'],
                'end_time': data['endTime'],
                'used_materials_count': len(used_materials),
                'next_actions': []
            }
            
            # 根据任务状态处理
            status = int(data['status'])
            if status == 30:  # 完成
                result['next_actions'].extend([
                    'update_order_completed', 
                    'process_material_usage',
                    'generate_completion_report'
                ])
                self._record_order_completion(data, used_materials, 'completed')
            elif status == -11:  # 异常停止
                result['next_actions'].extend([
                    'log_order_error', 
                    'trigger_error_handling',
                    'process_partial_material_usage'
                ])
                self._record_order_completion(data, used_materials, 'error')
            elif status == -12:  # 人工停止或取消
                result['next_actions'].extend([
                    'log_order_cancelled',
                    'revert_material_reservations'
                ])
                self._record_order_completion(data, used_materials, 'cancelled')
            
            # 处理物料使用记录
            if used_materials:
                material_usage_result = self._process_material_usage(used_materials)
                result['material_usage'] = material_usage_result
            
            return result
            
        except Exception as e:
            logger.error(f"处理任务完成报送失败: {e}")
            return {
                'processed': False,
                'error': str(e),
                'next_actions': ['retry_processing']
            }

    # ============ 具体的报送处理方法 ============

    def _handle_material_created(self, report):
        """处理物料创建报送"""
        try:
            # 已废弃的方法，保留用于兼容性
            logger.debug(f"处理物料创建: {getattr(report, 'resource_id', 'unknown')}")
        except Exception as e:
            logger.error(f"处理物料创建失败: {e}")

    def _handle_material_updated(self, report):
        """处理物料更新报送"""
        try:
            logger.debug(f"处理物料更新: {getattr(report, 'resource_id', 'unknown')}")
        except Exception as e:
            logger.error(f"处理物料更新失败: {e}")

    def _handle_material_moved(self, report):
        """处理物料移动报送"""
        try:
            logger.debug(f"处理物料移动: {getattr(report, 'resource_id', 'unknown')}")
        except Exception as e:
            logger.error(f"处理物料移动失败: {e}")

    def _handle_material_consumed(self, report):
        """处理物料消耗报送"""
        try:
            logger.debug(f"处理物料消耗: {getattr(report, 'resource_id', 'unknown')}")
        except Exception as e:
            logger.error(f"处理物料消耗失败: {e}")

    def _handle_material_completed(self, report):
        """处理物料完成报送"""
        try:
            logger.debug(f"处理物料完成: {getattr(report, 'resource_id', 'unknown')}")
        except Exception as e:
            logger.error(f"处理物料完成失败: {e}")
    
    # ============ 工作流控制接口 ============
    def handle_external_error(self, error_request):
        """处理外部错误请求"""
        try:
            logger.error(f"收到外部错误处理请求: {getattr(error_request, 'error_type', 'unknown')}")
            return {
                'success': True,
                'message': "错误已记录",
                'error_code': 'OK'
            }
        except Exception as e:
            logger.error(f"处理外部错误失败: {e}")
            return {
                'success': False,
                'message': f"错误处理失败: {str(e)}",
                'error_code': 'ERROR_HANDLING_FAILED'
            }

    def _process_error_handling(self, error_request, error_record):
        """处理具体的错误类型"""
        return {'success': True, 'actions_taken': ['已转换为统一报送']}
        """处理具体的错误类型"""
        try:
            result = {'success': True, 'actions_taken': []}
            
            # 1. 如果有特定动作ID，标记该动作失败
            if error_request.action_id:
                self._mark_action_failed(error_request.action_id, error_request.error_message)
                result['actions_taken'].append(f"标记动作 {error_request.action_id} 为失败")
            
            # 2. 如果有工作流ID，停止相关工作流
            if error_request.workflow_id:
                self._handle_workflow_error(error_request.workflow_id, error_request.error_message)
                result['actions_taken'].append(f"处理工作流 {error_request.workflow_id} 错误")
            
            # 3. 根据错误类型执行特定处理
            error_type = error_request.error_type.lower()
            
            if error_type in ['material_error', 'resource_error']:
                # 物料相关错误
                material_result = self._handle_material_error(error_request)
                result['actions_taken'].extend(material_result.get('actions', []))
                
            elif error_type in ['device_error', 'communication_error']:
                # 设备通信错误
                device_result = self._handle_device_error(error_request)
                result['actions_taken'].extend(device_result.get('actions', []))
                
            elif error_type in ['workflow_error', 'process_error']:
                # 工作流程错误
                workflow_result = self._handle_process_error(error_request)
                result['actions_taken'].extend(workflow_result.get('actions', []))
                
            else:
                # 通用错误处理
                result['actions_taken'].append("执行通用错误处理")
            
            # 4. 如果是严重错误，触发紧急停止
            if error_request.error_type.lower() in ['critical_error', 'safety_error', 'emergency']:
                self._trigger_emergency_stop(error_request.error_message)
                result['actions_taken'].append("触发紧急停止")
            
            result['message'] = "错误处理完成"
            return result
            
        except Exception as e:
            logger.error(f"错误处理过程失败: {e}")
            return {
                'success': False,
                'message': f"错误处理过程失败: {str(e)}",
                'error_code': 'ERROR_PROCESSING_FAILED'
            }

    def _mark_action_failed(self, action_id: str, error_message: str):
        """标记指定动作为失败"""
        try:
            # 创建失败结果
            failed_result = {
                'success': False,
                'error': True,
                'error_message': error_message,
                'timestamp': time.time(),
                'marked_by_external_error': True
            }
            
            # 存储到动作结果缓存
            self.action_results[action_id] = failed_result
            
            # 如果当前有正在执行的动作，更新其状态
            if self.current_action_context and self.current_action_context.get('action_id') == action_id:
                self.current_action_context['failed'] = True
                self.current_action_context['error_message'] = error_message
            
            logger.info(f"动作 {action_id} 已标记为失败: {error_message}")
            
        except Exception as e:
            logger.error(f"标记动作失败时出错: {e}")

    def _handle_workflow_error(self, workflow_id: str, error_message: str):
        """处理工作流错误"""
        try:
            # 如果是当前正在运行的工作流
            if (self.current_workflow_info and 
                self.current_workflow_info.get('id') == workflow_id):
                
                # 停止当前工作流
                self.stop_workflow(emergency=True)
                logger.info(f"因外部错误停止工作流 {workflow_id}: {error_message}")
            
        except Exception as e:
            logger.error(f"处理工作流错误失败: {e}")

    def _handle_material_error(self, error_request):
        """处理物料相关错误（已废弃，请使用统一报送接口）"""
        return {'success': True, 'message': '物料错误已记录'}
        """处理物料相关错误"""
        actions = []
        try:
            # 可以触发物料重新扫描、位置重置等
            if error_request.context and 'resource_id' in error_request.context:
                resource_id = error_request.context['resource_id']
                # 触发物料状态更新
                actions.append(f"更新物料 {resource_id} 状态")
            
            actions.append("执行物料错误恢复流程")
            
        except Exception as e:
            logger.error(f"处理物料错误失败: {e}")
            
        return {'actions': actions}

    def _handle_device_error(self, error_request):
        """处理设备相关错误（已废弃，请使用统一报送接口）"""
        return {'success': True, 'message': '设备错误已记录'}
        """处理设备错误"""
        actions = []
        try:
            if error_request.device_id:
                # 重置设备连接
                actions.append(f"重置设备 {error_request.device_id} 连接")
                
                # 如果是通信设备，重新建立连接
                if error_request.device_id in self.communication_devices:
                    actions.append(f"重新建立通信设备 {error_request.device_id} 连接")
            
            actions.append("执行设备错误恢复流程")
            
        except Exception as e:
            logger.error(f"处理设备错误失败: {e}")
            
        return {'actions': actions}

    def _handle_process_error(self, error_request):
        """处理流程相关错误（已废弃，请使用统一报送接口）"""
        return {'success': True, 'message': '流程错误已记录'}
        """处理工作流程错误"""
        actions = []
        try:
            # 暂停当前工作流
            if self.current_workflow_status not in [WorkflowStatus.IDLE, WorkflowStatus.STOPPED]:
                actions.append("暂停当前工作流")
                
            actions.append("执行工作流程错误恢复")
            
        except Exception as e:
            logger.error(f"处理工作流程错误失败: {e}")
            
        return {'actions': actions}

    def _trigger_emergency_stop(self, reason: str):
        """触发紧急停止"""
        try:
            logger.critical(f"触发紧急停止: {reason}")
            
            # 停止所有工作流
            self.stop_workflow(emergency=True)
            
            # 设置错误状态
            self.current_workflow_status = WorkflowStatus.ERROR
            
            # 可以在这里添加更多紧急停止逻辑
            # 例如：断开设备连接、保存当前状态等
            
        except Exception as e:
            logger.error(f"执行紧急停止失败: {e}")

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
            "material_status": self.material_status,
            "http_service_running": self.http_service.is_running if self.http_service else False
        }

    # ============ 增强动作执行 - 支持错误处理和追踪 ============

    async def execute_single_action(self, device_id, action_name, action_kwargs):
        """执行单个动作 - 增强版，支持错误处理和动作追踪"""
        # 构建动作ID
        if device_id in ["", None, "self"]:
            action_id = f"/devices/{self.device_id}/{action_name}"
        else:
            action_id = f"/devices/{device_id}/{action_name}"

        # 设置动作上下文
        self.current_action_context = {
            'action_id': action_id,
            'device_id': device_id,
            'action_name': action_name,
            'action_kwargs': action_kwargs,
            'start_time': time.time(),
            'failed': False,
            'error_message': None
        }

        try:
            # 检查是否已被外部标记为失败
            if action_id in self.action_results:
                cached_result = self.action_results[action_id]
                if cached_result.get('marked_by_external_error'):
                    logger.warning(f"动作 {action_id} 已被外部标记为失败")
                    return self._create_failed_result(cached_result['error_message'])

            # 检查动作客户端是否存在
            if action_id not in self._action_clients:
                error_msg = f"找不到动作客户端: {action_id}"
                self.lab_logger().error(error_msg)
                return self._create_failed_result(error_msg)

            # 发送动作请求
            action_client = self._action_clients[action_id]
            goal_msg = convert_to_ros_msg(action_client._action_type.Goal(), action_kwargs)

            self.lab_logger().debug(f"发送动作请求到: {action_id}")
            action_client.wait_for_server()

            # 等待动作完成
            request_future = action_client.send_goal_async(goal_msg)
            handle = await request_future

            if not handle.accepted:
                error_msg = f"动作请求被拒绝: {action_name}"
                self.lab_logger().error(error_msg)
                return self._create_failed_result(error_msg)

            # 在执行过程中检查是否被外部标记为失败
            result_future = await handle.get_result_async()
            
            # 再次检查是否在执行过程中被标记为失败
            if self.current_action_context.get('failed'):
                error_msg = self.current_action_context.get('error_message', '动作被外部标记为失败')
                logger.warning(f"动作 {action_id} 在执行过程中被标记为失败: {error_msg}")
                return self._create_failed_result(error_msg)

            result = result_future.result
            
            # 存储成功结果
            self.action_results[action_id] = {
                'success': True,
                'result': result,
                'timestamp': time.time(),
                'execution_time': time.time() - self.current_action_context['start_time']
            }

            self.lab_logger().debug(f"动作完成: {action_name}")
            return result

        except Exception as e:
            error_msg = f"动作执行异常: {str(e)}"
            logger.error(f"执行动作 {action_id} 失败: {e}\n{traceback.format_exc()}")
            return self._create_failed_result(error_msg)

        finally:
            # 清理动作上下文
            self.current_action_context = None

    def _create_failed_result(self, error_message: str):
        """创建失败结果对象"""
        # 这需要根据具体的动作类型来创建相应的结果对象
        # 这里返回一个通用的失败标识
        class FailedResult:
            def __init__(self, error_msg):
                self.success = False
                self.return_info = json.dumps({
                    "suc": False,
                    "error": True,
                    "error_message": error_msg,
                    "timestamp": time.time()
                })

        return FailedResult(error_message)

    def __del__(self):
        """析构函数 - 清理HTTP服务"""
        try:
            self._stop_http_service()
            self._stop_reporting_service()
        except:
            pass

    # ============ LIMS辅助方法 ============
    
    def _record_step_completion(self, step_data: Dict[str, Any]):
        """记录步骤完成事件"""
        try:
            logger.debug(f"记录步骤完成: {step_data['stepName']} - {step_data['stepId']}")
            # 这里可以添加步骤完成的记录逻辑
            # 例如：更新数据库、发送通知等
        except Exception as e:
            logger.error(f"记录步骤完成失败: {e}")
    
    def _record_sample_completion(self, sample_data: Dict[str, Any], completion_type: str):
        """记录通量完成事件"""
        try:
            logger.debug(f"记录通量完成: {sample_data['sampleId']} - {completion_type}")
            # 这里可以添加通量完成的记录逻辑
        except Exception as e:
            logger.error(f"记录通量完成失败: {e}")
    
    def _record_sample_start(self, sample_data: Dict[str, Any]):
        """记录通量开始事件"""
        try:
            logger.debug(f"记录通量开始: {sample_data['sampleId']}")
            # 这里可以添加通量开始的记录逻辑
        except Exception as e:
            logger.error(f"记录通量开始失败: {e}")
    
    def _record_sample_intake(self, sample_data: Dict[str, Any]):
        """记录通量进样事件"""
        try:
            logger.debug(f"记录通量进样: {sample_data['sampleId']}")
            # 这里可以添加通量进样的记录逻辑
        except Exception as e:
            logger.error(f"记录通量进样失败: {e}")
    
    def _record_order_completion(self, order_data: Dict[str, Any], used_materials: List, completion_type: str):
        """记录任务完成事件"""
        try:
            logger.debug(f"记录任务完成: {order_data['orderCode']} - {completion_type}")
            # 这里可以添加任务完成的记录逻辑
            # 包括物料使用记录的处理
        except Exception as e:
            logger.error(f"记录任务完成失败: {e}")
    
    def _process_material_usage(self, used_materials: List) -> Dict[str, Any]:
        """处理物料使用记录"""
        try:
            logger.debug(f"处理物料使用记录: {len(used_materials)} 条")
            
            processed_materials = []
            for material in used_materials:
                material_record = {
                    'material_id': material.materialId,
                    'location_id': material.locationId,
                    'type_mode': material.typeMode,
                    'used_quantity': material.usedQuantity,
                    'processed_time': time.time()
                }
                processed_materials.append(material_record)
                
                # 更新库存 
                self._update_material_inventory(material)
            
            return {
                'processed_count': len(processed_materials),
                'materials': processed_materials,
                'success': True
            }
            
        except Exception as e:
            logger.error(f"处理物料使用记录失败: {e}")
            return {
                'processed_count': 0,
                'materials': [],
                'success': False,
                'error': str(e)
            }
    
    def _update_material_inventory(self, material):
        """更新物料库存"""
        try:
            # 这里可以添加库存更新逻辑
            # 例如：调用库存管理系统API、更新本地缓存等
            logger.debug(f"更新物料库存: {material.materialId} - 使用量: {material.usedQuantity}")
        except Exception as e:
            logger.error(f"更新物料库存失败: {e}")
