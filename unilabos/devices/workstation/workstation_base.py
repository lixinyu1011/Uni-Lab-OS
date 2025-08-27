"""
工作站基类
Workstation Base Class - 简化版

基于PLR Deck的简化工作站架构
专注于核心物料系统和工作流管理
"""
import time
from typing import Dict, Any, List, Optional, Union
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

try:
    from pylabrobot.resources import Deck, Resource as PLRResource
    PYLABROBOT_AVAILABLE = True
except ImportError:
    PYLABROBOT_AVAILABLE = False
    class Deck: pass
    class PLRResource: pass

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


class ResourceSynchronizer(ABC):
    """资源同步器基类
    
    负责与外部物料系统的同步，并对 self.deck 做修改
    """
    
    def __init__(self, workstation: 'WorkstationBase'):
        self.workstation = workstation
        
    @abstractmethod
    async def sync_from_external(self) -> bool:
        """从外部系统同步物料到本地deck"""
        pass
    
    @abstractmethod
    async def sync_to_external(self, plr_resource: PLRResource) -> bool:
        """将本地物料同步到外部系统"""
        pass
    
    @abstractmethod
    async def handle_external_change(self, change_info: Dict[str, Any]) -> bool:
        """处理外部系统的变更通知"""
        pass


class WorkstationBase(ABC):
    """工作站基类 - 简化版
    
    核心功能：
    1. 基于 PLR Deck 的物料系统，支持格式转换
    2. 可选的资源同步器支持外部物料系统
    3. 简化的工作流管理
    """

    def __init__(
        self,
        device_id: str,
        deck_config: Dict[str, Any],
        children: Optional[Dict[str, Any]] = None,
        resource_synchronizer: Optional[ResourceSynchronizer] = None,
        *args,
        **kwargs,
    ):
        if not PYLABROBOT_AVAILABLE:
            raise ImportError("PyLabRobot 未安装，无法创建工作站")
        
        # 基本配置
        self.device_id = device_id
        self.deck_config = deck_config
        self.children = children or {}
        
        # PLR 物料系统
        self.deck: Optional[Deck] = None
        self.plr_resources: Dict[str, PLRResource] = {}
        
        # 资源同步器（可选）
        self.resource_synchronizer = resource_synchronizer
        
        # 硬件接口
        self.hardware_interface: Union[Any, str] = None
        
        # 协议节点引用（用于代理模式）
        self._workstation_node: Optional['ROS2WorkstationNode'] = None
        
        # 工作流状态
        self.current_workflow_status = WorkflowStatus.IDLE
        self.current_workflow_info = None
        self.workflow_start_time = None
        self.workflow_parameters = {}
        
        # 支持的工作流（静态预定义）
        self.supported_workflows: Dict[str, WorkflowInfo] = {}
        
        # 初始化物料系统
        self._initialize_material_system()
        
        # 注册支持的工作流
        self._register_supported_workflows()
        
        logger.info(f"工作站 {device_id} 初始化完成（简化版）")

    def _initialize_material_system(self):
        """初始化物料系统 - 使用 graphio 转换"""
        try:
            from unilabos.resources.graphio import resource_ulab_to_plr
            
            # 1. 合并 deck_config 和 children 创建完整的资源树
            complete_resource_config = self._create_complete_resource_config()
            
            # 2. 使用 graphio 转换为 PLR 资源
            self.deck = resource_ulab_to_plr(complete_resource_config, plr_model=True)
            
            # 3. 建立资源映射
            self._build_resource_mappings(self.deck)
            
            # 4. 如果有资源同步器，执行初始同步
            if self.resource_synchronizer:
                # 这里可以异步执行，暂时跳过
                pass
            
            logger.info(f"工作站 {self.device_id} 物料系统初始化成功，创建了 {len(self.plr_resources)} 个资源")
            
        except Exception as e:
            logger.error(f"工作站 {self.device_id} 物料系统初始化失败: {e}")
            raise

    def _create_complete_resource_config(self) -> Dict[str, Any]:
        """创建完整的资源配置 - 合并 deck_config 和 children"""
        # 创建主 deck 配置
        deck_resource = {
            "id": f"{self.device_id}_deck",
            "name": f"{self.device_id}_deck",
            "type": "deck",
            "position": {"x": 0, "y": 0, "z": 0},
            "config": {
                "size_x": self.deck_config.get("size_x", 1000.0),
                "size_y": self.deck_config.get("size_y", 1000.0),
                "size_z": self.deck_config.get("size_z", 100.0),
                **{k: v for k, v in self.deck_config.items() if k not in ["size_x", "size_y", "size_z"]}
            },
            "data": {},
            "children": [],
            "parent": None
        }
        
        # 添加子资源
        if self.children:
            children_list = []
            for child_id, child_config in self.children.items():
                child_resource = self._normalize_child_resource(child_id, child_config, deck_resource["id"])
                children_list.append(child_resource)
            deck_resource["children"] = children_list
        
        return deck_resource

    def _normalize_child_resource(self, resource_id: str, config: Dict[str, Any], parent_id: str) -> Dict[str, Any]:
        """标准化子资源配置"""
        return {
            "id": resource_id,
            "name": config.get("name", resource_id),
            "type": config.get("type", "container"),
            "position": self._normalize_position(config.get("position", {})),
            "config": config.get("config", {}),
            "data": config.get("data", {}),
            "children": [],  # 简化版本：只支持一层子资源
            "parent": parent_id
        }

    def _normalize_position(self, position: Any) -> Dict[str, float]:
        """标准化位置信息"""
        if isinstance(position, dict):
            return {
                "x": float(position.get("x", 0)),
                "y": float(position.get("y", 0)),
                "z": float(position.get("z", 0))
            }
        elif isinstance(position, (list, tuple)) and len(position) >= 2:
            return {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]) if len(position) > 2 else 0.0
            }
        else:
            return {"x": 0.0, "y": 0.0, "z": 0.0}

    def _build_resource_mappings(self, deck: Deck):
        """递归构建资源映射"""
        def add_resource_recursive(resource: PLRResource):
            if hasattr(resource, 'name'):
                self.plr_resources[resource.name] = resource
            
            if hasattr(resource, 'children'):
                for child in resource.children:
                    add_resource_recursive(child)
        
        add_resource_recursive(deck)

    # ============ 硬件接口管理 ============
    
    def set_hardware_interface(self, hardware_interface: Union[Any, str]):
        """设置硬件接口"""
        self.hardware_interface = hardware_interface
        logger.info(f"工作站 {self.device_id} 硬件接口设置: {type(hardware_interface).__name__}")

    def set_workstation_node(self, workstation_node: 'ROS2WorkstationNode'):
        """设置协议节点引用（用于代理模式）"""
        self._workstation_node = workstation_node
        logger.info(f"工作站 {self.device_id} 关联协议节点")

    # ============ 设备操作接口 ============
    
    def call_device_method(self, method: str, *args, **kwargs) -> Any:
        """调用设备方法的统一接口"""
        # 1. 代理模式：通过协议节点转发
        if isinstance(self.hardware_interface, str) and self.hardware_interface.startswith("proxy:"):
            if not self._workstation_node:
                raise RuntimeError("代理模式需要设置workstation_node")
            
            device_id = self.hardware_interface[6:]  # 移除 "proxy:" 前缀
            return self._workstation_node.call_device_method(device_id, method, *args, **kwargs)
        
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

    # ============ 物料系统接口 ============
    
    def get_deck(self) -> Deck:
        """获取主 Deck"""
        return self.deck
    
    def get_all_resources(self) -> Dict[str, PLRResource]:
        """获取所有 PLR 资源"""
        return self.plr_resources.copy()
    
    def find_resource_by_name(self, name: str) -> Optional[PLRResource]:
        """按名称查找资源"""
        return self.plr_resources.get(name)
    
    def find_resources_by_type(self, resource_type: type) -> List[PLRResource]:
        """按类型查找资源"""
        return [res for res in self.plr_resources.values() 
                if isinstance(res, resource_type)]
    
    async def sync_with_external_system(self) -> bool:
        """与外部物料系统同步"""
        if not self.resource_synchronizer:
            logger.info(f"工作站 {self.device_id} 没有配置资源同步器")
            return True
        
        try:
            success = await self.resource_synchronizer.sync_from_external()
            if success:
                logger.info(f"工作站 {self.device_id} 外部同步成功")
            else:
                logger.warning(f"工作站 {self.device_id} 外部同步失败")
            return success
        except Exception as e:
            logger.error(f"工作站 {self.device_id} 外部同步异常: {e}")
            return False

    # ============ 简化的工作流控制 ============

    def execute_workflow(self, workflow_name: str, parameters: Dict[str, Any]) -> bool:
        """执行工作流"""
        try:
            # 设置工作流状态
            self.current_workflow_status = WorkflowStatus.INITIALIZING
            self.workflow_parameters = parameters
            self.workflow_start_time = time.time()
            
            # 委托给子类实现
            success = self._execute_workflow_impl(workflow_name, parameters)
            
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

    def stop_workflow(self, emergency: bool = False) -> bool:
        """停止工作流"""
        try:
            if self.current_workflow_status in [WorkflowStatus.IDLE, WorkflowStatus.STOPPED]:
                logger.warning(f"工作站 {self.device_id} 没有正在运行的工作流")
                return True
            
            self.current_workflow_status = WorkflowStatus.STOPPING
            
            # 委托给子类实现
            success = self._stop_workflow_impl(emergency)
            
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

    # ============ 抽象方法 - 子类必须实现 ============
    
    @abstractmethod
    def _register_supported_workflows(self):
        """注册支持的工作流 - 子类必须实现"""
        pass
    
    @abstractmethod
    def _execute_workflow_impl(self, workflow_name: str, parameters: Dict[str, Any]) -> bool:
        """执行工作流的具体实现 - 子类必须实现"""
        pass
    
    @abstractmethod
    def _stop_workflow_impl(self, emergency: bool = False) -> bool:
        """停止工作流的具体实现 - 子类必须实现"""
        pass
