"""
Bioyond工作站实现
Bioyond Workstation Implementation

集成Bioyond物料管理的工作站示例
"""
from typing import Dict, Any, List, Optional, Union
import json

from unilabos.devices.workstation.workstation_base import WorkstationBase, ResourceSynchronizer
from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.utils.log import logger
from unilabos.resources.graphio import resource_bioyond_to_plr


class BioyondWorkstation(WorkstationBase):
    """Bioyond工作站
    
    集成Bioyond物料管理的工作站实现
    """
    
    def __init__(
        self,
        bioyond_config: Optional[Dict[str, Any]] = None,
        deck: Optional[str, Any] = None,
        *args,
        **kwargs,
    ):
        # 设置Bioyond配置
        self.bioyond_config = bioyond_config or {
            "base_url": "http://localhost:8080",
            "api_key": "",
            "sync_interval": 30,
            "timeout": 30
        }
        
        # 设置默认deck配置
        
        # 初始化父类
        super().__init__(
            #桌子
            deck=deck,
            *args,
            **kwargs,
        )
        
        # TODO: self._ros_node里面拿属性
        logger.info(f"Bioyond工作站初始化完成")

    def _create_communication_module(self):
        """创建Bioyond通信模块"""
        # 暂时返回None，因为工作站基类没有强制要求通信模块
        return None
    
    def _create_material_management_module(self) -> BioyondMaterialManagement:
        """创建Bioyond物料管理模块"""
        # 获取必要的属性，如果不存在则使用默认值
        device_id = getattr(self, 'device_id', 'bioyond_workstation')
        resource_tracker = getattr(self, 'resource_tracker', None)
        children_config = getattr(self, '_children', {})
        
       
    
    def _register_supported_workflows(self):
        """注册Bioyond支持的工作流"""
        from unilabos.devices.workstation.workstation_base import WorkflowInfo
        
        # Bioyond物料同步工作流
        self.supported_workflows["bioyond_sync"] = WorkflowInfo(
            name="bioyond_sync",
            description="从Bioyond系统同步物料",
            parameters={
                "sync_type": {"type": "string", "default": "full", "options": ["full", "incremental"]},
                "force_sync": {"type": "boolean", "default": False}
            }
        )
        
        # Bioyond物料更新工作流
        self.supported_workflows["bioyond_update"] = WorkflowInfo(
            name="bioyond_update",
            description="将本地物料变更同步到Bioyond",
            parameters={
                "material_ids": {"type": "list", "default": []},
                "sync_all": {"type": "boolean", "default": True}
            }
        )
        
        logger.info(f"注册了 {len(self.supported_workflows)} 个Bioyond工作流")
    
    async def execute_bioyond_sync_workflow(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """执行Bioyond同步工作流"""
        try:
            sync_type = parameters.get("sync_type", "full")
            force_sync = parameters.get("force_sync", False)
            
            logger.info(f"开始执行Bioyond同步工作流: {sync_type}")
            
            # 获取物料管理模块
            material_manager = self.material_management
            
            if sync_type == "full":
                # 全量同步
                success = await material_manager.sync_from_bioyond()
            else:
                # 增量同步（这里可以实现增量同步逻辑）
                success = await material_manager.sync_from_bioyond()
            
            if success:
                result = {
                    "status": "success",
                    "message": f"Bioyond同步完成: {sync_type}",
                    "synced_resources": len(material_manager.plr_resources)
                }
            else:
                result = {
                    "status": "failed",
                    "message": "Bioyond同步失败"
                }
            
            logger.info(f"Bioyond同步工作流执行完成: {result['status']}")
            return result
            
        except Exception as e:
            logger.error(f"Bioyond同步工作流执行失败: {e}")
            return {
                "status": "error",
                "message": str(e)
            }
    
    async def execute_bioyond_update_workflow(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """执行Bioyond更新工作流"""
        try:
            material_ids = parameters.get("material_ids", [])
            sync_all = parameters.get("sync_all", True)
            
            logger.info(f"开始执行Bioyond更新工作流: sync_all={sync_all}")
            
            # 获取物料管理模块
            material_manager = self.material_management
            
            if sync_all:
                # 同步所有物料
                success_count = 0
                for resource in material_manager.plr_resources.values():
                    success = await material_manager.sync_to_bioyond(resource)
                    if success:
                        success_count += 1
            else:
                # 同步指定物料
                success_count = 0
                for material_id in material_ids:
                    resource = material_manager.find_material_by_id(material_id)
                    if resource:
                        success = await material_manager.sync_to_bioyond(resource)
                        if success:
                            success_count += 1
            
            result = {
                "status": "success",
                "message": f"Bioyond更新完成",
                "updated_resources": success_count,
                "total_resources": len(material_ids) if not sync_all else len(material_manager.plr_resources)
            }
            
            logger.info(f"Bioyond更新工作流执行完成: {result['status']}")
            return result
            
        except Exception as e:
            logger.error(f"Bioyond更新工作流执行失败: {e}")
            return {
                "status": "error",
                "message": str(e)
            }
    
    def get_bioyond_status(self) -> Dict[str, Any]:
        """获取Bioyond系统状态"""
        try:
            material_manager = self.material_management
            
            return {
                "bioyond_connected": material_manager.bioyond_api_client is not None,
                "sync_interval": material_manager.sync_interval,
                "total_resources": len(material_manager.plr_resources),
                "deck_size": {
                    "x": material_manager.plr_deck.size_x,
                    "y": material_manager.plr_deck.size_y,
                    "z": material_manager.plr_deck.size_z
                },
                "bioyond_config": self.bioyond_config
            }
        except Exception as e:
            logger.error(f"获取Bioyond状态失败: {e}")
            return {
                "error": str(e)
            }
    
    def load_bioyond_data_from_file(self, file_path: str) -> bool:
        """从文件加载Bioyond数据（用于测试）"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                bioyond_data = json.load(f)
            
            # 获取物料管理模块
            material_manager = self.material_management
            
            # 转换为UniLab格式
            if isinstance(bioyond_data, dict) and "data" in bioyond_data:
                unilab_resources = material_manager.resource_bioyond_container_to_ulab(bioyond_data)
            else:
                unilab_resources = material_manager.resource_bioyond_to_ulab(bioyond_data)
            
            # 分配到Deck
            import asyncio
            asyncio.create_task(material_manager._assign_resources_to_deck(unilab_resources))
            
            logger.info(f"从文件 {file_path} 加载了 {len(unilab_resources)} 个Bioyond资源")
            return True
            
        except Exception as e:
            logger.error(f"从文件加载Bioyond数据失败: {e}")
            return False


# 使用示例
def create_bioyond_workstation_example():
    """创建Bioyond工作站示例"""
    
    # 配置参数
    device_id = "bioyond_workstation_001"
    
    # 子资源配置
    children = {
        "plate_1": {
            "name": "plate_1",
            "type": "plate",
            "position": {"x": 100, "y": 100, "z": 0},
            "config": {
                "size_x": 127.76,
                "size_y": 85.48,
                "size_z": 14.35,
                "model": "Generic 96 Well Plate"
            }
        }
    }
    
    # Bioyond配置
    bioyond_config = {
        "base_url": "http://bioyond.example.com/api",
        "api_key": "your_api_key_here",
        "sync_interval": 60,  # 60秒同步一次
        "timeout": 30
    }
    
    # Deck配置
    deck_config = {
        "size_x": 1000.0,
        "size_y": 1000.0,
        "size_z": 100.0,
        "model": "BioyondDeck"
    }
    
    # 创建工作站
    workstation = BioyondWorkstation(
        station_resource=deck_config,
        bioyond_config=bioyond_config,
        deck_config=deck_config,
    )
    
    return workstation


if __name__ == "__main__":
    # 创建示例工作站
    #workstation = create_bioyond_workstation_example()
    
    # 从文件加载测试数据
    #workstation.load_bioyond_data_from_file("bioyond_test_yibin.json")
    
    # 获取状态
    #status = workstation.get_bioyond_status()
    #print("Bioyond工作站状态:", status)

    # 创建测试数据 - 使用resource_bioyond_container_to_ulab函数期望的格式
  
  # 读取 bioyond_resources_unilab_output3 copy.json 文件
    from unilabos.resources.graphio import resource_ulab_to_plr, convert_resources_to_type
    from Bioyond_wuliao import *
    from typing import List
    from pylabrobot.resources import Resource as PLRResource
    import json
    from pylabrobot.resources.deck import Deck
    from pylabrobot.resources.coordinate import Coordinate
    
    with open("./bioyond_test_yibin3_unilab_result_corr.json", "r", encoding="utf-8") as f:
        bioyond_resources_unilab = json.load(f)
    print(f"成功读取 JSON 文件，包含 {len(bioyond_resources_unilab)} 个资源")
    ulab_resources = convert_resources_to_type(bioyond_resources_unilab, List[PLRResource])
    print(f"转换结果类型: {type(ulab_resources)}")
    print(f"转换结果长度: {len(ulab_resources) if ulab_resources else 0}")
    deck = Deck(size_x=2000,
                size_y=653.5,
                size_z=900)

    Stack0 = Stack(name="Stack0", location=Coordinate(0, 100, 0))
    Stack1 = Stack(name="Stack1", location=Coordinate(100, 100, 0))
    Stack2 = Stack(name="Stack2", location=Coordinate(200, 100, 0))
    Stack3 = Stack(name="Stack3", location=Coordinate(300, 100, 0))
    Stack4 = Stack(name="Stack4", location=Coordinate(400, 100, 0))
    Stack5 = Stack(name="Stack5", location=Coordinate(500, 100, 0))

    deck.assign_child_resource(Stack1, Stack1.location)
    deck.assign_child_resource(Stack2, Stack2.location)
    deck.assign_child_resource(Stack3, Stack3.location)
    deck.assign_child_resource(Stack4, Stack4.location)
    deck.assign_child_resource(Stack5, Stack5.location)

    Stack0.assign_child_resource(ulab_resources[0], Stack0.location)
    Stack1.assign_child_resource(ulab_resources[1], Stack1.location)
    Stack2.assign_child_resource(ulab_resources[2], Stack2.location)
    Stack3.assign_child_resource(ulab_resources[3], Stack3.location)
    Stack4.assign_child_resource(ulab_resources[4], Stack4.location)
    Stack5.assign_child_resource(ulab_resources[5], Stack5.location)

    from unilabos.resources.graphio import convert_resources_from_type
    from unilabos.app.web.client import http_client

    resources = convert_resources_from_type([deck], [PLRResource])

    
    print(resources)
    http_client.remote_addr = "https://uni-lab.bohrium.com/api/v1"
    #http_client.auth = "9F05593C"
    http_client.auth = "ED634D1C"
    http_client.resource_add(resources, database_process_later=False)