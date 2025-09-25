"""
Bioyond工作站实现
Bioyond Workstation Implementation

集成Bioyond物料管理的工作站示例
"""
from typing import Dict, Any, List, Optional, Union
import json
import requests
from datetime import datetime
from unilabos.devices.workstation.workstation_base import WorkstationBase
# from unilabos.devices.workstation.bioyond_cell.bioyond_material_management import BioyondMaterialManagement
from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.utils.log import logger
from pylabrobot.resources.deck import Deck
# from unilabos.resources.graphio import resource_bioyond_to_ulab, resource_ulab_to_plr




class BioyondWorkstation(WorkstationBase):
    """Bioyond工作站
    
    集成Bioyond物料管理的工作站实现
    """
    
    def __init__(
        self,
        bioyond_config: Optional[Dict[str, Any]] = None,
        station_resource: Optional[Dict[str, Any]] = None,
        *args,
        **kwargs,
    ):
        # 设置Bioyond配置
        self.bioyond_config = bioyond_config or {
            "base_url": "http://192.168.1.200:44388",
            "api_key": "8A819E5C",
            "sync_interval": 30,
            "timeout": 30
        }
        
        # 设置默认deck配置
        
        # 初始化父类
        super().__init__(
            #桌子
            deck=Deck,
            station_resource=station_resource,
            *args,
            **kwargs,
        )
        # TODO: self._ros_node里面拿属性
        logger.info(f"Bioyond工作站初始化完成")

    


    def api_lims_scheduler_start(self, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
            """
            向Bioyond系统发送POST请求
            :param endpoint: 接口路径，例如 "start_schedule"
            :param payload: 请求体字典，会自动附加apikey和请求时间
            :return: 返回的json数据
            """
            url = f"{self.bioyond_config['base_url'].rstrip('/')}/api/lims/scheduler/start"
            print(url)
            headers = {"Content-Type": "application/json"}

            # 构建请求体
            data = {
                "apikey": self.bioyond_config["api_key"],
                "requestTime": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ")
            }
            print(data)
            if payload:
                data.update(payload)

            try:
                response = requests.post(url, json=data, timeout=self.bioyond_config.get("timeout", 30))
                response.raise_for_status()
                return response.json()
            except Exception as e:
                logger.error(f"POST {url} 请求失败: {e}")
                return {"error": str(e)}




if __name__ == "__main__":
    ws = BioyondWorkstation()
    result = ws.api_lims_scheduler_start()
    print("返回结果:", result)



# if __name__ == "__main__":
#     # 创建示例工作站
#     workstation = create_bioyond_workstation_example()
    
#     # 从文件加载测试数据
#     workstation.load_bioyond_data_from_file("bioyond_test.json")
    
    # # 获取状态
    # status = workstation.get_bioyond_status()
    # print("Bioyond工作站状态:", status)


    # from pathlib import Path, PurePath
    # import json
    
    # text = Path(__file__).parent.joinpath("bioyond_test.json").read_text(encoding="utf-8")
    # bioyond_resources = json.loads(text)

    # #bioyond_resources = json.loads("bioyond_test.json")

    # bioyond_resources_unilab = resource_bioyond_to_ulab(bioyond_resources)
    # ulab_resources = resource_ulab_to_plr(bioyond_resources_unilab)

    # deck = Deck(size_x=2000,
    #             size_y=653.5,
    #             size_z=900)
    # for r in ulab_resources:
    #     deck.assign_child_resource(r, rails=r.location.rails[0])



#     # 使用示例
# def create_bioyond_workstation_example():
#     """创建Bioyond工作站示例"""
    
#     # 配置参数
#     device_id = "bioyond_workstation_001"
    
#     # 子资源配置
#     children = {
#         "plate_1": {
#             "name": "plate_1",
#             "type": "plate",
#             "position": {"x": 100, "y": 100, "z": 0},
#             "config": {
#                 "size_x": 127.76,
#                 "size_y": 85.48,
#                 "size_z": 14.35,
#                 "model": "Generic 96 Well Plate"
#             }
#         }
#     }
    
#     # Bioyond配置
#     bioyond_config = {
#         "base_url": "http://bioyond.example.com/api",
#         "api_key": "your_api_key_here",
#         "sync_interval": 60,  # 60秒同步一次
#         "timeout": 30
#     }
    
#     # Deck配置
#     deck_config = {
#         "size_x": 1000.0,
#         "size_y": 1000.0,
#         "size_z": 100.0,
#         "model": "BioyondDeck"
#     }
    
#     # 创建工作站
#     workstation = BioyondWorkstation(
#         device_id=device_id,
#         children=children,
#         deck_config=deck_config,
#         bioyond_config=bioyond_config
#     )
    
#     return workstation