import requests
from typing import List, Sequence, Optional, Union, Literal
# from geometry_msgs.msg import Point
# from unilabos_msgs.msg import Resource
from pylabrobot.resources import (
    Resource,
    TipRack,
    Container,
    Coordinate,
    Well
)
# from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker  # type: ignore
# from .liquid_handler_abstract import LiquidHandlerAbstract

import json
from typing import Sequence, Optional, List, Union, Literal

steps_info = ''' 
{
  "steps": [
    {
      "step_number": 1,
      "operation": "transfer",
      "description": "转移PCR产物或酶促反应液至0.05ml 96孔板中",
      "parameters": {
        "source": "P1",
        "target": "P11",
        "tip_rack": "BC230",
        "volume": 50
      }
    },
    {
      "step_number": 2,
      "operation": "transfer",
      "description": "加入2倍体积Bind Beads BC至产物中",
      "parameters": {
        "source": "P2",
        "target": "P11",
        "tip_rack": "BC230",
        "volume": 100
      }
    },
    {
      "step_number": 3,
      "operation": "move_labware",
      "description": "移动P11至Orbital1用于振荡混匀",
      "parameters": {
        "source": "P11",
        "target": "Orbital1"
      }
    },
    {
      "step_number": 4,
      "operation": "oscillation",
      "description": "在Orbital1上振荡混匀Bind Beads BC与PCR产物（700-900rpm，300秒）",
      "parameters": {
        "rpm": 800,
        "time": 300
      }
    },
    {
      "step_number": 5,
      "operation": "move_labware",
      "description": "移动混匀后的板回P11",
      "parameters": {
        "source": "Orbital1",
        "target": "P11"
      }
    },
    {
      "step_number": 6,
      "operation": "move_labware",
      "description": "将P11移动到磁力架（P12）吸附3分钟",
      "parameters": {
        "source": "P11",
        "target": "P12"
      }
    },
    {
      "step_number": 7,
      "operation": "incubation",
      "description": "磁力架上室温静置3分钟完成吸附",
      "parameters": {
        "time": 180
      }
    },
    {
      "step_number": 8,
      "operation": "transfer",
      "description": "去除上清液至废液槽",
      "parameters": {
        "source": "P12",
        "target": "P22",
        "tip_rack": "BC230",
        "volume": 150
      }
    },
    {
      "step_number": 9,
      "operation": "transfer",
      "description": "加入300-500μl 75%乙醇清洗",
      "parameters": {
        "source": "P3",
        "target": "P12",
        "tip_rack": "BC230",
        "volume": 400
      }
    },
    {
      "step_number": 10,
      "operation": "move_labware",
      "description": "移动清洗板到Orbital1进行振荡",
      "parameters": {
        "source": "P12",
        "target": "Orbital1"
      }
    },
    {
      "step_number": 11,
      "operation": "oscillation",
      "description": "乙醇清洗液振荡混匀（700-900rpm, 45秒）",
      "parameters": {
        "rpm": 800,
        "time": 45
      }
    },
    {
      "step_number": 12,
      "operation": "move_labware",
      "description": "振荡后将板移回磁力架P12吸附",
      "parameters": {
        "source": "Orbital1",
        "target": "P12"
      }
    },
    {
      "step_number": 13,
      "operation": "incubation",
      "description": "吸附3分钟",
      "parameters": {
        "time": 180
      }
    },
    {
      "step_number": 14,
      "operation": "transfer",
      "description": "去除乙醇上清液至废液槽",
      "parameters": {
        "source": "P12",
        "target": "P22",
        "tip_rack": "BC230",
        "volume": 400
      }
    },
    {
      "step_number": 15,
      "operation": "transfer",
      "description": "第二次加入300-500μl 75%乙醇清洗",
      "parameters": {
        "source": "P3",
        "target": "P12",
        "tip_rack": "BC230",
        "volume": 400
      }
    },
    {
      "step_number": 16,
      "operation": "move_labware",
      "description": "再次移动清洗板到Orbital1振荡",
      "parameters": {
        "source": "P12",
        "target": "Orbital1"
      }
    },
    {
      "step_number": 17,
      "operation": "oscillation",
      "description": "再次乙醇清洗液振荡混匀（700-900rpm, 45秒）",
      "parameters": {
        "rpm": 800,
        "time": 45
      }
    },
    {
      "step_number": 18,
      "operation": "move_labware",
      "description": "振荡后板送回磁力架P12吸附",
      "parameters": {
        "source": "Orbital1",
        "target": "P12"
      }
    },
    {
      "step_number": 19,
      "operation": "incubation",
      "description": "再次吸附3分钟",
      "parameters": {
        "time": 180
      }
    },
    {
      "step_number": 20,
      "operation": "transfer",
      "description": "去除乙醇上清液至废液槽",
      "parameters": {
        "source": "P12",
        "target": "P22",
        "tip_rack": "BC230",
        "volume": 400
      }
    },
    {
      "step_number": 21,
      "operation": "incubation",
      "description": "空气干燥15分钟",
      "parameters": {
        "time": 900
      }
    },
    {
      "step_number": 22,
      "operation": "transfer",
      "description": "加30-50μl Elution Buffer洗脱",
      "parameters": {
        "source": "P4",
        "target": "P12",
        "tip_rack": "BC230",
        "volume": 40
      }
    },
    {
      "step_number": 23,
      "operation": "move_labware",
      "description": "移动到Orbital1振荡混匀（60秒）",
      "parameters": {
        "source": "P12",
        "target": "Orbital1"
      }
    },
    {
      "step_number": 24,
      "operation": "oscillation",
      "description": "Elution Buffer振荡混匀（700-900rpm, 60秒）",
      "parameters": {
        "rpm": 800,
        "time": 60
      }
    },
    {
      "step_number": 25,
      "operation": "move_labware",
      "description": "振荡后送回磁力架P12",
      "parameters": {
        "source": "Orbital1",
        "target": "P12"
      }
    },
    {
      "step_number": 26,
      "operation": "incubation",
      "description": "室温静置3分钟（洗脱反应）",
      "parameters": {
        "time": 180
      }
    },
    {
      "step_number": 27,
      "operation": "transfer",
      "description": "将上清液（DNA）转移到新板（P13）",
      "parameters": {
        "source": "P12",
        "target": "P13",
        "tip_rack": "BC230",
        "volume": 40
      }
    }
  ]
}
'''



#class LiquidHandlerBiomek(LiquidHandlerAbstract):


class LiquidHandlerBiomek:
    """
    Biomek液体处理器的实现类，继承自LiquidHandlerAbstract。
    该类用于处理Biomek液体处理器的特定操作。
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._status = "Idle"  # 初始状态为 Idle
        self._success = False  # 初始成功状态为 False
        self._status_queue = kwargs.get("status_queue", None)  # 状态队列
        self.temp_protocol = {}
        self.py32_path = "/opt/py32"  # Biomek的Python 3.2路径
        self.technique = {
            'MC P300 high':{       
                "use_channels": "Span8",
                "asp_vols": [50],
                "asp_flow_rates": 50,
                "dis_flow_rates": 50,
                "offsets": None,
                "touch_tip": True,
                "liquid_height": 20,
                "blow_out_air_volume": 20,
                "spread": "wide",
                "is_96_well": True,
                "mix_stage": "none",
                "mix_times": None,
                "mix_vol": None,
                "mix_rate": None,
                "mix_liquid_height": None,
                "delays": None,
                "none_keys": [] 
            }
        }


    def create_protocol(
        self,
        protocol_name: str,
        protocol_description: str,
        protocol_version: str,
        protocol_author: str,
        protocol_date: str,
        protocol_type: str,
        none_keys: List[str] = [],
    ):
        """
        创建一个新的协议。

        Args:
            protocol_name (str): 协议名称
            protocol_description (str): 协议描述
            protocol_version (str): 协议版本
            protocol_author (str): 协议作者
            protocol_date (str): 协议日期
            protocol_type (str): 协议类型
            none_keys (List[str]): 需要设置为None的键列表

        Returns:
            dict: 创建的协议字典
        """
        self.temp_protocol = {
            "meta": {
                "name": protocol_name,
                "description": protocol_description,
                "version": protocol_version,
                "author": protocol_author,
                "date": protocol_date,
                "type": protocol_type,
            },
            "labwares": [],
            "steps": [],
        }
        return self.temp_protocol

#     def run_protocol(self):
#         """
#         执行创建的实验流程。
#         工作站的完整执行流程是，
#         从 create_protocol 开始，创建新的 method，
#         随后执行 transfer_liquid 等操作向实验流程添加步骤，
#         最后 run_protocol 执行整个方法。

#         Returns:
#             dict: 执行结果
#         """
#         #use popen or subprocess to create py32 process and communicate send the temp protocol to it
#         if not self.temp_protocol:
#             raise ValueError("No protocol created. Please create a protocol first.")

#         # 模拟执行协议
#         self._status = "Running"
#         self._success = True
#         # 在这里可以添加实际执行协议的逻辑

#         response = requests.post("localhost:5000/api/protocols", json=self.temp_protocol)

#     def create_resource(
#         self,
#         resource_tracker: DeviceNodeResourceTracker,
#         resources: list[Resource],
#         bind_parent_id: str,
#         bind_location: dict[str, float],
#         liquid_input_slot: list[int],
#         liquid_type: list[str],
#         liquid_volume: list[int],
#         slot_on_deck: int,
#         res_id,
#         class_name,
#         bind_locations,
#         parent
#     ):
#         """
#         创建一个新的资源。

#         Args:
#             device_id (str): 设备ID
#             res_id (str): 资源ID
#             class_name (str): 资源类名
#             parent (str): 父级ID
#             bind_locations (Point): 绑定位置
#             liquid_input_slot (list[int]): 液体输入槽列表
#             liquid_type (list[str]): 液体类型列表
#             liquid_volume (list[int]): 液体体积列表
#             slot_on_deck (int): 甲板上的槽位

#         Returns:
#             dict: 创建的资源字典
#         """
#         # TODO：需要对好接口，下面这个是临时的
#         resource = {
#             "id": res_id,
#             "class": class_name,
#             "parent": parent,
#             "bind_locations": bind_locations.to_dict(),
#             "liquid_input_slot": liquid_input_slot,
#             "liquid_type": liquid_type,
#             "liquid_volume": liquid_volume,
#             "slot_on_deck": slot_on_deck,
#         }
#         self.temp_protocol["labwares"].append(resource)
#         return resource

    def transfer_liquid(
        self,
        sources: Sequence[Container],
        targets: Sequence[Container],
        tip_racks: Sequence[TipRack],
        solvent: Optional[str] = None,
        TipLocation :str = None, 
        *,
        use_channels: Optional[List[int]] = None,
        asp_vols: Union[List[float], float],
        dis_vols: Union[List[float], float],
        asp_flow_rates: Optional[List[Optional[float]]] = None,
        dis_flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        touch_tip: bool = False,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Literal["wide", "tight", "custom"] = "wide",
        is_96_well: bool = False,
        mix_stage: Optional[Literal["none", "before", "after", "both"]] = "none",
        mix_times: Optional[int] = None,
        mix_vol: Optional[int] = None,
        mix_rate: Optional[int] = None,
        mix_liquid_height: Optional[float] = None,
        delays: Optional[List[int]] = None,
        none_keys: List[str] = []
    ):

        transfer_params = {
            "Span8": False,
            "Pod": "Pod1",
            "items": {},                      
            "Wash": False,
            "Dynamic?": True,
            "AutoSelectActiveWashTechnique": False,
            "ActiveWashTechnique": "",
            "ChangeTipsBetweenDests": False,
            "ChangeTipsBetweenSources": False,
            "DefaultCaption": "",             
            "UseExpression": False,
            "LeaveTipsOn": False,
            "MandrelExpression": "",
            "Repeats": "1",
            "RepeatsByVolume": False,
            "Replicates": "1",
            "ShowTipHandlingDetails": False,
            "ShowTransferDetails": True,
            "Solvent": "Water",
            "Span8Wash": False,
            "Span8WashVolume": "2",
            "Span8WasteVolume": "1",
            "SplitVolume": False,
            "SplitVolumeCleaning": False,
            "Stop": "Destinations",
            "TipLocation": "BC1025F",
            "UseCurrentTips": False,    
            "UseDisposableTips": True,
            "UseFixedTips": False,
            "UseJIT": True,
            "UseMandrelSelection": True,
            "UseProbes": [True, True, True, True, True, True, True, True],
            "WashCycles": "1",
            "WashVolume": "110%",
            "Wizard": False
        }

        items: dict = {}
        for idx, (src, dst) in enumerate(zip(sources, targets)):
            items[str(idx)] = {
                "Source": str(src),
                "Destination": str(dst),
                "Volume": dis_vols[idx] 
            }
        transfer_params["items"] = items
        transfer_params["Solvent"] =  solvent if solvent else "Water"
        transfer_params["TipLocation"] = TipLocation

        if len(tip_racks) == 1:
                transfer_params['UseCurrentTips'] = True
        elif len(tip_racks) > 1:
            transfer_params["ChangeTipsBetweenDests"] = True

        self.temp_protocol["steps"].append(transfer_params)

        return 
        
    def transfer_biomek(
            self,
            parameters: dict,
            technique: str,
    ):
        """
        创建一个Biomek液体处理器的转移步骤。

        Args:
            step_number (int): 步骤编号
            operation (str): 操作类型
            description (str): 步骤描述
            parameters (dict): 步骤参数

        Returns:
            dict: 创建的转移步骤字典
        """
        
        sources = parameters.get("source")
        targets = parameters.get("target")
        tip_rack = parameters.get("tip_rack")
        volume = parameters.get("volume")
        liquid_type = parameters.get("liquid_type", "Well Content")
        other_params = self.technique.get(technique, {})

        self.transfer_liquid(
            sources=[sources],
            targets=[targets],
            TipLocation=tip_rack,
            tip_racks=[tip_rack],
            solvent=liquid_type,
            dis_vols=[volume],
            **other_params
        )

        return 

handler = LiquidHandlerBiomek()

handler.temp_protocol = {
    "meta": {},
    "labwares": [],
    "steps": []
}
input_steps = json.loads(steps_info)

for step in input_steps['steps']:
    if step['operation'] != 'transfer':
        continue
    parameters = step['parameters']
    handler.transfer_biomek(parameters, technique='MC P300 high')

print(json.dumps(handler.temp_protocol['steps'],indent=4, ensure_ascii=False))

