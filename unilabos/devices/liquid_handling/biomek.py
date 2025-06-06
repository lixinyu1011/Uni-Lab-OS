import requests
from typing import List, Sequence, Optional, Union, Literal
from geometry_msgs.msg import Point
from pylabrobot.liquid_handling import LiquidHandler
from unilabos_msgs.msg import Resource

from pylabrobot.resources import (
    TipRack,
    Container,
    Coordinate,
)

from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker  # type: ignore
from .liquid_handler_abstract import LiquidHandlerAbstract

import json
from typing import Sequence, Optional, List, Union, Literal

class LiquidHandlerBiomek(LiquidHandlerAbstract):

    """
    Biomek液体处理器的实现类，继承自LiquidHandlerAbstract。
    该类用于处理Biomek液体处理器的特定操作。
    """

    def __init__(self, backend=None, deck=None, *args, **kwargs):
        super().__init__(backend, deck, *args, **kwargs)
        self._status = "Idle"  # 初始状态为 Idle
        self._success = False  # 初始成功状态为 False
        self._status_queue = kwargs.get("status_queue", None)  # 状态队列
        self.temp_protocol = {}
        self.py32_path = "/opt/py32"  # Biomek的Python 3.2路径
        self.aspirate_techniques = {
            'MC P300 high':{       
                            'Position': 'P1', 
                            'Height': -2.0, 
                            'Volume': '50', 
                            'liquidtype': 'Well Contents', 
                            'WellsX': 12, 
                            'LabwareClass': 'Matrix96_750uL', 
                            'AutoSelectPrototype': True, 
                            'ColsFirst': True, 
                            'CustomHeight': False, 
                            'DataSetPattern': False, 
                            'HeightFrom': 0, 
                            'LocalPattern': True, 
                            'Operation': 'Aspirate', 
                            'OverrideHeight': False, 
                            'Pattern': (True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True),
                            'Prototype': 'MC P300 High', 
                            'ReferencedPattern': '', 
                            'RowsFirst': False, 
                            'SectionExpression': '', 
                            'SelectionInfo': (1,), 
                            'SetMark': True, 
                            'Source': True, 
                            'StartAtMark': False, 
                            'StartAtSelection': True, 
                            'UseExpression': False},
            }
      
        self.dispense_techniques = {
            'MC P300 high':{       
                          'Position': 'P11', 
                          'Height': -2.0, 
                          'Volume': '50', 
                          'liquidtype': 'Tip Contents', 
                          'WellsX': 12, 
                          'LabwareClass': 'Matrix96_750uL', 
                          'AutoSelectPrototype': True, 
                          'ColsFirst': True, 
                          'CustomHeight': False, 
                          'DataSetPattern': False, 
                          'HeightFrom': 0, 
                          'LocalPattern': True, 
                          'Operation': 'Dispense', 
                          'OverrideHeight': False, 
                          'Pattern': (True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True), 
                          'Prototype': 'MC P300 High', 
                          'ReferencedPattern': '', 
                          'RowsFirst': False, 
                          'SectionExpression': '', 
                          'SelectionInfo': (1,), 
                          'SetMark': True, 
                          'Source': False, 
                          'StartAtMark': False, 
                          'StartAtSelection': True, 
                          'UseExpression': False}
        }


    @classmethod
    def deserialize(cls, data: dict, allow_marshal: bool = False) -> LiquidHandler:
        return LiquidHandler.deserialize(data, allow_marshal)


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

    def run_protocol(self):
        """
        执行创建的实验流程。
        工作站的完整执行流程是，
        从 create_protocol 开始，创建新的 method，
        随后执行 transfer_liquid 等操作向实验流程添加步骤，
        最后 run_protocol 执行整个方法。

        Returns:
            dict: 执行结果
        """
        #use popen or subprocess to create py32 process and communicate send the temp protocol to it
        if not self.temp_protocol:
            raise ValueError("No protocol created. Please create a protocol first.")

        # 模拟执行协议
        self._status = "Running"
        self._success = True
        # 在这里可以添加实际执行协议的逻辑

        response = requests.post("localhost:5000/api/protocols", json=self.temp_protocol)

    def create_resource(
        self,
        resource_tracker: DeviceNodeResourceTracker,
        resources: list[Resource],
        bind_parent_id: str,
        bind_location: dict[str, float],
        liquid_input_slot: list[int],
        liquid_type: list[str],
        liquid_volume: list[int],
        slot_on_deck: int,

    ):
        """
        创建一个新的资源。

        Args:
            device_id (str): 设备ID
            res_id (str): 资源ID
            class_name (str): 资源类名
            parent (str): 父级ID
            bind_locations (Point): 绑定位置
            liquid_input_slot (list[int]): 液体输入槽列表
            liquid_type (list[str]): 液体类型列表
            liquid_volume (list[int]): 液体体积列表
            slot_on_deck (int): 甲板上的槽位

        Returns:
            dict: 创建的资源字典
        """
        # TODO：需要对好接口，下面这个是临时的
        for resource in resources:
            res_id = resource.id
            class_name = resource.class_name
            parent = bind_parent_id
            bind_locations = Coordinate.from_point(resource.bind_location)
            liquid_input_slot = liquid_input_slot
            liquid_type = liquid_type
            liquid_volume = liquid_volume
            slot_on_deck = slot_on_deck

        resource = {
            "id": res_id,
            "class": class_name,
            "parent": parent,
            "bind_locations": bind_locations.to_dict(),
            "liquid_input_slot": liquid_input_slot,
            "liquid_type": liquid_type,
            "liquid_volume": liquid_volume,
            "slot_on_deck": slot_on_deck,
        }
        self.temp_protocol["labwares"].append(resource)
        return resource

    def transfer_liquid(
        self,
        sources: Sequence[Container],
        targets: Sequence[Container],
        tip_racks: Sequence[TipRack],
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
            "Solvent": "Well Content",
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

        transfer_params["Solvent"] = "Water"
        TipLocation = tip_racks[0].name
        transfer_params["TipLocation"] = TipLocation

        if len(tip_racks) == 1:
                transfer_params['UseCurrentTips'] = True
        elif len(tip_racks) > 1:
            transfer_params["ChangeTipsBetweenDests"] = True

        self.temp_protocol["steps"].append(transfer_params)

        return 

    def transfer_biomek(
            self,
            source: str,
            target: str,
            tip_rack: str,
            volume: float,
            aspirate_techniques: str,
            dispense_techniques: str,
    ):
        """
        处理Biomek的液体转移操作。

        """
        items = []

        asp_params = self.aspirate_techniques.get(aspirate_techniques, {})
        dis_params = self.dispense_techniques.get(dispense_techniques, {})

        asp_params['Position'] = source
        dis_params['Position'] = target
        asp_params['Volume'] = str(volume)
        dis_params['Volume'] = str(volume)

        items.append(asp_params)
        items.append(dis_params)

        transfer_params = {
            "Span8": False,
            "Pod": "Pod1",
            "items": [],                      
            "Wash": False,
            "Dynamic?": True,
            "AutoSelectActiveWashTechnique": False,
            "ActiveWashTechnique": "",
            "ChangeTipsBetweenDests": True,
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
            "TipLocation": "BC230",
            "UseCurrentTips": False,    
            "UseDisposableTips": False,
            "UseFixedTips": False,
            "UseJIT": True,
            "UseMandrelSelection": True,
            "UseProbes": [True, True, True, True, True, True, True, True],
            "WashCycles": "4",
            "WashVolume": "110%",
            "Wizard": False
        }
        transfer_params["items"] = items
        transfer_params["Solvent"] = 'Water'
        transfer_params["TipLocation"] = tip_rack
        self.temp_protocol["steps"].append(transfer_params)

        return 

    def move_biomek(
        self,
        source: str,
        target: str,
    ):
        """
        处理Biomek移动板子的操作。

        """

        move_params = {
            "Pod": "Pod1",
            "GripSide": "A1 near",
            "Source": source,
            "Target": target,
            "LeaveBottomLabware": False,
        }
        self.temp_protocol["steps"].append(move_params)

        return
    
    def incubation_biomek(
        self,
        time: int,
    ):
        """
        处理Biomek的孵育操作。
        """
        incubation_params = {
            "Message": "Paused",
            "Location": "the whole system",
            "Time": time,
            "Mode": "TimedResource"
        }
        self.temp_protocol["steps"].append(incubation_params)

        return
    
    def oscillation_biomek(
        self,
        rpm: int,
        time: int,
    ):
        """
        处理Biomek的振荡操作。
        """   
        oscillation_params = {
            'Device': 'OrbitalShaker0',
            'Parameters': (str(rpm), '2', str(time), 'CounterClockwise'), 
            'Command': 'Timed Shake'
        }
        self.temp_protocol["steps"].append(oscillation_params)

        return


if __name__ == "__main__":
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
    labware_with_liquid = '''
    [    {
            "id": "stock plate on P1",
            "parent": "deck",
            "slot_on_deck": "P1",
            "class_name": "nest_12_reservoir_15ml",
            "liquid_type": [
                "master_mix"
            ],
            "liquid_volume": [10000],
            "liquid_input_wells": [
                "A1"
            ]
        },
        {
            "id": "Tip Rack BC230 TL2",
            "parent": "deck",
            "slot_on_deck": "TL2",
            "class_name": "BC230",
            "liquid_type": [],
            "liquid_volume": [],
            "liquid_input_wells": [
            ]
        },
            {
                "id": "Tip Rack BC230 TL2",
                "parent": "deck",
                "slot_on_deck": "TL2",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                {
                "id": "Tip Rack BC230 on TL3",
                "parent": "deck",
                "slot_on_deck": "TL3",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                {
                "id": "Tip Rack BC230 on TL4",
                "parent": "deck",
                "slot_on_deck": "TL4",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                {
                "id": "Tip Rack BC230 on TL5",
                "parent": "deck",
                "slot_on_deck": "TL5",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                    {
                "id": "Tip Rack BC230 on P5",
                "parent": "deck",
                "slot_on_deck": "P5",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                        {
                "id": "Tip Rack BC230 on P6",
                "parent": "deck",
                "slot_on_deck": "P6",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                            {
                "id": "Tip Rack BC230 on P7",
                "parent": "deck",
                "slot_on_deck": "P7",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
            {
                "id": "Tip Rack BC230 on P8",
                "parent": "deck",
                "slot_on_deck": "P8",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                {
                "id": "Tip Rack BC230 P16",
                "parent": "deck",
                "slot_on_deck": "P16",
                "class_name": "BC230",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
            {
                "id": "stock plate on 4",
                "parent": "deck",
                "slot_on_deck": "P2",
                "class_name": "nest_12_reservoir_15ml",
                "liquid_type": [
                    "bind beads"
                ],
                "liquid_volume": [10000],
                "liquid_input_wells": [
                    "A1"
                ]
            },
                {
                "id": "stock plate on P2",
                "parent": "deck",
                "slot_on_deck": "P2",
                "class_name": "nest_12_reservoir_15ml",
                "liquid_type": [
                    "bind beads"
                ],
                "liquid_volume": [10000],
                "liquid_input_wells": [
                    "A1"
                ]
            },
                {
                "id": "stock plate on P3",
                "parent": "deck",
                "slot_on_deck": "P3",
                "class_name": "nest_12_reservoir_15ml",
                "liquid_type": [
                    "ethyl alcohol"
                ],
                "liquid_volume": [10000],
                "liquid_input_wells": [
                    "A1"
                ]
            },
        
                {
                "id": "oscillation",
                "parent": "deck",
                "slot_on_deck": "Orbital1",
                "class_name": "Orbital",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                {
                "id": "working plate on P11",
                "parent": "deck",
                "slot_on_deck": "P11",
                "class_name": "NEST 2ml Deep Well Plate",
                "liquid_type": [
                ],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                    {
                "id": "magnetics module on P12",
                "parent": "deck",
                "slot_on_deck": "P12",
                "class_name": "magnetics module",
                "liquid_type": [],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                        {
                "id": "working plate on P13",
                "parent": "deck",
                "slot_on_deck": "P13",
                "class_name": "NEST 2ml Deep Well Plate",
                "liquid_type": [
                ],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            },
                {
                "id": "waste on P22",
                "parent": "deck",
                "slot_on_deck": "P22",
                "class_name": "nest_1_reservoir_195ml",
                "liquid_type": [
                ],
                "liquid_volume": [],
                "liquid_input_wells": [
                ]
            }
        ]
        
        '''
    handler = LiquidHandlerBiomek()

    handler.temp_protocol = {
        "meta": {},
        "labwares": [],
        "steps": []
    }

    input_steps = json.loads(steps_info)
    labwares = json.loads(labware_with_liquid)

    for step in input_steps['steps']:
        operation = step['operation']
        parameters = step['parameters']

        if operation == 'transfer':
            handler.transfer_biomek(source=parameters['source'],
                                    target=parameters['target'],
                                    volume=parameters['volume'],
                                    tip_rack=parameters['tip_rack'],
                                    aspirate_techniques='MC P300 high',
                                    dispense_techniques='MC P300 high')
        elif operation == 'move_labware':
            handler.move_biomek(source=parameters['source'],
                                  target=parameters['target'])
        elif operation == 'oscillation':
            handler.oscillation_biomek(rpm=parameters['rpm'],
                                        time=parameters['time'])
        elif operation == 'incubation':
            handler.incubation_biomek(time=parameters['time'])

    print(json.dumps(handler.temp_protocol, indent=4))


