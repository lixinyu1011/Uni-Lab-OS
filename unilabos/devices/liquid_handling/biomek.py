import requests
from typing import List, Sequence, Optional, Union, Literal

from .liquid_handler_abstract import LiquidHandlerAbstract


class LiquidHandlerBiomek(LiquidHandlerAbstract):
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

    def create_protocol(self,
        protocol_name: str,
        protocol_description: str,
        protocol_version: str,
        protocol_author: str,
        protocol_date: str,
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
        if not self.temp_protocol:
            raise ValueError("No protocol created. Please create a protocol first.")

        # 模拟执行协议
        self._status = "Running"
        self._success = True
        # 在这里可以添加实际执行协议的逻辑

        response = requests.post("localhost:5000/api/protocols", json=self.temp_protocol)

    def create_resource(
        self,
        device_id: str,
        res_id: str,
        class_name: str,
        parent: str,
        bind_locations: Point,
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
        mix_times: Optional[List(int)] = None,
        mix_vol: Optional[int] = None,
        mix_rate: Optional[int] = None,
        mix_liquid_height: Optional[float] = None,
        delays: Optional[List[int]] = None,
        none_keys: List[str] = []
    ):
        # TODO：需要对好接口，下面这个是临时的
        self.temp_protocol["steps"].append({
            "type": "transfer",
            "sources": [source.to_dict() for source in sources],
            "targets": [target.to_dict() for target in targets],
            "tip_racks": [tip_rack.to_dict() for tip_rack in tip_racks],
            "use_channels": use_channels,
            "asp_vols": asp_vols,
            "dis_vols": dis_vols,
            "asp_flow_rates": asp_flow_rates,
            "dis_flow_rates": dis_flow_rates,
            "offsets": offsets,
            "touch_tip": touch_tip,
            "liquid_height": liquid_height,
            "blow_out_air_volume": blow_out_air_volume,
            "spread": spread,
            "is_96_well": is_96_well,
            "mix_stage": mix_stage,
            "mix_times": mix_times,
            "mix_vol": mix_vol,
            "mix_rate": mix_rate,
            "mix_liquid_height": mix_liquid_height,
            "delays": delays,
        })