import asyncio
import collections
import contextlib
import json
import socket
import time
from typing import Any, List, Dict, Optional, Tuple, TypedDict, Union, Sequence, Iterator, Literal

from pylabrobot.liquid_handling import (
    LiquidHandlerBackend,
    Pickup,
    SingleChannelAspiration,
    Drop,
    SingleChannelDispense,
    PickupTipRack,
    DropTipRack,
    MultiHeadAspirationPlate,
    ChatterBoxBackend,
    LiquidHandlerChatterboxBackend,
)
from pylabrobot.liquid_handling.standard import (
    MultiHeadAspirationContainer,
    MultiHeadDispenseContainer,
    MultiHeadDispensePlate,
    ResourcePickup,
    ResourceMove,
    ResourceDrop,
)
from pylabrobot.resources import Tip, Deck, Plate, Well, TipRack, Resource, Container, Coordinate, TipSpot, Trash

from unilabos.devices.liquid_handling.liquid_handler_abstract import LiquidHandlerAbstract


class PRCXIError(RuntimeError):
    """Lilith 返回 Success=false 时抛出的业务异常"""


class Material(TypedDict):  # 和Plate同关系
    uuid: str
    Code: Optional[str]
    Name: Optional[str]
    SummaryName: Optional[str]
    PipetteHeight: Optional[int]
    materialEnum: Optional[int]


class WorkTablets(TypedDict):
    Number: int
    Code: str
    Material: Dict[str, Any]


class MatrixInfo(TypedDict):
    MatrixId: str
    MatrixName: str
    MatrixCount: int
    WorkTablets: list[WorkTablets]


class PRCXI9300Deck(Deck):
    """PRCXI 9300 的专用 Deck 类，继承自 Deck。

    该类定义了 PRCXI 9300 的工作台布局和槽位信息。
    """

    def __init__(self, name: str, size_x: float, size_y: float, size_z: float):
        super().__init__(name, size_x, size_y, size_z)
        self.slots = [None] * 6  # PRCXI 9300 有 6 个槽位


class PRCXI9300Container(Plate, TipRack):
    """PRCXI 9300 的专用 Container 类，继承自 Plate和TipRack。

    该类定义了 PRCXI 9300 的工作台布局和槽位信息。
    """

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        category: str,
        ordering: collections.OrderedDict,
        model: Optional[str] = None,
    ):
        super().__init__(name, size_x, size_y, size_z, category=category, ordering=ordering, model=model)
        self._unilabos_state = {}

    def load_state(self, state: Dict[str, Any]) -> None:
        """从给定的状态加载工作台信息。"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        data = super().serialize_state()
        data.update(self._unilabos_state)
        return data


class PRCXI9300Trash(Trash):
    """PRCXI 9300 的专用 Trash 类，继承自 Trash。

    该类定义了 PRCXI 9300 的工作台布局和槽位信息。
    """

    def __init__(self, name: str, size_x: float, size_y: float, size_z: float, category: str, **kwargs):
        if name != "trash":
            name = "trash"
            print("PRCXI9300Trash name must be 'trash', using 'trash' instead.")
        super().__init__(name, size_x, size_y, size_z, category=category, **kwargs)
        self._unilabos_state = {}

    def load_state(self, state: Dict[str, Any]) -> None:
        """从给定的状态加载工作台信息。"""
        # super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        data = super().serialize_state()
        data.update(self._unilabos_state)
        return data


class PRCXI9300Handler(LiquidHandlerAbstract):
    support_touch_tip = False

    @property
    def reset_ok(self) -> bool:
        """检查设备是否已重置成功。"""
        if self._unilabos_backend.debug:
            return True
        return self._unilabos_backend.is_reset_ok

    def __init__(
        self,
        deck: Deck,
        host: str,
        port: int,
        timeout: float,
        channel_num=8,
        axis="Left",
        setup=True,
        debug=False,
        simulator=False,
        matrix_id="",
        is_9320=False,
    ):
        tablets_info = []
        count = 0
        for child in deck.children:
            if "Material" in child._unilabos_state:
                count += 1
                tablets_info.append(
                    WorkTablets(Number=count, Code=f"T{count}", Material=child._unilabos_state["Material"])
                )
        if is_9320:
            print("当前设备是9320")
        self._unilabos_backend = PRCXI9300Backend(
            tablets_info, host, port, timeout, channel_num, axis, setup, debug, matrix_id, is_9320
        )
        super().__init__(backend=self._unilabos_backend, deck=deck, simulator=simulator, channel_num=channel_num)

    def set_liquid(self, wells: list[Well], liquid_names: list[str], volumes: list[float]):
        return super().set_liquid(wells, liquid_names, volumes)

    def set_group(self, group_name: str, wells: List[Well], volumes: List[float]):
        return super().set_group(group_name, wells, volumes)

    async def transfer_group(self, source_group_name: str, target_group_name: str, unit_volume: float):
        return await super().transfer_group(source_group_name, target_group_name, unit_volume)

    async def create_protocol(
        self,
        protocol_name: str = "",
        protocol_description: str = "",
        protocol_version: str = "",
        protocol_author: str = "",
        protocol_date: str = "",
        protocol_type: str = "",
        none_keys: List[str] = [],
    ):
        self._unilabos_backend.create_protocol(protocol_name)

    async def run_protocol(self):
        return self._unilabos_backend.run_protocol()

    async def remove_liquid(
        self,
        vols: List[float],
        sources: Sequence[Container],
        waste_liquid: Optional[Container] = None,
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Optional[Literal["wide", "tight", "custom"]] = "wide",
        delays: Optional[List[int]] = None,
        is_96_well: Optional[bool] = False,
        top: Optional[List[float]] = None,
        none_keys: List[str] = [],
    ):
        return await super().remove_liquid(
            vols,
            sources,
            waste_liquid,
            use_channels=use_channels,
            flow_rates=flow_rates,
            offsets=offsets,
            liquid_height=liquid_height,
            blow_out_air_volume=blow_out_air_volume,
            spread=spread,
            delays=delays,
            is_96_well=is_96_well,
            top=top,
            none_keys=none_keys,
        )

    async def add_liquid(
        self,
        asp_vols: Union[List[float], float],
        dis_vols: Union[List[float], float],
        reagent_sources: Sequence[Container],
        targets: Sequence[Container],
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Optional[Literal["wide", "tight", "custom"]] = "wide",
        is_96_well: bool = False,
        delays: Optional[List[int]] = None,
        mix_time: Optional[int] = None,
        mix_vol: Optional[int] = None,
        mix_rate: Optional[int] = None,
        mix_liquid_height: Optional[float] = None,
        none_keys: List[str] = [],
    ):
        return await super().add_liquid(
            asp_vols,
            dis_vols,
            reagent_sources,
            targets,
            use_channels=use_channels,
            flow_rates=flow_rates,
            offsets=offsets,
            liquid_height=liquid_height,
            blow_out_air_volume=blow_out_air_volume,
            spread=spread,
            is_96_well=is_96_well,
            delays=delays,
            mix_time=mix_time,
            mix_vol=mix_vol,
            mix_rate=mix_rate,
            mix_liquid_height=mix_liquid_height,
            none_keys=none_keys,
        )

    async def transfer_liquid(
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
        mix_times: Optional[List[int]] = None,
        mix_vol: Optional[int] = None,
        mix_rate: Optional[int] = None,
        mix_liquid_height: Optional[float] = None,
        delays: Optional[List[int]] = None,
        none_keys: List[str] = [],
    ):
        return await super().transfer_liquid(
            sources,
            targets,
            tip_racks,
            use_channels=use_channels,
            asp_vols=asp_vols,
            dis_vols=dis_vols,
            asp_flow_rates=asp_flow_rates,
            dis_flow_rates=dis_flow_rates,
            offsets=offsets,
            touch_tip=touch_tip,
            liquid_height=liquid_height,
            blow_out_air_volume=blow_out_air_volume,
            spread=spread,
            is_96_well=is_96_well,
            mix_stage=mix_stage,
            mix_times=mix_times,
            mix_vol=mix_vol,
            mix_rate=mix_rate,
            mix_liquid_height=mix_liquid_height,
            delays=delays,
            none_keys=none_keys,
        )

    async def custom_delay(self, seconds=0, msg=None):
        return await super().custom_delay(seconds, msg)

    async def touch_tip(self, targets: Sequence[Container]):
        return await super().touch_tip(targets)

    async def mix(
        self,
        targets: Sequence[Container],
        mix_time: int = None,
        mix_vol: Optional[int] = None,
        height_to_bottom: Optional[float] = None,
        offsets: Optional[Coordinate] = None,
        mix_rate: Optional[float] = None,
        none_keys: List[str] = [],
    ):
        return await self._unilabos_backend.mix(
            targets, mix_time, mix_vol, height_to_bottom, offsets, mix_rate, none_keys
        )

    def iter_tips(self, tip_racks: Sequence[TipRack]) -> Iterator[Resource]:
        return super().iter_tips(tip_racks)

    async def pick_up_tips(
        self,
        tip_spots: List[TipSpot],
        use_channels: Optional[List[int]] = None,
        offsets: Optional[List[Coordinate]] = None,
        **backend_kwargs,
    ):
        return await super().pick_up_tips(tip_spots, use_channels, offsets, **backend_kwargs)

    async def aspirate(
        self,
        resources: Sequence[Container],
        vols: List[float],
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Literal["wide", "tight", "custom"] = "wide",
        **backend_kwargs,
    ):

        return await super().aspirate(
            resources,
            vols,
            use_channels,
            flow_rates,
            offsets,
            liquid_height,
            blow_out_air_volume,
            spread,
            **backend_kwargs,
        )

    async def drop_tips(
        self,
        tip_spots: Sequence[Union[TipSpot, Trash]],
        use_channels: Optional[List[int]] = None,
        offsets: Optional[List[Coordinate]] = None,
        allow_nonzero_volume: bool = False,
        **backend_kwargs,
    ):
        return await super().drop_tips(tip_spots, use_channels, offsets, allow_nonzero_volume, **backend_kwargs)

    async def dispense(
        self,
        resources: Sequence[Container],
        vols: List[float],
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Literal["wide", "tight", "custom"] = "wide",
        **backend_kwargs,
    ):
        return await super().dispense(
            resources,
            vols,
            use_channels,
            flow_rates,
            offsets,
            liquid_height,
            blow_out_air_volume,
            spread,
            **backend_kwargs,
        )

    async def discard_tips(
        self,
        use_channels: Optional[List[int]] = None,
        allow_nonzero_volume: bool = True,
        offsets: Optional[List[Coordinate]] = None,
        **backend_kwargs,
    ):
        return await super().discard_tips(use_channels, allow_nonzero_volume, offsets, **backend_kwargs)

    def set_tiprack(self, tip_racks: Sequence[TipRack]):
        super().set_tiprack(tip_racks)

    async def move_to(self, well: Well, dis_to_top: float = 0, channel: int = 0):
        return await super().move_to(well, dis_to_top, channel)


class PRCXI9300Backend(LiquidHandlerBackend):
    """PRCXI 9300 的后端实现，继承自 LiquidHandlerBackend。

    该类提供了与 PRCXI 9300 设备进行通信的基本方法，包括方案管理、自动化控制、运行状态查询等。
    """

    _num_channels = 8  # 默认通道数为 8
    _is_reset_ok = False

    @property
    def is_reset_ok(self) -> bool:
        self._is_reset_ok = self.api_client.get_reset_status()
        return self._is_reset_ok

    matrix_info: MatrixInfo
    protocol_name: str
    steps_todo_list = []

    def __init__(
        self,
        tablets_info: list[WorkTablets],
        host: str = "127.0.0.1",
        port: int = 9999,
        timeout: float = 10.0,
        channel_num: int = 8,
        axis: str = "Left",
        setup=True,
        debug=False,
        matrix_id="",
        is_9320=False,
    ) -> None:
        super().__init__()
        self.tablets_info = tablets_info
        self.matrix_id = matrix_id
        self.api_client = PRCXI9300Api(host, port, timeout, axis, debug, is_9320)
        self.host, self.port, self.timeout = host, port, timeout
        self._num_channels = channel_num
        self._execute_setup = setup
        self.debug = debug

    def create_protocol(self, protocol_name):
        self.protocol_name = protocol_name
        self.steps_todo_list = []

    def run_protocol(self):
        assert self.is_reset_ok, "PRCXI9300Backend is not reset successfully. Please call setup() first."
        run_time = time.time()
        self.matrix_info = MatrixInfo(
            MatrixId=f"{int(run_time)}",
            MatrixName=f"protocol_{run_time}",
            MatrixCount=len(self.tablets_info),
            WorkTablets=self.tablets_info,
        )
        # print(json.dumps(self.matrix_info, indent=2))
        if not len(self.matrix_id):
            res = self.api_client.add_WorkTablet_Matrix(self.matrix_info)
            assert res["Success"], f"Failed to create matrix: {res.get('Message', 'Unknown error')}"
            print(f"PRCXI9300Backend created matrix with ID: {self.matrix_info['MatrixId']}, result: {res}")
            solution_id = self.api_client.add_solution(
                f"protocol_{run_time}", self.matrix_info["MatrixId"], self.steps_todo_list
            )
        else:
            print(f"PRCXI9300Backend using predefined worktable {self.matrix_id}, skipping matrix creation.")
            solution_id = self.api_client.add_solution(f"protocol_{run_time}", self.matrix_id, self.steps_todo_list)
        print(f"PRCXI9300Backend created solution with ID: {solution_id}")
        self.api_client.load_solution(solution_id)
        print(json.dumps(self.steps_todo_list, indent=2))
        return self.api_client.start()

    @classmethod
    def check_channels(cls, use_channels: List[int]) -> List[int]:
        """检查通道是否符合要求，PRCXI9300Backend 只支持所有 8 个通道。"""
        if use_channels != [0, 1, 2, 3, 4, 5, 6, 7]:
            print("PRCXI9300Backend only supports all 8 channels, using default [0, 1, 2, 3, 4, 5, 6, 7].")
            return [0, 1, 2, 3, 4, 5, 6, 7]
        return use_channels

    async def setup(self):
        await super().setup()
        try:
            if self._execute_setup:
                self.api_client.call("IAutomation", "Reset")
                while not self.is_reset_ok:
                    print("Waiting for PRCXI9300 to reset...")
                    await asyncio.sleep(1)
                print("PRCXI9300 reset successfully.")
        except ConnectionRefusedError as e:
            raise RuntimeError(
                f"Failed to connect to PRCXI9300 API at {self.host}:{self.port}. "
                "Please ensure the PRCXI9300 service is running."
            ) from e

    async def stop(self):
        self.api_client.call("IAutomation", "Stop")

    async def pick_up_tips(self, ops: List[Pickup], use_channels: List[int] = None):
        """Pick up tips from the specified resource."""

        plate_indexes = []
        for op in ops:
            plate = op.resource.parent
            deck = plate.parent
            plate_index = deck.children.index(plate)
            # print(f"Plate index: {plate_index}, Plate name: {plate.name}")
            # print(f"Number of children in deck: {len(deck.children)}")

            plate_indexes.append(plate_index)

        if len(set(plate_indexes)) != 1:
            raise ValueError("All pickups must be from the same plate. Found different plates: " + str(plate_indexes))

        tip_columns = []
        for op in ops:
            tipspot = op.resource
            tipspot_index = tipspot.parent.children.index(tipspot)
            tip_columns.append(tipspot_index // 8)
        if len(set(tip_columns)) != 1:
            raise ValueError("All pickups must be from the same tip column. Found different columns: " + str(tip_columns))
        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1
        hole_row = 1
        if self._num_channels == 1:
            hole_row = tipspot_index % 8 + 1

        step = self.api_client.Load(
            dosage=0,
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=hole_row,
            hole_col=hole_col,
            blending_times=0,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)

    async def drop_tips(self, ops: List[Drop], use_channels: List[int] = None):
        """Pick up tips from the specified resource."""

        # 检查trash #
        if ops[0].resource.name == "trash":

            PlateNo = ops[0].resource.parent.children.index(ops[0].resource) + 1

            step = self.api_client.UnLoad(
                dosage=0,
                plate_no=PlateNo,
                is_whole_plate=False,
                hole_row=1,
                hole_col=3,
                blending_times=0,
                balance_height=0,
                plate_or_hole=f"H{1}-8,T{PlateNo}",
                hole_numbers="1,2,3,4,5,6,7,8",
            )
            self.steps_todo_list.append(step)
            return
        # print(ops[0].resource.parent.children.index(ops[0].resource))

        plate_indexes = []
        for op in ops:
            plate = op.resource.parent
            deck = plate.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)
        if len(set(plate_indexes)) != 1:
            raise ValueError(
                "All drop_tips must be from the same plate. Found different plates: " + str(plate_indexes)
            )

        tip_columns = []
        for op in ops:
            tipspot = op.resource
            tipspot_index = tipspot.parent.children.index(tipspot)
            tip_columns.append(tipspot_index // 8)
        if len(set(tip_columns)) != 1:
            raise ValueError(
                "All drop_tips must be from the same tip column. Found different columns: " + str(tip_columns)
            )

        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1

        if self.channel_num == 1:
            hole_row = tipspot_index % 8 + 1

        step = self.api_client.UnLoad(
            dosage=0,
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=hole_row,
            hole_col=hole_col,
            blending_times=0,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)

    async def mix(
        self,
        targets: Sequence[Container],
        mix_time: int = None,
        mix_vol: Optional[int] = None,
        height_to_bottom: Optional[float] = None,
        offsets: Optional[Coordinate] = None,
        mix_rate: Optional[float] = None,
        none_keys: List[str] = [],
    ):
        """Mix liquid in the specified resources."""

        plate_indexes = []
        for op in targets:
            deck = op.parent.parent
            plate = op.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)

        if len(set(plate_indexes)) != 1:
            raise ValueError("All pickups must be from the same plate. Found different plates: " + str(plate_indexes))

        tip_columns = []
        for op in targets:
            tipspot_index = op.parent.children.index(op)
            tip_columns.append(tipspot_index // 8)

        if len(set(tip_columns)) != 1:
            raise ValueError(
                "All pickups must be from the same tip column. Found different columns: " + str(tip_columns)
            )

        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1
        hole_row = 1
        if self.num_channels == 1:
            hole_row = tipspot_index % 8 + 1

        assert mix_time > 0
        step = self.api_client.Blending(
            dosage=mix_vol,
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=hole_row,
            hole_col=hole_col,
            blending_times=mix_time,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)

    async def aspirate(self, ops: List[SingleChannelAspiration], use_channels: List[int] = None):
        """Aspirate liquid from the specified resources."""

        plate_indexes = []
        for op in ops:
            plate = op.resource.parent
            deck = plate.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)

        if len(set(plate_indexes)) != 1:
            raise ValueError("All pickups must be from the same plate. Found different plates: " + str(plate_indexes))

        tip_columns = []
        for op in ops:
            tipspot = op.resource
            tipspot_index = tipspot.parent.children.index(tipspot)
            tip_columns.append(tipspot_index // 8)

        if len(set(tip_columns)) != 1:
            raise ValueError(
                "All pickups must be from the same tip column. Found different columns: " + str(tip_columns)
            )

        volumes = [op.volume for op in ops]
        if len(set(volumes)) != 1:
            raise ValueError("All aspirate volumes must be the same. Found different volumes: " + str(volumes))

        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1
        hole_row = 1
        if self.num_channels == 1:
            hole_row = tipspot_index % 8 + 1

        step = self.api_client.Imbibing(
            dosage=int(volumes[0]),
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=hole_row,
            hole_col=hole_col,
            blending_times=0,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)

    async def dispense(self, ops: List[SingleChannelDispense], use_channels: List[int] = None):
        """Dispense liquid into the specified resources."""

        plate_indexes = []
        for op in ops:
            plate = op.resource.parent
            deck = plate.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)

        if len(set(plate_indexes)) != 1:
            raise ValueError("All dispense must be from the same plate. Found different plates: " + str(plate_indexes))

        tip_columns = []
        for op in ops:
            tipspot = op.resource
            tipspot_index = tipspot.parent.children.index(tipspot)
            tip_columns.append(tipspot_index // 8)

        if len(set(tip_columns)) != 1:
            raise ValueError(
                "All dispense must be from the same tip column. Found different columns: " + str(tip_columns)
            )

        volumes = [op.volume for op in ops]
        if len(set(volumes)) != 1:
            raise ValueError("All dispense volumes must be the same. Found different volumes: " + str(volumes))

        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1

        hole_row = 1
        if self.num_channels == 1:
            hole_row = tipspot_index % 8 + 1

        step = self.api_client.Tapping(
            dosage=int(volumes[0]),
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=hole_row,
            hole_col=hole_col,
            blending_times=0,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)

    async def pick_up_tips96(self, pickup: PickupTipRack):
        raise NotImplementedError("The PRCXI backend does not support the 96 head.")

    async def drop_tips96(self, drop: DropTipRack):
        raise NotImplementedError("The PRCXI backend does not support the 96 head.")

    async def aspirate96(self, aspiration: Union[MultiHeadAspirationPlate, MultiHeadAspirationContainer]):
        raise NotImplementedError("The Opentrons backend does not support the 96 head.")

    async def dispense96(self, dispense: Union[MultiHeadDispensePlate, MultiHeadDispenseContainer]):
        raise NotImplementedError("The Opentrons backend does not support the 96 head.")

    async def pick_up_resource(self, pickup: ResourcePickup):
        raise NotImplementedError("The Opentrons backend does not support the robotic arm.")

    async def move_picked_up_resource(self, move: ResourceMove):
        raise NotImplementedError("The Opentrons backend does not support the robotic arm.")

    async def drop_resource(self, drop: ResourceDrop):
        raise NotImplementedError("The Opentrons backend does not support the robotic arm.")

    def can_pick_up_tip(self, channel_idx: int, tip: Tip) -> bool:
        return True  # PRCXI9300Backend does not have tip compatibility issues

    def serialize(self) -> dict:
        raise NotImplementedError()

    @property
    def num_channels(self) -> int:
        return self._num_channels


class PRCXI9300Api:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9999,
        timeout: float = 10.0,
        axis="Left",
        debug: bool = False,
        is_9320: bool = False,
    ) -> None:
        self.host, self.port, self.timeout = host, port, timeout
        self.debug = debug
        self.axis = axis
        self.is_9320 = is_9320

    @staticmethod
    def _len_prefix(n: int) -> bytes:
        return bytes.fromhex(format(n, "016x"))

    def _raw_request(self, payload: str) -> str:
        if self.debug:
            return " "
        with contextlib.closing(socket.socket()) as sock:
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.port))
            data = payload.encode()
            sock.sendall(self._len_prefix(len(data)) + data)

            chunks, first = [], True
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                if first:
                    chunk, first = chunk[8:], False
                chunks.append(chunk)
            return b"".join(chunks).decode()

    # ---------------------------------------------------- 方案相关（ISolution）
    def list_solutions(self) -> List[Dict[str, Any]]:
        """GetSolutionList"""
        return self.call("ISolution", "GetSolutionList")

    def load_solution(self, solution_id: str) -> bool:
        """LoadSolution"""
        return self.call("ISolution", "LoadSolution", [solution_id])

    def add_solution(self, name: str, matrix_id: str, steps: List[Dict[str, Any]]) -> str:
        """AddSolution → 返回新方案 GUID"""
        return self.call("ISolution", "AddSolution", [name, matrix_id, steps])

    # ---------------------------------------------------- 自动化控制（IAutomation）
    def start(self) -> bool:
        return self.call("IAutomation", "Start")

    def call(self, service: str, method: str, params: Optional[list] = None) -> Any:
        payload = json.dumps(
            {"ServiceName": service, "MethodName": method, "Paramters": params or []}, separators=(",", ":")
        )
        resp = json.loads(self._raw_request(payload))
        if not resp.get("Success", False):
            raise PRCXIError(resp.get("Msg", "Unknown error"))
        data = resp.get("Data")
        try:
            return json.loads(data)
        except (TypeError, json.JSONDecodeError):
            return data

    def pause(self) -> bool:
        """Pause"""
        return self.call("IAutomation", "Pause")

    def resume(self) -> bool:
        """Resume"""
        return self.call("IAutomation", "Resume")

    def get_error_code(self) -> Optional[str]:
        """GetErrorCode"""
        return self.call("IAutomation", "GetErrorCode")

    def get_reset_status(self) -> bool:
        """GetErrorCode"""
        if self.debug:
            return True
        res = self.call("IAutomation", "GetResetStatus")
        return not res

    def clear_error_code(self) -> bool:
        """RemoveErrorCodet"""
        return self.call("IAutomation", "RemoveErrorCodet")

    # ---------------------------------------------------- 运行状态（IMachineState）
    def step_state_list(self) -> List[Dict[str, Any]]:
        """GetStepStateList"""
        return self.call("IMachineState", "GetStepStateList")

    def step_status(self, seq_num: int) -> Dict[str, Any]:
        """GetStepStatus"""
        return self.call("IMachineState", "GetStepStatus", [seq_num])

    def step_state(self, seq_num: int) -> Dict[str, Any]:
        """GetStepState"""
        return self.call("IMachineState", "GetStepState", [seq_num])

    def axis_location(self, axis_num: int = 1) -> Dict[str, Any]:
        """GetLocation"""
        return self.call("IMachineState", "GetLocation", [axis_num])

    # ---------------------------------------------------- 版位矩阵（IMatrix）
    def get_all_materials(self) -> Dict[str, Any]:
        """GetStepState"""
        return self.call("IMatrix", "GetAllMaterial", [])

    def list_matrices(self) -> List[Dict[str, Any]]:
        """GetWorkTabletMatrices"""
        return self.call("IMatrix", "GetWorkTabletMatrices")

    def matrix_by_id(self, matrix_id: str) -> Dict[str, Any]:
        """GetWorkTabletMatrixById"""
        return self.call("IMatrix", "GetWorkTabletMatrixById", [matrix_id])

    def add_WorkTablet_Matrix(self, matrix: MatrixInfo):
        return self.call("IMatrix", "AddWorkTabletMatrix2" if self.is_9320 else "AddWorkTabletMatrix", [matrix])

    def Load(
        self,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": self.axis,
            "Function": "Load",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

    def Imbibing(
        self,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": self.axis,
            "Function": "Imbibing",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

    def Tapping(
        self,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": self.axis,
            "Function": "Tapping",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

    def Blending(
        self,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": self.axis,
            "Function": "Blending",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

    def UnLoad(
        self,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": self.axis,
            "Function": "UnLoad",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

class DefaultLayout:

    def __init__(self, product_name: str = "PRCXI9300"):
        self.labresource = {}
        if product_name not in ["PRCXI9300", "PRCXI9320"]:
            raise ValueError(f"Unsupported product_name: {product_name}. Only 'PRCXI9300' and 'PRCXI9320' are supported.")

        if product_name == "PRCXI9300":
            self.rows = 2
            self.columns = 3
            self.layout = [1, 2, 3, 4, 5, 6]
            self.trash_slot = 3
            self.waste_liquid_slot = 6

        elif product_name == "PRCXI9320":
            self.rows = 3
            self.columns = 4
            self.layout = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
            self.trash_slot = 16
            self.waste_liquid_slot = 12
            self.default_layout =  {"MatrixId":f"{time.time()}","MatrixName":f"{time.time()}","MatrixCount":16,"WorkTablets":
            [{"Number": 1, "Code": "T1", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 2, "Code": "T2", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 3, "Code": "T3", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 4, "Code": "T4", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 5, "Code": "T5", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 6, "Code": "T6", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 7, "Code": "T7", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 8, "Code": "T8", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 9, "Code": "T9", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 10, "Code": "T10", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 11, "Code": "T11", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 12, "Code": "T12", "Material": {"uuid": "730067cf07ae43849ddf4034299030e9", "materialEnum": 0}},  # 这个设置成废液槽，用储液槽表示
  {"Number": 13, "Code": "T13", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 14, "Code": "T14", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 15, "Code": "T15", "Material": {"uuid": "57b1e4711e9e4a32b529f3132fc5931f", "materialEnum": 0}},
  {"Number": 16, "Code": "T16", "Material": {"uuid": "730067cf07ae43849ddf4034299030e9", "materialEnum": 0}} # 这个设置成垃圾桶，用储液槽表示
]
}

    def get_layout(self) -> Dict[str, Any]:
        return {
            "rows": self.rows,
            "columns": self.columns,
            "layout": self.layout,
            "trash_slot": self.trash_slot,
            "waste_liquid_slot": self.waste_liquid_slot
        }

    def get_trash_slot(self) -> int:
        return self.trash_slot

    def get_waste_liquid_slot(self) -> int:
        return self.waste_liquid_slot

    def add_lab_resource(self, material_info):
        self.labresource = material_info

    def recommend_layout(self, needs: List[Tuple[str, str, int]]) -> Dict[str, Any]:
        layout_list = []
        for reagent_name, material_name, count in needs:

            if material_name not in self.labresource:
                raise ValueError(f"Material {reagent_name} not found in lab resources.")

            # 预留位置12和16不动
        reserved_positions = {12, 16}
        available_positions = [i for i in range(1, 17) if i not in reserved_positions]

            # 计算总需求
        total_needed = sum(count for _, _, count in needs)
        if total_needed > len(available_positions):
            raise ValueError(f"需要 {total_needed} 个位置，但只有 {len(available_positions)} 个可用位置（排除位置12和16）")

            # 依次分配位置
        current_pos = 0
        for reagent_name, material_name, count in needs:

            material_uuid = self.labresource[material_name]['uuid']
            material_enum = self.labresource[material_name]['materialEnum']

            for _ in range(count):
                if current_pos >= len(available_positions):
                    raise ValueError("位置不足，无法分配更多物料")

                position = available_positions[current_pos]
                # 找到对应的tablet并更新
                for tablet in self.default_layout['WorkTablets']:
                    if tablet['Number'] == position:
                        tablet['Material']['uuid'] = material_uuid
                        tablet['Material']['materialEnum'] = material_enum
                        layout_list.append(dict(reagent_name=reagent_name, material_name=material_name, positions=position))
                        break
                current_pos += 1
        return self.default_layout, layout_list



if __name__ == "__main__":
    # Example usage
    # 1. 用导出的json，给每个T1 T2板子设定相应的物料，如果是孔板和枪头盒，要对应区分
    # 2. backend需要支持num channel为1的情况
    # 3. 设计一个单点动作流程，可以跑
    # 4.

    # deck = PRCXI9300Deck(name="PRCXI_Deck_9300", size_x=100, size_y=100, size_z=100)

    # from pylabrobot.resources.opentrons.tip_racks import opentrons_96_tiprack_300ul,opentrons_96_tiprack_10ul
    # from pylabrobot.resources.opentrons.plates import corning_96_wellplate_360ul_flat, nest_96_wellplate_2ml_deep

    # def get_well_container(name: str) -> PRCXI9300Container:
    #     well_containers = corning_96_wellplate_360ul_flat(name).serialize()
    #     plate = PRCXI9300Container(name=name, size_x=50, size_y=50, size_z=10, category="plate",
    #                        ordering=well_containers["ordering"])
    #     plate_serialized = plate.serialize()
    #     plate_serialized["parent_name"] = deck.name
    #     well_containers.update({k: v for k, v in plate_serialized.items() if k not in ["children"]})
    #     new_plate: PRCXI9300Container = PRCXI9300Container.deserialize(well_containers)
    #     return new_plate

    # def get_tip_rack(name: str) -> PRCXI9300Container:
    #     tip_racks = opentrons_96_tiprack_300ul("name").serialize()
    #     tip_rack = PRCXI9300Container(name=name, size_x=50, size_y=50, size_z=10, category="tip_rack",
    #                        ordering=tip_racks["ordering"])
    #     tip_rack_serialized = tip_rack.serialize()
    #     tip_rack_serialized["parent_name"] = deck.name
    #     tip_racks.update({k: v for k, v in tip_rack_serialized.items() if k not in ["children"]})
    #     new_tip_rack: PRCXI9300Container = PRCXI9300Container.deserialize(tip_racks)
    #     return new_tip_rack

    # plate1 = get_tip_rack("RackT1")
    # plate1.load_state({
    #     "Material": {
    #         "uuid": "076250742950465b9d6ea29a225dfb00",
    #         "Code": "ZX-001-300",
    #         "Name": "300μL Tip头"
    #     }
    # })

    # plate2 = get_well_container("PlateT2")
    # plate2.load_state({
    #     "Material": {
    #         "uuid": "57b1e4711e9e4a32b529f3132fc5931f",
    #         "Code": "ZX-019-2.2",
    #         "Name": "96深孔板"
    #     }
    # })

    # plate3 = PRCXI9300Trash("trash", size_x=50, size_y=100, size_z=10, category="trash")
    # plate3.load_state({
    #     "Material": {
    #         "uuid": "730067cf07ae43849ddf4034299030e9"
    #     }
    # })

    # plate4 = get_well_container("PlateT4")
    # plate4.load_state({
    #     "Material": {
    #         "uuid": "57b1e4711e9e4a32b529f3132fc5931f",
    #         "Code": "ZX-019-2.2",
    #         "Name": "96深孔板"
    #     }
    # })

    # plate5 = get_well_container("PlateT5")
    # plate5.load_state({
    #     "Material": {
    #         "uuid": "57b1e4711e9e4a32b529f3132fc5931f",
    #         "Code": "ZX-019-2.2",
    #         "Name": "96深孔板"
    #     }
    # })
    # plate6 = get_well_container("PlateT6")

    # plate6.load_state({
    #     "Material": {
    #         "uuid": "57b1e4711e9e4a32b529f3132fc5931f",
    #         "Code": "ZX-019-2.2",
    #         "Name": "96深孔板"
    #     }
    # })

    # deck.assign_child_resource(plate1, location=Coordinate(0, 0, 0))
    # deck.assign_child_resource(plate2, location=Coordinate(0, 0, 0))
    # deck.assign_child_resource(plate3, location=Coordinate(0, 0, 0))
    # deck.assign_child_resource(plate4, location=Coordinate(0, 0, 0))
    # deck.assign_child_resource(plate5, location=Coordinate(0, 0, 0))
    # deck.assign_child_resource(plate6, location=Coordinate(0, 0, 0))

    # # # plate_2_liquids = [[('water', 500)]]*96

    # # # plate2.set_well_liquids(plate_2_liquids)




    # handler = PRCXI9300Handler(deck=deck, host="10.181.214.132", port=9999, 
    #                            timeout=10.0, setup=False, debug=False,
    #                            simulator=True,
    #                            matrix_id="71593",
    #                            channel_num=8, axis="Left")  # Initialize the handler with the deck and host settings

    # plate_2_liquids = handler.set_group("water", plate2.children[:8], [200]*8)

    # plate5_liquids = handler.set_group("master_mix", plate5.children[:8], [100]*8)

    # handler.set_tiprack([plate1])
    # asyncio.run(handler.setup())  # Initialize the handler and setup the connection
    # from pylabrobot.resources import set_volume_tracking
    # from pylabrobot.resources import set_tip_tracking
    # set_volume_tracking(enabled=True)
    # from unilabos.resources.graphio import *
    # # A = tree_to_list([resource_plr_to_ulab(deck)])
    # # with open("deck_9300_new.json", "w", encoding="utf-8") as f:
    # #     json.dump(A, f, indent=4, ensure_ascii=False)
    # asyncio.run(handler.create_protocol(protocol_name="Test Protocol"))  # Initialize the backend and setup the connection
    # asyncio.run(handler.transfer_group("water", "master_mix", 100))  # Reset tip tracking

    # asyncio.run(handler.pick_up_tips(plate1.children[:8],[0,1,2,3,4,5,6,7]))
    # print(plate1.children[:8])
    # asyncio.run(handler.aspirate(plate2.children[:8],[50]*8, [0,1,2,3,4,5,6,7]))
    # print(plate2.children[:8])
    # asyncio.run(handler.dispense(plate5.children[:8],[50]*8,[0,1,2,3,4,5,6,7]))
    # print(plate5.children[:8])

    # #asyncio.run(handler.drop_tips(tip_rack.children[8:16],[0,1,2,3,4,5,6,7]))
    # asyncio.run(handler.discard_tips([0,1,2,3,4,5,6,7]))

    # asyncio.run(handler.mix(well_containers.children[:8
    # ], mix_time=3, mix_vol=50, height_to_bottom=0.5, offsets=Coordinate(0, 0, 0), mix_rate=100))
    # #print(json.dumps(handler._unilabos_backend.steps_todo_list, indent=2))  # Print matrix info
    # asyncio.run(handler.add_liquid(
    #     asp_vols=[100]*16,
    #     dis_vols=[100]*16,
    #     reagent_sources=plate2.children[:16],
    #     targets=plate5.children[:16],
    #     use_channels=[0, 1, 2, 3, 4, 5, 6, 7],
    #     flow_rates=[None] * 32,
    #     offsets=[Coordinate(0, 0, 0)] * 32,
    #     liquid_height=[None] * 16,
    #     blow_out_air_volume=[None] * 16,
    #     delays=None,
    #     mix_time=3,
    #     mix_vol=50,
    #     spread="wide",
    # ))
    # asyncio.run(handler.run_protocol())  # Run the protocol
    # asyncio.run(handler.remove_liquid(
    #     vols=[100]*16,
    #     sources=plate2.children[-16:],
    #     waste_liquid=plate5.children[:16], # 这个有些奇怪，但是好像也只能这么写
    #     use_channels=[0, 1, 2, 3, 4, 5, 6, 7],
    #     flow_rates=[None] * 32,
    #     offsets=[Coordinate(0, 0, 0)] * 32,
    #     liquid_height=[None] * 32,
    #     blow_out_air_volume=[None] * 32,
    #     spread="wide",
    # ))

    # acid = [20]*8+[40]*8+[60]*8+[80]*8+[100]*8+[120]*8+[140]*8+[160]*8+[180]*8+[200]*8+[220]*8+[240]*8
    # alkaline = acid[::-1]  # Reverse the acid list for alkaline
    # asyncio.run(handler.transfer_liquid(
    #     asp_vols=acid,
    #     dis_vols=acid,
    #     tip_racks=[plate1],
    #     sources=plate2.children[:],
    #     targets=plate5.children[:],
    #     use_channels=[0, 1, 2, 3, 4, 5, 6, 7],
    #     offsets=[Coordinate(0, 0, 0)] * 32,
    #     asp_flow_rates=[None] * 16,
    #     dis_flow_rates=[None] * 16,
    #     liquid_height=[None] * 32,
    #     blow_out_air_volume=[None] * 32,
    #     mix_times=3,
    #     mix_vol=50,
    #     spread="wide",
    # ))
    # asyncio.run(handler.run_protocol())  # Run the protocol
    # # input("Running protocol...")
    # # input("Press Enter to continue...")  # Wait for user input before proceeding
    # # print("PRCXI9300Handler initialized with deck and host settings.")



### 9320 ###


    deck = PRCXI9300Deck(name="PRCXI_Deck", size_x=100, size_y=100, size_z=100)

    from pylabrobot.resources.opentrons.tip_racks import tipone_96_tiprack_200ul, opentrons_96_tiprack_10ul
    from pylabrobot.resources.opentrons.plates import corning_96_wellplate_360ul_flat, nest_96_wellplate_2ml_deep

    def get_well_container(name: str) -> PRCXI9300Container:
        well_containers = corning_96_wellplate_360ul_flat(name).serialize()
        plate = PRCXI9300Container(
            name=name, size_x=50, size_y=50, size_z=10, category="plate", ordering=well_containers["ordering"]
        )
        plate_serialized = plate.serialize()
        plate_serialized["parent_name"] = deck.name
        well_containers.update({k: v for k, v in plate_serialized.items() if k not in ["children"]})
        new_plate: PRCXI9300Container = PRCXI9300Container.deserialize(well_containers)
        return new_plate

    def get_tip_rack(name: str, child_prefix: str="tip") -> PRCXI9300Container:
        tip_racks = opentrons_96_tiprack_10ul(name).serialize()
        tip_rack = PRCXI9300Container(
            name=name, size_x=50, size_y=50, size_z=10, category="tip_rack", ordering=collections.OrderedDict({
                k: f"{child_prefix}_{k}" for k, v in tip_racks["ordering"].items()
            })
        )
        tip_rack_serialized = tip_rack.serialize()
        tip_rack_serialized["parent_name"] = deck.name
        tip_racks.update({k: v for k, v in tip_rack_serialized.items() if k not in ["children"]})
        new_tip_rack: PRCXI9300Container = PRCXI9300Container.deserialize(tip_racks)
        return new_tip_rack

    plate1 = get_tip_rack("RackT1")
    plate1.load_state(
        {"Material": {"uuid": "068b3815e36b4a72a59bae017011b29f", "Code": "ZX-001-10+", "Name": "10μL加长 Tip头"}}
    )
    plate2 = get_well_container("PlateT2")
    plate2.load_state(
        {"Material": {"uuid": "b05b3b2aafd94ec38ea0cd3215ecea8f", "Code": "ZX-78-096", "Name": "细菌培养皿"}}
    )
    plate3 = get_well_container("PlateT3")
    plate3.load_state(
        {
            "Material": {
                "uuid": "04211a2dc93547fe9bf6121eac533650",
            }
        }
    )
    plate4 = get_well_container("PlateT4")
    plate4.load_state(
        {"Material": {"uuid": "b05b3b2aafd94ec38ea0cd3215ecea8f", "Code": "ZX-78-096", "Name": "细菌培养皿"}}
    )

    plate5 = get_tip_rack("RackT5")
    plate5.load_state(
        {
            "Material": {
                "uuid": "076250742950465b9d6ea29a225dfb00",
                "Code": "ZX-001-300",
                "SupplyType": 1,
                "Name": "300μL Tip头",
            }
        }
    )
    plate6 = get_well_container("PlateT6")
    plate6.load_state(
        {
            "Material": {
                "uuid": "e146697c395e4eabb3d6b74f0dd6aaf7",
                "Code": "1",
                "SupplyType": 1,
                "Name": "ep适配器",
                "SummaryName": "ep适配器",
            }
        }
    )
    plate7 = PRCXI9300Container(
        name="plateT7", size_x=50, size_y=50, size_z=10, category="plate", ordering=collections.OrderedDict()
    )
    plate7.load_state({"Material": {"uuid": "04211a2dc93547fe9bf6121eac533650"}})
    plate8 = get_tip_rack("PlateT8")
    plate8.load_state({"Material": {"uuid": "04211a2dc93547fe9bf6121eac533650"}})
    plate9 = get_well_container("PlateT9")
    plate9.load_state(
        {
            "Material": {
                "uuid": "4a043a07c65a4f9bb97745e1f129b165",
                "Code": "ZX-58-0001",
                "SupplyType": 2,
                "Name": "全裙边 PCR适配器",
                "SummaryName": "全裙边 PCR适配器",
            }
        }
    )
    plate10 = get_well_container("PlateT10")
    plate10.load_state(
        {
            "Material": {
                "uuid": "4a043a07c65a4f9bb97745e1f129b165",
                "Code": "ZX-58-0001",
                "SupplyType": 2,
                "Name": "全裙边 PCR适配器",
                "SummaryName": "全裙边 PCR适配器",
            }
        }
    )
    plate11 = get_well_container("PlateT11")
    plate11.load_state(
        {
            "Material": {
                "uuid": "04211a2dc93547fe9bf6121eac533650",
            }
        }
    )
    plate12 = get_well_container("PlateT12")
    plate12.load_state({"Material": {"uuid": "04211a2dc93547fe9bf6121eac533650"}})
    plate13 = get_well_container("PlateT13")
    plate13.load_state(
        {
            "Material": {
                "uuid": "4a043a07c65a4f9bb97745e1f129b165",
                "Code": "ZX-58-0001",
                "SupplyType": 2,
                "Name": "全裙边 PCR适配器",
                "SummaryName": "全裙边 PCR适配器",
            }
        }
    ),
    plate14 = get_well_container("PlateT14")
    plate14.load_state(
        {
            "Material": {
                "uuid": "4a043a07c65a4f9bb97745e1f129b165",
                "Code": "ZX-58-0001",
                "SupplyType": 2,
                "Name": "全裙边 PCR适配器",
                "SummaryName": "全裙边 PCR适配器",
            }
        }
    ),
    plate15 = get_well_container("PlateT15")
    plate15.load_state({"Material": {"uuid": "04211a2dc93547fe9bf6121eac533650"}})

    trash = PRCXI9300Trash(name="trash", size_x=50, size_y=50, size_z=10, category="trash")
    trash.load_state({"Material": {"uuid": "730067cf07ae43849ddf4034299030e9"}})

    # container_for_nothing = PRCXI9300Container(name="container_for_nothing", size_x=50, size_y=50, size_z=10, category="plate", ordering=collections.OrderedDict())

    deck.assign_child_resource(plate1, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate2, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(
        PRCXI9300Container(
            name="container_for_nothin3",
            size_x=50,
            size_y=50,
            size_z=10,
            category="plate",
            ordering=collections.OrderedDict(),
        ),
        location=Coordinate(0, 0, 0),
    )
    deck.assign_child_resource(plate4, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate5, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate6, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(
        PRCXI9300Container(
            name="container_for_nothing7",
            size_x=50,
            size_y=50,
            size_z=10,
            category="plate",
            ordering=collections.OrderedDict(),
        ),
        location=Coordinate(0, 0, 0),
    )
    deck.assign_child_resource(
        PRCXI9300Container(
            name="container_for_nothing8",
            size_x=50,
            size_y=50,
            size_z=10,
            category="plate",
            ordering=collections.OrderedDict(),
        ),
        location=Coordinate(0, 0, 0),
    )
    deck.assign_child_resource(plate9, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate10, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(
        PRCXI9300Container(
            name="container_for_nothing11",
            size_x=50,
            size_y=50,
            size_z=10,
            category="plate",
            ordering=collections.OrderedDict(),
        ),
        location=Coordinate(0, 0, 0),
    )
    deck.assign_child_resource(
        PRCXI9300Container(
            name="container_for_nothing12",
            size_x=50,
            size_y=50,
            size_z=10,
            category="plate",
            ordering=collections.OrderedDict(),
        ),
        location=Coordinate(0, 0, 0),
    )
    deck.assign_child_resource(plate13, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate14, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate15, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(trash, location=Coordinate(0, 0, 0))

    from unilabos.resources.graphio import tree_to_list, resource_plr_to_ulab

    A = tree_to_list([resource_plr_to_ulab(deck)])
    with open("deck.json", "w", encoding="utf-8") as f:
        json.dump(A, f, indent=4, ensure_ascii=False)

    handler = PRCXI9300Handler(
        deck=deck,
        host="192.168.0.121",
        port=9999,
        timeout=10.0,
        setup=False,
        debug=False,
        matrix_id="5de524d0-3f95-406c-86dd-f83626ebc7cb",
        channel_num=1,
        axis="Left",
        simulator=False,
        is_9320=True,
    )  # Initialize the handler with the deck and host settings
    backend: PRCXI9300Backend = handler.backend
    res = backend.api_client.get_all_materials()
    handler.set_tiprack([plate8])  # Set the tip rack for the handler
    asyncio.run(handler.setup())  # Initialize the handler and setup the connection
    from pylabrobot.resources import set_volume_tracking

    # from pylabrobot.resources import set_tip_tracking
    set_volume_tracking(enabled=True)


# 第一种情景：一个孔往多个孔加液
    # plate_2_liquids = handler.set_group("water", [plate2.children[0]], [300])
    # plate5_liquids = handler.set_group("master_mix", plate5.children[:23], [100]*23)
# 第二个情景：多个孔往多个孔加液(但是个数得对应)
    plate_2_liquids = handler.set_group("water", plate2.children[:23], [300]*23)
    plate5_liquids = handler.set_group("master_mix", plate5.children[:23], [100]*23)

    # plate11.set_well_liquids([("Water", 100) if (i % 8 == 0 and i // 8 < 6) else (None, 100) for i in range(96)])  # Set liquids for every 8 wells in plate8

    # plate11.set_well_liquids([("Water", 100) if (i % 8 == 0 and i // 8 < 6) else (None, 100) for i in range(96)])  # Set liquids for every 8 wells in plate8

#     A = tree_to_list([resource_plr_to_ulab(deck)])
#     # with open("deck.json", "w", encoding="utf-8") as f:
#     #     json.dump(A, f, indent=4, ensure_ascii=False)

#     print(plate11.get_well(0).tracker.get_used_volume())
    asyncio.run(handler.create_protocol(protocol_name="Test Protocol"))  # Initialize the backend and setup the connection
    asyncio.run(handler.transfer_group("water", "master_mix", 10))  # Reset tip tracking


    # asyncio.run(handler.pick_up_tips([plate8.children[8]],[0]))
    # print(plate8.children[8])
    # asyncio.run(handler.run_protocol())
    # asyncio.run(handler.aspirate([plate11.children[0]],[10], [0]))
    # print(plate11.children[0])
    # # asyncio.run(handler.run_protocol())
    # asyncio.run(handler.dispense([plate1.children[0]],[10],[0]))
    # print(plate1.children[0])
    # asyncio.run(handler.run_protocol())
    # asyncio.run(handler.mix([plate1.children[0]], mix_time=3, mix_vol=5, height_to_bottom=0.5, offsets=Coordinate(0, 0, 0), mix_rate=100))
    # print(plate1.children[0])
    # asyncio.run(handler.discard_tips([0]))

#     asyncio.run(handler.add_liquid(
#     asp_vols=[10]*7,
#     dis_vols=[10]*7,
#     reagent_sources=plate11.children[:7],
#     targets=plate1.children[2:9],
#     use_channels=[0],
#     flow_rates=[None] * 7,
#     offsets=[Coordinate(0, 0, 0)] * 7,
#     liquid_height=[None] * 7,
#     blow_out_air_volume=[None] * 2,
#     delays=None,
#     mix_time=3,
#     mix_vol=5,
#     spread="custom",
# ))

    # asyncio.run(handler.run_protocol())  # Run the protocol




# # #     asyncio.run(handler.transfer_liquid(
# # #     asp_vols=[10]*2,
# # #     dis_vols=[10]*2,
# # #     sources=plate11.children[:2],
# # #     targets=plate11.children[-2:],
# # #     use_channels=[0],
# # #     offsets=[Coordinate(0, 0, 0)] * 4,
# # #     liquid_height=[None] * 2,
# # #     blow_out_air_volume=[None] * 2,
# # #     delays=None,
# # #     mix_times=3,
# # #     mix_vol=5,
# # #     spread="wide",
# # #     tip_racks=[plate8]
# # # ))

# # #     asyncio.run(handler.remove_liquid(
# # #     vols=[10]*2,
# # #     sources=plate11.children[:2],
# # #     waste_liquid=plate11.children[43],
# # #     use_channels=[0],
# # #     offsets=[Coordinate(0, 0, 0)] * 4,
# # #     liquid_height=[None] * 2,
# # #     blow_out_air_volume=[None] * 2,
# # #     delays=None,
# # #     spread="wide"
# # # ))
# #     asyncio.run(handler.run_protocol())

# #     # asyncio.run(handler.discard_tips())
# #     # asyncio.run(handler.mix(well_containers.children[:8
# #     # ], mix_time=3, mix_vol=50, height_to_bottom=0.5, offsets=Coordinate(0, 0, 0), mix_rate=100))
# #     #print(json.dumps(handler._unilabos_backend.steps_todo_list, indent=2))  # Print matrix info


# #     # asyncio.run(handler.remove_liquid(
# #     #     vols=[100]*16,
# #     #     sources=well_containers.children[-16:],
# #     #     waste_liquid=well_containers.children[:16], # 这个有些奇怪，但是好像也只能这么写
# #     #     use_channels=[0, 1, 2, 3, 4, 5, 6, 7],
# #     #     flow_rates=[None] * 32,
# #     #     offsets=[Coordinate(0, 0, 0)] * 32,
# #     #     liquid_height=[None] * 32,
# #     #     blow_out_air_volume=[None] * 32,
# #     #     spread="wide",
# #     # ))
# #     # asyncio.run(handler.transfer_liquid(
# #     #     asp_vols=[100]*16,
# #     #     dis_vols=[100]*16,
# #     #     tip_racks=[tip_rack],
# #     #     sources=well_containers.children[-16:],
# #     #     targets=well_containers.children[:16],
# #     #     use_channels=[0, 1, 2, 3, 4, 5, 6, 7],
# #     #     offsets=[Coordinate(0, 0, 0)] * 32,
# #     #     asp_flow_rates=[None] * 16,
# #     #     dis_flow_rates=[None] * 16,
# #     #     liquid_height=[None] * 32,
# #     #     blow_out_air_volume=[None] * 32,
# #     #     mix_times=3,
# #     #     mix_vol=50,
# #     #     spread="wide",
# #     # ))
#       # print(json.dumps(handler._unilabos_backend.steps_todo_list, indent=2))  # Print matrix info
# #     # input("pick_up_tips add step")
     #asyncio.run(handler.run_protocol())  # Run the protocol
# #     # input("Running protocol...")
# #     # input("Press Enter to continue...")  # Wait for user input before proceeding
# #     # print("PRCXI9300Handler initialized with deck and host settings.")


# 一些推荐版位组合的测试样例：

# 一些推荐版位组合的测试样例：



    with open("prcxi_material.json", "r") as f:
        material_info = json.load(f)

    layout = DefaultLayout("PRCXI9320")
    layout.add_lab_resource(material_info)
    MatrixLayout_1, dict_1 = layout.recommend_layout([
        ("reagent_1", "96 细胞培养皿", 3),
        ("reagent_2", "12道储液槽", 1),
        ("reagent_3", "200μL Tip头", 7),
        ("reagent_4", "10μL加长 Tip头", 1),
    ])
    print(dict_1)
    MatrixLayout_2, dict_2 = layout.recommend_layout([
        ("reagent_1", "96深孔板", 4),
        ("reagent_2", "12道储液槽", 1),
        ("reagent_3", "200μL Tip头", 1),
        ("reagent_4", "10μL加长 Tip头", 1),
    ])

# with open("prcxi_material.json", "r") as f:
#     material_info = json.load(f)

# layout = DefaultLayout("PRCXI9320")
# layout.add_lab_resource(material_info)
# MatrixLayout_1, dict_1 = layout.recommend_layout([
#     ("reagent_1", "96 细胞培养皿", 3),
#     ("reagent_2", "12道储液槽", 1),
#     ("reagent_3", "200μL Tip头", 7),
#     ("reagent_4", "10μL加长 Tip头", 1),
# ])
# print(dict_1)
# MatrixLayout_2, dict_2 = layout.recommend_layout([
#     ("reagent_1", "96深孔板", 4),
#     ("reagent_2", "12道储液槽", 1),
#     ("reagent_3", "200μL Tip头", 1),
#     ("reagent_4", "10μL加长 Tip头", 1),
# ])
