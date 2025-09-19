from __future__ import annotations
import re
import traceback
from typing import List, Sequence, Optional, Literal, Union, Iterator, Dict, Any, Callable, Set, cast
from collections import Counter
import asyncio
import time
import pprint as pp
from pylabrobot.liquid_handling import LiquidHandler, LiquidHandlerBackend, LiquidHandlerChatterboxBackend, Strictness
from pylabrobot.liquid_handling.liquid_handler import TipPresenceProbingMethod
from pylabrobot.liquid_handling.standard import GripDirection
from pylabrobot.resources import (
    Resource,
    TipRack,
    Container,
    Coordinate,
    Well,
    Deck,
    TipSpot,
    Plate,
    ResourceStack,
    ResourceHolder,
    Lid,
    Trash,
    Tip,
)


class LiquidHandlerMiddleware(LiquidHandler):
    def __init__(self, backend: LiquidHandlerBackend, deck: Deck, simulator: bool = False, channel_num: int = 8):
        self._simulator = simulator
        self.channel_num = channel_num
        if simulator:
            self._simulate_backend = LiquidHandlerChatterboxBackend(channel_num)
            self._simulate_handler = LiquidHandlerAbstract(self._simulate_backend, deck, False)
        super().__init__(backend, deck)

    async def setup(self, **backend_kwargs):
        if self._simulator:
            return await self._simulate_handler.setup(**backend_kwargs)
        return await super().setup(**backend_kwargs)

    def serialize_state(self) -> Dict[str, Any]:
        if self._simulator:
            self._simulate_handler.serialize_state()
        return super().serialize_state()

    def load_state(self, state: Dict[str, Any]):
        if self._simulator:
            self._simulate_handler.load_state(state)
        super().load_state(state)

    def update_head_state(self, state: Dict[int, Optional[Tip]]):
        if self._simulator:
            self._simulate_handler.update_head_state(state)
        super().update_head_state(state)

    def clear_head_state(self):
        if self._simulator:
            self._simulate_handler.clear_head_state()
        super().clear_head_state()

    def _run_async_in_thread(self, func, *args, **kwargs):
        super()._run_async_in_thread(func, *args, **kwargs)

    def _send_assigned_resource_to_backend(self, resource: Resource):
        if self._simulator:
            self._simulate_handler._send_assigned_resource_to_backend(resource)
        super()._send_assigned_resource_to_backend(resource)

    def _send_unassigned_resource_to_backend(self, resource: Resource):
        if self._simulator:
            self._simulate_handler._send_unassigned_resource_to_backend(resource)
        super()._send_unassigned_resource_to_backend(resource)

    def summary(self):
        if self._simulator:
            self._simulate_handler.summary()
        super().summary()

    def _assert_positions_unique(self, positions: List[str]):
        super()._assert_positions_unique(positions)

    def _assert_resources_exist(self, resources: Sequence[Resource]):
        super()._assert_resources_exist(resources)

    def _check_args(
        self, method: Callable, backend_kwargs: Dict[str, Any], default: Set[str], strictness: Strictness
    ) -> Set[str]:
        return super()._check_args(method, backend_kwargs, default, strictness)

    def _make_sure_channels_exist(self, channels: List[int]):
        super()._make_sure_channels_exist(channels)

    def _format_param(self, value: Any) -> Any:
        return super()._format_param(value)

    def _log_command(self, name: str, **kwargs) -> None:
        super()._log_command(name, **kwargs)

    async def pick_up_tips(
        self,
        tip_spots: List[TipSpot],
        use_channels: Optional[List[int]] = None,
        offsets: Optional[List[Coordinate]] = None,
        **backend_kwargs,
    ):

        if self._simulator:
            return await self._simulate_handler.pick_up_tips(tip_spots, use_channels, offsets, **backend_kwargs)
        return await super().pick_up_tips(tip_spots, use_channels, offsets, **backend_kwargs)

    async def drop_tips(
        self,
        tip_spots: Sequence[Union[TipSpot, Trash]],
        use_channels: Optional[List[int]] = None,
        offsets: Optional[List[Coordinate]] = None,
        allow_nonzero_volume: bool = False,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.drop_tips(
                tip_spots, use_channels, offsets, allow_nonzero_volume, **backend_kwargs
            )
        return await super().drop_tips(tip_spots, use_channels, offsets, allow_nonzero_volume, **backend_kwargs)

    async def return_tips(
        self, use_channels: Optional[list[int]] = None, allow_nonzero_volume: bool = False, **backend_kwargs
    ):
        if self._simulator:
            return await self._simulate_handler.return_tips(use_channels, allow_nonzero_volume, **backend_kwargs)
        return await super().return_tips(use_channels, allow_nonzero_volume, **backend_kwargs)

    async def discard_tips(
        self,
        use_channels: Optional[List[int]] = None,
        allow_nonzero_volume: bool = True,
        offsets: Optional[List[Coordinate]] = None,
        **backend_kwargs,
    ):
        if not offsets or (isinstance(offsets, list) and len(offsets) != len(use_channels)):
            offsets = [Coordinate.zero()] * len(use_channels)
        if self._simulator:
            return await self._simulate_handler.discard_tips(use_channels, allow_nonzero_volume, offsets, **backend_kwargs)
        return await super().discard_tips(use_channels, allow_nonzero_volume, offsets, **backend_kwargs)

    def _check_containers(self, resources: Sequence[Resource]):
        super()._check_containers(resources)

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
        if self._simulator:
            return await self._simulate_handler.aspirate(
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
        if self._simulator:
            return await self._simulate_handler.dispense(
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

    async def transfer(
        self,
        source: Well,
        targets: List[Well],
        source_vol: Optional[float] = None,
        ratios: Optional[List[float]] = None,
        target_vols: Optional[List[float]] = None,
        aspiration_flow_rate: Optional[float] = None,
        dispense_flow_rates: Optional[List[Optional[float]]] = None,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.transfer(
                source,
                targets,
                source_vol,
                ratios,
                target_vols,
                aspiration_flow_rate,
                dispense_flow_rates,
                **backend_kwargs,
            )
        return await super().transfer(
            source,
            targets,
            source_vol,
            ratios,
            target_vols,
            aspiration_flow_rate,
            dispense_flow_rates,
            **backend_kwargs,
        )

    def use_channels(self, channels: List[int]):
        if self._simulator:
            self._simulate_handler.use_channels(channels)
        return super().use_channels(channels)

    async def pick_up_tips96(self, tip_rack: TipRack, offset: Coordinate = Coordinate.zero(), **backend_kwargs):
        if self._simulator:
            return await self._simulate_handler.pick_up_tips96(tip_rack, offset, **backend_kwargs)
        return await super().pick_up_tips96(tip_rack, offset, **backend_kwargs)

    async def drop_tips96(
        self,
        resource: Union[TipRack, Trash],
        offset: Coordinate = Coordinate.zero(),
        allow_nonzero_volume: bool = False,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.drop_tips96(resource, offset, allow_nonzero_volume, **backend_kwargs)
        return await super().drop_tips96(resource, offset, allow_nonzero_volume, **backend_kwargs)

    def _get_96_head_origin_tip_rack(self) -> Optional[TipRack]:
        return super()._get_96_head_origin_tip_rack()

    async def return_tips96(self, allow_nonzero_volume: bool = False, **backend_kwargs):
        if self._simulator:
            return await self._simulate_handler.return_tips96(allow_nonzero_volume, **backend_kwargs)
        return await super().return_tips96(allow_nonzero_volume, **backend_kwargs)

    async def discard_tips96(self, allow_nonzero_volume: bool = True, **backend_kwargs):
        if self._simulator:
            return await self._simulate_handler.discard_tips96(allow_nonzero_volume, **backend_kwargs)
        return await super().discard_tips96(allow_nonzero_volume, **backend_kwargs)

    async def aspirate96(
        self,
        resource: Union[Plate, Container, List[Well]],
        volume: float,
        offset: Coordinate = Coordinate.zero(),
        flow_rate: Optional[float] = None,
        blow_out_air_volume: Optional[float] = None,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.aspirate96(
                resource, volume, offset, flow_rate, blow_out_air_volume, **backend_kwargs
            )
        return await super().aspirate96(resource, volume, offset, flow_rate, blow_out_air_volume, **backend_kwargs)

    async def dispense96(
        self,
        resource: Union[Plate, Container, List[Well]],
        volume: float,
        offset: Coordinate = Coordinate.zero(),
        flow_rate: Optional[float] = None,
        blow_out_air_volume: Optional[float] = None,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.dispense96(
                resource, volume, offset, flow_rate, blow_out_air_volume, **backend_kwargs
            )
        return await super().dispense96(resource, volume, offset, flow_rate, blow_out_air_volume, **backend_kwargs)

    async def stamp(
        self,
        source: Plate,
        target: Plate,
        volume: float,
        aspiration_flow_rate: Optional[float] = None,
        dispense_flow_rate: Optional[float] = None,
    ):
        if self._simulator:
            return await self._simulate_handler.stamp(source, target, volume, aspiration_flow_rate, dispense_flow_rate)
        return await super().stamp(source, target, volume, aspiration_flow_rate, dispense_flow_rate)

    async def pick_up_resource(
        self,
        resource: Resource,
        offset: Coordinate = Coordinate.zero(),
        pickup_distance_from_top: float = 0,
        direction: GripDirection = GripDirection.FRONT,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.pick_up_resource(
                resource, offset, pickup_distance_from_top, direction, **backend_kwargs
            )
        return await super().pick_up_resource(resource, offset, pickup_distance_from_top, direction, **backend_kwargs)

    async def move_picked_up_resource(
        self,
        to: Coordinate,
        offset: Coordinate = Coordinate.zero(),
        direction: Optional[GripDirection] = None,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.move_picked_up_resource(to, offset, direction, **backend_kwargs)
        return await super().move_picked_up_resource(to, offset, direction, **backend_kwargs)

    async def drop_resource(
        self,
        destination: Union[ResourceStack, ResourceHolder, Resource, Coordinate],
        offset: Coordinate = Coordinate.zero(),
        direction: GripDirection = GripDirection.FRONT,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.drop_resource(destination, offset, direction, **backend_kwargs)
        return await super().drop_resource(destination, offset, direction, **backend_kwargs)

    async def move_resource(
        self,
        resource: Resource,
        to: Union[ResourceStack, ResourceHolder, Resource, Coordinate],
        intermediate_locations: Optional[List[Coordinate]] = None,
        pickup_offset: Coordinate = Coordinate.zero(),
        destination_offset: Coordinate = Coordinate.zero(),
        pickup_distance_from_top: float = 0,
        pickup_direction: GripDirection = GripDirection.FRONT,
        drop_direction: GripDirection = GripDirection.FRONT,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.move_resource(
                resource,
                to,
                intermediate_locations,
                pickup_offset,
                destination_offset,
                pickup_distance_from_top,
                pickup_direction,
                drop_direction,
                **backend_kwargs,
            )
        return await super().move_resource(
            resource,
            to,
            intermediate_locations,
            pickup_offset,
            destination_offset,
            pickup_distance_from_top,
            pickup_direction,
            drop_direction,
            **backend_kwargs,
        )

    async def move_lid(
        self,
        lid: Lid,
        to: Union[Plate, ResourceStack, Coordinate],
        intermediate_locations: Optional[List[Coordinate]] = None,
        pickup_offset: Coordinate = Coordinate.zero(),
        destination_offset: Coordinate = Coordinate.zero(),
        pickup_direction: GripDirection = GripDirection.FRONT,
        drop_direction: GripDirection = GripDirection.FRONT,
        pickup_distance_from_top: float = 5.7 - 3.33,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.move_lid(
                lid,
                to,
                intermediate_locations,
                pickup_offset,
                destination_offset,
                pickup_direction,
                drop_direction,
                pickup_distance_from_top,
                **backend_kwargs,
            )
        return await super().move_lid(
            lid,
            to,
            intermediate_locations,
            pickup_offset,
            destination_offset,
            pickup_direction,
            drop_direction,
            pickup_distance_from_top,
            **backend_kwargs,
        )

    async def move_plate(
        self,
        plate: Plate,
        to: Union[ResourceStack, ResourceHolder, Resource, Coordinate],
        intermediate_locations: Optional[List[Coordinate]] = None,
        pickup_offset: Coordinate = Coordinate.zero(),
        destination_offset: Coordinate = Coordinate.zero(),
        drop_direction: GripDirection = GripDirection.FRONT,
        pickup_direction: GripDirection = GripDirection.FRONT,
        pickup_distance_from_top: float = 13.2 - 3.33,
        **backend_kwargs,
    ):
        if self._simulator:
            return await self._simulate_handler.move_plate(
                plate,
                to,
                intermediate_locations,
                pickup_offset,
                destination_offset,
                drop_direction,
                pickup_direction,
                pickup_distance_from_top,
                **backend_kwargs,
            )
        return await super().move_plate(
            plate,
            to,
            intermediate_locations,
            pickup_offset,
            destination_offset,
            drop_direction,
            pickup_direction,
            pickup_distance_from_top,
            **backend_kwargs,
        )

    def serialize(self):
        if self._simulator:
            self._simulate_handler.serialize()
        return super().serialize()

    @classmethod
    def deserialize(cls, data: dict, allow_marshal: bool = False) -> LiquidHandler:
        return super().deserialize(data, allow_marshal)

    @classmethod
    def load(cls, path: str) -> LiquidHandler:
        return super().load(path)

    async def prepare_for_manual_channel_operation(self, channel: int):
        if self._simulator:
            return await self._simulate_handler.prepare_for_manual_channel_operation(channel)
        return await super().prepare_for_manual_channel_operation(channel)

    async def move_channel_x(self, channel: int, x: float):
        if self._simulator:
            return await self._simulate_handler.move_channel_x(channel, x)
        return await super().move_channel_x(channel, x)

    async def move_channel_y(self, channel: int, y: float):
        if self._simulator:
            return await self._simulate_handler.move_channel_y(channel, y)
        return await super().move_channel_y(channel, y)

    async def move_channel_z(self, channel: int, z: float):
        if self._simulator:
            return await self._simulate_handler.move_channel_z(channel, z)
        return await super().move_channel_z(channel, z)

    def assign_child_resource(self, resource: Resource, location: Optional[Coordinate], reassign: bool = True):
        if self._simulator:
            self._simulate_handler.assign_child_resource(resource, location, reassign)
        pass

    async def probe_tip_presence_via_pickup(
        self, tip_spots: List[TipSpot], use_channels: Optional[List[int]] = None
    ) -> Dict[str, bool]:
        if self._simulator:
            return await self._simulate_handler.probe_tip_presence_via_pickup(tip_spots, use_channels)
        return await super().probe_tip_presence_via_pickup(tip_spots, use_channels)

    async def probe_tip_inventory(
        self,
        tip_spots: List[TipSpot],
        probing_fn: Optional[TipPresenceProbingMethod] = None,
        use_channels: Optional[List[int]] = None,
    ) -> Dict[str, bool]:
        if self._simulator:
            return await self._simulate_handler.probe_tip_inventory(tip_spots, probing_fn, use_channels)
        return await super().probe_tip_inventory(tip_spots, probing_fn, use_channels)

    async def consolidate_tip_inventory(self, tip_racks: List[TipRack], use_channels: Optional[List[int]] = None):
        if self._simulator:
            return await self._simulate_handler.consolidate_tip_inventory(tip_racks, use_channels)
        return await super().consolidate_tip_inventory(tip_racks, use_channels)


class LiquidHandlerAbstract(LiquidHandlerMiddleware):
    """Extended LiquidHandler with additional operations."""
    support_touch_tip = True

    def __init__(self, backend: LiquidHandlerBackend, deck: Deck, simulator: bool=False, channel_num:int = 8):
        """Initialize a LiquidHandler.

        Args:
          backend: Backend to use.
          deck: Deck to use.
        """
        self._simulator = simulator
        self.group_info = dict()
        super().__init__(backend, deck, simulator, channel_num)

    @classmethod
    def set_liquid(self, wells: list[Well], liquid_names: list[str], volumes: list[float]):
        """Set the liquid in a well."""
        for well, liquid_name, volume in zip(wells, liquid_names, volumes):
            well.set_liquids([(liquid_name, volume)])  # type: ignore
    # ---------------------------------------------------------------
    # REMOVE LIQUID --------------------------------------------------
    # ---------------------------------------------------------------

    def set_group(self, group_name: str, wells: List[Well], volumes: List[float]):
        if self.channel_num == 8 and len(wells) != 8:
            raise RuntimeError(f"Expected 8 wells, got {len(wells)}")
        self.group_info[group_name] = wells
        self.set_liquid(wells, [group_name] * len(wells), volumes)

    async def transfer_group(self, source_group_name: str, target_group_name: str, unit_volume: float):

        source_wells = self.group_info.get(source_group_name, [])
        target_wells = self.group_info.get(target_group_name, [])
        
        rack_info = dict()
        for child in self.deck.children:
            if issubclass(child.__class__, TipRack):
                rack: TipRack = cast(TipRack, child)
                if "plate" not in rack.name.lower():
                    for tip in rack.get_all_tips():
                        if unit_volume > tip.maximal_volume:
                            break
                        else:
                            rack_info[rack.name] = (rack, tip.maximal_volume - unit_volume)
        
        if len(rack_info) == 0:
            raise ValueError(f"No tip rack can support volume {unit_volume}.")
        
        rack_info = sorted(rack_info.items(), key=lambda x: x[1][1])
        for child in self.deck.children:
            if child.name == rack_info[0][0]:
                target_rack = child
        target_rack = cast(TipRack, target_rack)
        available_tips = {}
        for (idx, tipSpot) in enumerate(target_rack.get_all_items()):
            if tipSpot.has_tip():
                available_tips[idx] = tipSpot
                continue
        # 一般移动液体有两种方式，一对多和多对多
        print("channel_num", self.channel_num)
        if self.channel_num == 8:

            tip_prefix = list(available_tips.values())[0].name.split('_')[0]
            colnum_list = [int(tip.name.split('_')[-1][1:]) for tip in available_tips.values()]
            available_cols = [colnum for colnum, count in dict(Counter(colnum_list)).items() if count == 8]
            available_cols.sort() 
            available_tips_dict = {tip.name: tip for tip in available_tips.values()}
            tips_to_use = [available_tips_dict[f"{tip_prefix}_{chr(65 + i)}{available_cols[0]}"] for i in range(8)]
            print("tips_to_use", tips_to_use)
            await self.pick_up_tips(tips_to_use, use_channels=list(range(0, 8)))
            print("source_wells", source_wells)
            await self.aspirate(source_wells, [unit_volume] * 8, use_channels=list(range(0, 8)))
            print("target_wells", target_wells)
            await self.dispense(target_wells, [unit_volume] * 8, use_channels=list(range(0, 8)))
            await self.discard_tips(use_channels=list(range(0, 8)))

        elif self.channel_num == 1: 
            
            for num_well in range(len(target_wells)):
                tip_to_use = available_tips[list(available_tips.keys())[num_well]] 
                print("tip_to_use", tip_to_use)
                await self.pick_up_tips([tip_to_use], use_channels=[0])
                print("source_wells", source_wells)
                print("target_wells", target_wells)
                if len(source_wells) == 1:
                    await self.aspirate([source_wells[0]], [unit_volume], use_channels=[0]) 
                else:
                    await self.aspirate([source_wells[num_well]], [unit_volume], use_channels=[0])
                await self.dispense([target_wells[num_well]], [unit_volume], use_channels=[0])
                await self.discard_tips(use_channels=[0])

        else:
            raise ValueError(f"Unsupported channel number {self.channel_num}.")

    async def create_protocol(
        self,
        protocol_name: str,
        protocol_description: str,
        protocol_version: str,
        protocol_author: str,
        protocol_date: str,
        protocol_type: str,
        none_keys: List[str] = [],
    ):
        """Create a new protocol with the given metadata."""
        pass


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
        """A complete *remove* (aspirate → waste) operation."""

        try:
            if is_96_well:
                pass  # This mode is not verified.
            else:
                # 首先应该对任务分组，然后每次1个/8个进行操作处理
                if len(use_channels) == 1 and self.backend.num_channels == 1:

                    for _ in range(len(sources)):
                        tip = []
                        for __ in range(len(use_channels)):
                            tip.extend(next(self.current_tip))
                        await self.pick_up_tips(tip)
                        await self.aspirate(
                            resources=[sources[_]],
                            vols=[vols[_]],
                            use_channels=use_channels,
                            flow_rates=[flow_rates[0]] if flow_rates else None,
                            offsets=[offsets[0]] if offsets else None,
                            liquid_height=[liquid_height[0]] if liquid_height else None,
                            blow_out_air_volume=[blow_out_air_volume[0]] if blow_out_air_volume else None,
                            spread=spread,
                        )
                        if delays is not None:
                            await self.custom_delay(seconds=delays[0])

                        await self.dispense(
                            resources=[waste_liquid],
                            vols=[vols[_]],
                            use_channels=use_channels,
                            flow_rates=[flow_rates[1]] if flow_rates else None,
                            offsets=[offsets[1]] if offsets else None,
                            blow_out_air_volume=[blow_out_air_volume[1]] if blow_out_air_volume else None,
                            liquid_height=[liquid_height[1]] if liquid_height else None,
                            spread=spread,
                        )
                        await self.discard_tips()

                elif len(use_channels) == 8 and self.backend.num_channels == 8:
               
                  
                    # 对于8个的情况，需要判断此时任务是不是能被8通道移液站来成功处理
                    if len(sources) % 8 != 0:
                        raise ValueError(f"Length of `sources` {len(sources)} must be a multiple of 8 for 8-channel mode.")

                    # 8个8个来取任务序列

                    for i in range(0, len(sources), 8):
                        tip = []
                        for _ in range(len(use_channels)):
                            tip.extend(next(self.current_tip))
                        await self.pick_up_tips(tip)
                        current_targets = waste_liquid[i:i + 8]
                        current_reagent_sources = sources[i:i + 8]
                        current_asp_vols = vols[i:i + 8]
                        current_dis_vols = vols[i:i + 8]
                        current_asp_flow_rates = flow_rates[i:i + 8] if flow_rates else [None] * 8
                        current_dis_flow_rates = flow_rates[-i*8-8:len(flow_rates)-i*8] if flow_rates else [None] * 8
                        current_asp_offset = offsets[i:i + 8] if offsets else [None] * 8
                        current_dis_offset = offsets[-i*8-8:len(offsets)-i*8] if offsets else [None] * 8
                        current_asp_liquid_height = liquid_height[i:i + 8] if liquid_height else [None] * 8
                        current_dis_liquid_height = liquid_height[-i*8-8:len(liquid_height)-i*8] if liquid_height else [None] * 8
                        current_asp_blow_out_air_volume = blow_out_air_volume[i:i + 8] if blow_out_air_volume else [None] * 8
                        current_dis_blow_out_air_volume = blow_out_air_volume[-i*8-8:len(blow_out_air_volume)-i*8] if blow_out_air_volume else [None] * 8

                        await self.aspirate(
                            resources=current_reagent_sources,
                            vols=current_asp_vols,
                            use_channels=use_channels,
                            flow_rates=current_asp_flow_rates,
                            offsets=current_asp_offset,
                            liquid_height=current_asp_liquid_height,
                            blow_out_air_volume=current_asp_blow_out_air_volume,
                            spread=spread,
                        )
                        if delays is not None:
                            await self.custom_delay(seconds=delays[0])
                        await self.dispense(
                            resources=current_targets,
                            vols=current_dis_vols,
                            use_channels=use_channels,
                            flow_rates=current_dis_flow_rates,
                            offsets=current_dis_offset,
                            liquid_height=current_dis_liquid_height,
                            blow_out_air_volume=current_dis_blow_out_air_volume,
                            spread=spread,
                        )
                        if delays is not None:
                            await self.custom_delay(seconds=delays[1])
                        await self.touch_tip(current_targets)
                        await self.discard_tips()        

        except Exception as e:
            traceback.print_exc()
            raise RuntimeError(f"Liquid addition failed: {e}") from e

    # ---------------------------------------------------------------
    # ADD LIQUID -----------------------------------------------------
    # ---------------------------------------------------------------

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
        # """A complete *add* (aspirate reagent → dispense into targets) operation."""

        # # try:
            if is_96_well:
                pass  # This mode is not verified.
            else:
                if len(asp_vols) != len(targets):
                    raise ValueError(f"Length of `asp_vols` {len(asp_vols)} must match `targets` {len(targets)}.")
                # 首先应该对任务分组，然后每次1个/8个进行操作处理
                if len(use_channels) == 1:
                    for _ in range(len(targets)):
                        tip = []
                        for x in range(len(use_channels)):
                            tip.extend(next(self.current_tip))
                        await self.pick_up_tips(tip)

                        await self.aspirate(
                            resources=[reagent_sources[_]],
                            vols=[asp_vols[_]],
                            use_channels=use_channels,
                            flow_rates=[flow_rates[0]] if flow_rates else None,
                            offsets=[offsets[0]] if offsets else None,
                            liquid_height=[liquid_height[0]] if liquid_height else None,
                            blow_out_air_volume=[blow_out_air_volume[0]] if blow_out_air_volume else None,
                            spread=spread,
                        )
                        
                        if delays is not None:
                            await self.custom_delay(seconds=delays[0])
                        await self.dispense(
                            resources=[targets[_]],
                            vols=[dis_vols[_]],
                            use_channels=use_channels,
                            flow_rates=[flow_rates[1]] if flow_rates else None,
                            offsets=[offsets[1]] if offsets else None,
                            blow_out_air_volume=[blow_out_air_volume[1]] if blow_out_air_volume else None,
                            liquid_height=[liquid_height[1]] if liquid_height else None,
                            spread=spread,
                        )

                        if delays is not None:
                            await self.custom_delay(seconds=delays[1])
                        await self.mix(
                            targets=[targets[_]],
                            mix_time=mix_time,
                            mix_vol=mix_vol,
                            offsets=offsets if offsets else None,
                            height_to_bottom=mix_liquid_height if mix_liquid_height else None,
                            mix_rate=mix_rate if mix_rate else None,
                        )
                        if delays is not None:
                            await self.custom_delay(seconds=delays[1])
                        await self.touch_tip(targets[_])
                        await self.discard_tips()

                elif len(use_channels) == 8:
                    # 对于8个的情况，需要判断此时任务是不是能被8通道移液站来成功处理
                    if len(targets) % 8 != 0:
                        raise ValueError(f"Length of `targets` {len(targets)} must be a multiple of 8 for 8-channel mode.")

                    for i in range(0, len(targets), 8):
                        tip = []
                        for _ in range(len(use_channels)):
                            tip.extend(next(self.current_tip))
                        await self.pick_up_tips(tip)
                        current_targets = targets[i:i + 8]
                        current_reagent_sources = reagent_sources[i:i + 8]
                        current_asp_vols = asp_vols[i:i + 8]
                        current_dis_vols = dis_vols[i:i + 8]
                        current_asp_flow_rates = flow_rates[i:i + 8] if flow_rates else [None] * 8
                        current_dis_flow_rates = flow_rates[-i*8-8:len(flow_rates)-i*8] if flow_rates else [None] * 8
                        current_asp_offset = offsets[i:i + 8] if offsets else [None] * 8
                        current_dis_offset = offsets[-i*8-8:len(offsets)-i*8] if offsets else [None] * 8
                        current_asp_liquid_height = liquid_height[i:i + 8] if liquid_height else [None] * 8
                        current_dis_liquid_height = liquid_height[-i*8-8:len(liquid_height)-i*8] if liquid_height else [None] * 8
                        current_asp_blow_out_air_volume = blow_out_air_volume[i:i + 8] if blow_out_air_volume else [None] * 8
                        current_dis_blow_out_air_volume = blow_out_air_volume[-i*8-8:len(blow_out_air_volume)-i*8] if blow_out_air_volume else [None] * 8

                        await self.aspirate(
                            resources=current_reagent_sources,
                            vols=current_asp_vols,
                            use_channels=use_channels,
                            flow_rates=current_asp_flow_rates,
                            offsets=current_asp_offset,
                            liquid_height=current_asp_liquid_height,
                            blow_out_air_volume=current_asp_blow_out_air_volume,
                            spread=spread,
                        )
                        if delays is not None:
                            await self.custom_delay(seconds=delays[0])
                        await self.dispense(
                            resources=current_targets,
                            vols=current_dis_vols,
                            use_channels=use_channels,
                            flow_rates=current_dis_flow_rates,
                            offsets=current_dis_offset,
                            liquid_height=current_dis_liquid_height,
                            blow_out_air_volume=current_dis_blow_out_air_volume,
                            spread=spread,
                        )
                        if delays is not None:
                            await self.custom_delay(seconds=delays[1])

                        await self.mix(
                            targets=current_targets,
                            mix_time=mix_time,
                            mix_vol=mix_vol,
                            offsets=offsets if offsets else None,
                            height_to_bottom=mix_liquid_height if mix_liquid_height else None,
                            mix_rate=mix_rate if mix_rate else None,
                        )
                        if delays is not None:
                            await self.custom_delay(seconds=delays[1])
                        await self.touch_tip(current_targets)
                        await self.discard_tips()


        # except Exception as e:
        #     traceback.print_exc()
        #     raise RuntimeError(f"Liquid addition failed: {e}") from e

    # ---------------------------------------------------------------
    # TRANSFER LIQUID ------------------------------------------------
    # ---------------------------------------------------------------
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
        """Transfer liquid from each *source* well/plate to the corresponding *target*.

        Parameters
        ----------
        asp_vols, dis_vols
            Single volume (µL) or list matching the number of transfers.
        sources, targets
            Same‑length sequences of containers (wells or plates). In 96‑well mode
            each must contain exactly one plate.
        tip_racks
            One or more TipRacks providing fresh tips.
        is_96_well
            Set *True* to use the 96‑channel head.
        """


        if is_96_well:
            pass  # This mode is not verified.
        else:
            if len(asp_vols) != len(targets):
                raise ValueError(f"Length of `asp_vols` {len(asp_vols)} must match `targets` {len(targets)}.")

            # 首先应该对任务分组，然后每次1个/8个进行操作处理
            if len(use_channels) == 1:
                for _ in range(len(targets)):
                    tip = []
                    for ___ in range(len(use_channels)):
                        tip.extend(next(self.current_tip))
                    await self.pick_up_tips(tip)

                    await self.aspirate(
                        resources=[sources[_]],
                        vols=[asp_vols[_]],
                        use_channels=use_channels,
                        flow_rates=[asp_flow_rates[0]] if asp_flow_rates else None,
                        offsets=[offsets[0]] if offsets else None,
                        liquid_height=[liquid_height[0]] if liquid_height else None,
                        blow_out_air_volume=[blow_out_air_volume[0]] if blow_out_air_volume else None,
                        spread=spread,
                    )
                    if delays is not None:
                        await self.custom_delay(seconds=delays[0])
                    await self.dispense(
                        resources=[targets[_]],
                        vols=[dis_vols[_]],
                        use_channels=use_channels,
                        flow_rates=[dis_flow_rates[1]] if dis_flow_rates else None,
                        offsets=[offsets[1]] if offsets else None,
                        blow_out_air_volume=[blow_out_air_volume[1]] if blow_out_air_volume else None,
                        liquid_height=[liquid_height[1]] if liquid_height else None,
                        spread=spread,
                    )
                    if delays is not None:
                        await self.custom_delay(seconds=delays[1])
                    await self.mix(
                        targets=[targets[_]],
                        mix_time=mix_times,
                        mix_vol=mix_vol,
                        offsets=offsets if offsets else None,
                        height_to_bottom=mix_liquid_height if mix_liquid_height else None,
                        mix_rate=mix_rate if mix_rate else None,
                    )
                    if delays is not None:
                        await self.custom_delay(seconds=delays[1])
                    await self.touch_tip(targets[_])
                    await self.discard_tips()

            elif len(use_channels) == 8:
                # 对于8个的情况，需要判断此时任务是不是能被8通道移液站来成功处理
                if len(targets) % 8 != 0:
                    raise ValueError(f"Length of `targets` {len(targets)} must be a multiple of 8 for 8-channel mode.")

                # 8个8个来取任务序列

                for i in range(0, len(targets), 8):
                    # 取出8个任务
                    tip = []
                    for _ in range(len(use_channels)):
                        tip.extend(next(self.current_tip))
                    await self.pick_up_tips(tip)
                    current_targets = targets[i:i + 8]
                    current_reagent_sources = sources[i:i + 8]
                    current_asp_vols = asp_vols[i:i + 8]
                    current_dis_vols = dis_vols[i:i + 8]
                    current_asp_flow_rates = asp_flow_rates[i:i + 8]
                    current_asp_offset = offsets[i:i + 8] if offsets else [None] * 8
                    current_dis_offset = offsets[-i*8-8:len(offsets)-i*8] if offsets else [None] * 8
                    current_asp_liquid_height = liquid_height[i:i + 8] if liquid_height else [None] * 8
                    current_dis_liquid_height = liquid_height[-i*8-8:len(liquid_height)-i*8] if liquid_height else [None] * 8
                    current_asp_blow_out_air_volume = blow_out_air_volume[i:i + 8] if blow_out_air_volume else [None] * 8
                    current_dis_blow_out_air_volume = blow_out_air_volume[-i*8-8:len(blow_out_air_volume)-i*8] if blow_out_air_volume else [None] * 8
                    current_dis_flow_rates = dis_flow_rates[i:i + 8] if dis_flow_rates else [None] * 8

                    await self.aspirate(
                        resources=current_reagent_sources,
                        vols=current_asp_vols,
                        use_channels=use_channels,
                        flow_rates=current_asp_flow_rates,
                        offsets=current_asp_offset,
                        blow_out_air_volume=current_asp_blow_out_air_volume,
                        liquid_height=current_asp_liquid_height,
                        spread=spread,
                    )

                    if delays is not None:
                        await self.custom_delay(seconds=delays[0])
                    await self.dispense(
                        resources=current_targets,
                        vols=current_dis_vols,
                        use_channels=use_channels,
                        flow_rates=current_dis_flow_rates,
                        offsets=current_dis_offset,
                        blow_out_air_volume=current_dis_blow_out_air_volume,
                        liquid_height=current_dis_liquid_height,
                        spread=spread,
                    )
                    if delays is not None:
                        await self.custom_delay(seconds=delays[1])

                    await self.mix(
                        targets=current_targets,
                        mix_time=mix_times,
                        mix_vol=mix_vol,
                        offsets=offsets if offsets else None,
                        height_to_bottom=mix_liquid_height if mix_liquid_height else None,
                        mix_rate=mix_rate if mix_rate else None,
                    )
                    if delays is not None:
                        await self.custom_delay(seconds=delays[1])
                    await self.touch_tip(current_targets)
                    await self.discard_tips([0,1,2,3,4,5,6,7])

    # except Exception as e:
    #     traceback.print_exc()
    #     raise RuntimeError(f"Liquid addition failed: {e}") from e


    # ---------------------------------------------------------------
    # Helper utilities
    # ---------------------------------------------------------------

    async def custom_delay(self, seconds=0, msg=None):
        """
        seconds: seconds to wait
        msg: information to be printed
        """
        if seconds != None and seconds > 0:
            if msg:
                print(f"Waiting time: {msg}")
                print(f"Current time: {time.strftime('%H:%M:%S')}")
                print(f"Time to finish: {time.strftime('%H:%M:%S', time.localtime(time.time() + seconds))}")
            await asyncio.sleep(seconds)
            if msg:
                print(f"Done: {msg}")
                print(f"Current time: {time.strftime('%H:%M:%S')}")

    async def touch_tip(self, targets: Sequence[Container]):

        """Touch the tip to the side of the well."""

        if not self.support_touch_tip:
            return
        await self.aspirate(
            resources=[targets],
            vols=[0],
            use_channels=None,
            flow_rates=None,
            offsets=[Coordinate(x=-targets.get_size_x() / 2, y=0, z=0)],
            liquid_height=None,
            blow_out_air_volume=None,
        )
        # await self.custom_delay(seconds=1) # In the simulation, we do not need to wait
        await self.aspirate(
            resources=[targets],
            vols=[0],
            use_channels=None,
            flow_rates=None,
            offsets=[Coordinate(x=targets.get_size_x() / 2, y=0, z=0)],
            liquid_height=None,
            blow_out_air_volume=None,
        )

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
        if mix_time is None:  # No mixing required
            return
        """Mix the liquid in the target wells."""
        for _ in range(mix_time):
            await self.aspirate(
                resources=[targets],
                vols=[mix_vol],
                flow_rates=[mix_rate] if mix_rate else None,
                offsets=[offsets] if offsets else None,
                liquid_height=[height_to_bottom] if height_to_bottom else None,
            )
            await self.custom_delay(seconds=1)
            await self.dispense(
                resources=[targets],
                vols=[mix_vol],
                flow_rates=[mix_rate] if mix_rate else None,
                offsets=[offsets] if offsets else None,
                liquid_height=[height_to_bottom] if height_to_bottom else None,
            )

    def iter_tips(self, tip_racks: Sequence[TipRack]) -> Iterator[Resource]:
        """Yield tips from a list of TipRacks one-by-one until depleted."""
        for rack in tip_racks:
            for tip in rack:
                yield tip
        raise RuntimeError("Out of tips!")

    def set_tiprack(self, tip_racks: Sequence[TipRack]):
        """Set the tip racks for the liquid handler."""

        self.tip_racks = tip_racks
        tip_iter = self.iter_tips(tip_racks)
        self.current_tip = tip_iter

    async def move_to(self, well: Well, dis_to_top: float = 0, channel: int = 0):
        """
        Move a single channel to a specific well with a given z-height.

        Parameters
        ----------
        well : Well
            The target well.
        dis_to_top : float
            Height in mm to move to relative to the well top.
        channel : int
            Pipetting channel to move (default: 0).
        """
        await self.prepare_for_manual_channel_operation(channel=channel)
        abs_loc = well.get_absolute_location()
        well_height = well.get_absolute_size_z()
        await self.move_channel_x(channel, abs_loc.x)
        await self.move_channel_y(channel, abs_loc.y)
        await self.move_channel_z(channel, abs_loc.z + well_height + dis_to_top)
