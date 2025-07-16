from __future__ import annotations

import traceback
from typing import List, Sequence, Optional, Literal, Union, Iterator

import asyncio
import time

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.resources import Resource, TipRack, Container, Coordinate, Well


class LiquidHandlerAbstract(LiquidHandler):
    """Extended LiquidHandler with additional operations."""

    # ---------------------------------------------------------------
    # REMOVE LIQUID --------------------------------------------------
    # ---------------------------------------------------------------

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
                if len(use_channels) == 1:
                    tip = []
                    for _ in range(len(use_channels)):
                        tip.extend(next(self.current_tip))
                    await self.pick_up_tips(tip)

                    for _ in range(len(waste_liquid)):
                        await self.aspirate(
                            resources=sources,
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
                            resources=[waste_liquid[_]],
                            vols=[vols[_]],
                            use_channels=use_channels,
                            flow_rates=[flow_rates[1]] if flow_rates else None,
                            offsets=[offsets[1]] if offsets else None,
                            blow_out_air_volume=[blow_out_air_volume[1]] if blow_out_air_volume else None,
                            liquid_height=[liquid_height[1]] if liquid_height else None,
                            spread=spread,
                        )
                    await self.discard_tips()
                elif len(use_channels) == 8:
                    tip = []
                    for _ in range(len(use_channels)):
                        tip.extend(next(self.current_tip))
                    await self.pick_up_tips(tip)

                    # 对于8个的情况，需要判断此时任务是不是能被8通道移液站来成功处理
                    if len(sources) % 8 != 0:
                        raise ValueError(f"Length of `sources` {len(sources)} must be a multiple of 8 for 8-channel mode.")

                    # 8个8个来取任务序列

                    for i in range(0, len(sources), 8):
                        # 取出8个任务
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

                        #await self.touch_tip(current_targets)

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
        """A complete *add* (aspirate reagent → dispense into targets) operation."""

        try:
            if is_96_well:
                pass  # This mode is not verified.
            else:
                if len(asp_vols) != len(targets):
                    raise ValueError(f"Length of `asp_vols` {len(asp_vols)} must match `targets` {len(targets)}.")
                
                # 首先应该对任务分组，然后每次1个/8个进行操作处理
                if len(use_channels) == 1:
                    tip = []
                    for _ in range(len(use_channels)):
                        tip.extend(next(self.current_tip))
                    await self.pick_up_tips(tip)

                    for _ in range(len(targets)):
                        await self.aspirate(
                            resources=reagent_sources,
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
                            targets=targets[_],
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
                    tip = []
                    for _ in range(len(use_channels)):
                        tip.extend(next(self.current_tip))
                    await self.pick_up_tips(tip)

                    # 对于8个的情况，需要判断此时任务是不是能被8通道移液站来成功处理
                    if len(targets) % 8 != 0:
                        raise ValueError(f"Length of `targets` {len(targets)} must be a multiple of 8 for 8-channel mode.")
                    
                    # 8个8个来取任务序列

                    for i in range(0, len(targets), 8):
                        # 取出8个任务
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
                        #await self.touch_tip(current_targets)

        except Exception as e:
            traceback.print_exc()
            raise RuntimeError(f"Liquid addition failed: {e}") from e

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

        try:
            if is_96_well:
                pass  # This mode is not verified.
            else:
                if len(asp_vols) != len(targets):
                    raise ValueError(f"Length of `asp_vols` {len(asp_vols)} must match `targets` {len(targets)}.")
                
                # 首先应该对任务分组，然后每次1个/8个进行操作处理
                if len(use_channels) == 1:
                    tip = []
                    for _ in range(len(use_channels)):
                        tip.extend(next(self.current_tip))
                    await self.pick_up_tips(tip)

                    for _ in range(len(targets)):
                        await self.aspirate(
                            resources=sources,
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
                            targets=targets[_],
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
                    tip = []
                    for _ in range(len(use_channels)):
                        tip.extend(next(self.current_tip))
                    await self.pick_up_tips(tip)

                    # 对于8个的情况，需要判断此时任务是不是能被8通道移液站来成功处理
                    if len(targets) % 8 != 0:
                        raise ValueError(f"Length of `targets` {len(targets)} must be a multiple of 8 for 8-channel mode.")
                    
                    # 8个8个来取任务序列

                    for i in range(0, len(targets), 8):
                        # 取出8个任务
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
                        
                        #await self.touch_tip(current_targets)
        except Exception as e:
            traceback.print_exc()
            raise RuntimeError(f"Liquid addition failed: {e}") from e


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
