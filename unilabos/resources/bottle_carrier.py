"""
自动化液体处理工作站物料类定义 - 简化版
Automated Liquid Handling Station Resource Classes - Simplified Version
"""

from __future__ import annotations

from typing import Dict, Optional

from pylabrobot.resources.coordinate import Coordinate
from pylabrobot.resources.container import Container
from pylabrobot.resources.carrier import TubeCarrier
from pylabrobot.resources.resource_holder import ResourceHolder


class Bottle(Container):
    """瓶子类 - 简化版，不追踪瓶盖"""

    def __init__(
        self,
        name: str,
        diameter: float,
        height: float,
        max_volume: float,
        barcode: Optional[str] = "",
        category: str = "container",
        model: Optional[str] = None,
    ):
        super().__init__(
            name=name,
            size_x=diameter,
            size_y=diameter,
            size_z=height,
            max_volume=max_volume,
            category=category,
            model=model,
        )
        self.diameter = diameter
        self.height = height
        self.barcode = barcode

    def serialize(self) -> dict:
        return {
            **super().serialize(),
            "diameter": self.diameter,
            "height": self.height,
            "barcode": self.barcode,
        }


class BottleCarrier(TubeCarrier):
    """瓶载架 - 直接继承自 TubeCarrier"""

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        sites: Optional[Dict[int, ResourceHolder]] = None,
        category: str = "bottle_carrier",
        model: Optional[str] = None,
    ):
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            sites=sites,
            category=category,
            model=model,
        )
