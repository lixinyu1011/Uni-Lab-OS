from os import name
from typing import Optional

from pylabrobot.resources import Deck, Coordinate, Rotation, Resource
from unilabos.devices.workstation.ResinWorkstation.warehouses import (
    Hydrogel_warehouse_1x5x1,
    Station_1_warehouse_4x2x1,
    Station_2_warehouse_1x1x1,
    Station_3_warehouse_1x1x1,
    Raw_electrode_warehouse_3x5x1,
    Finished_electrode_warehouse_3x5x1,
    Stir_1_warehouse_1x1x1,
    Stir_2_warehouse_1x1x1,
    Water_wash_warehouse_1x1x1,
    Acid_wash_warehouse_1x1x1,
)



class ResinWorkstation_Deck(Deck):
    def __init__(
        self,
        name: str = "ResinWorkstation_Deck",
        size_x: float = 1217.0,
        size_y: float = 1560.0,
        size_z: float = 2670.0,
        origin: Coordinate = Coordinate(0, 35, 0),
        category: str = "deck",
        setup: bool = False,
    ) -> None:
        super().__init__(name=name, size_x=size_x, size_y=size_y, size_z=size_z, origin=origin)
        self.warehouses = {}
        self.warehouse_locations = {}
        if setup:
            self.setup()

    def setup(self) -> None:
        # 添加仓库
        self.warehouses = {
            "树脂工作站堆栈": Hydrogel_warehouse_1x5x1("树脂工作站堆栈"),
            "反应工站1": Station_1_warehouse_4x2x1("反应工站1"),
            # "反应工站2": Station_2_warehouse_1x1x1("反应工站2"),
            # "反应工站3": Station_3_warehouse_1x1x1("反应工站3")
        }
        # warehouse 的位置
        # 根据前端显示位置转换计算（Deck尺寸: 1217x1580mm, 前端显示: 688x895px）
        # 缩放比例: x=1.769, y=1.766
        # 前端坐标 -> 实际坐标: x' = x * 1.769, y' = y * 1.766
        self.warehouse_locations = {
            "树脂工作站堆栈": Coordinate(083.416, 399.630, 0.0),     # 前端: 9x623
            "反应工站1": Coordinate(462.610, 1158.7, 0.0),         # 前端: 474x139
            # "反应工站2": Coordinate(850.9, 706.4, 0.0),         # 前端: 481x400
            # "反应工站3": Coordinate(842.0, 245.5, 0.0)         # 前端: 476x656
        }

        for warehouse_name, warehouse in self.warehouses.items():
            self.assign_child_resource(warehouse, location=self.warehouse_locations[warehouse_name])

    def assign_child_resource(
        self,
        resource: Resource,
        location: Optional[Coordinate],
        reassign: bool = True,
      ):
          super().assign_child_resource(resource, location, reassign)
          self.warehouses[resource.name] = resource
          self.warehouse_locations[resource.name] = location
