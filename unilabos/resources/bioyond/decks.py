from pylabrobot.resources import Deck, Coordinate

from unilabos.resources.bioyond.warehouses import bioyond_warehouse_1x4x4, bioyond_warehouse_1x4x2, bioyond_warehouse_liquid_and_lid_handling


class BIOYOND_PolymerReactionStation_Deck(Deck):
    def __init__(self, name: str = "PolymerReactionStation_Deck") -> None:
        super().__init__(name=name, size_x=2700.0, size_y=1080.0, size_z=1500.0)

    def setup(self) -> None:
        # 添加仓库
        self.warehouses = {
            "io_warehouse_left": bioyond_warehouse_1x4x4("io_warehouse_left"),
            "io_warehouse_right": bioyond_warehouse_1x4x4("io_warehouse_right"),
            "liquid_and_lid_handling": bioyond_warehouse_liquid_and_lid_handling("liquid_and_lid_handling"),
        }
        self.warehouse_locations = {
            "io_warehouse_left": Coordinate(0.0, 650.0, 0.0),
            "io_warehouse_right": Coordinate(2550.0, 650.0, 0.0),
            "liquid_and_lid_handling": Coordinate(800.0, 475.0, 0.0),
        }

        for warehouse_name, warehouse in self.warehouses.items():
            self.assign_child_resource(warehouse, location=self.warehouse_locations[warehouse_name])


class BIOYOND_PolymerPreparationStation_Deck(Deck):
    def __init__(self, name: str = "PolymerPreparationStation_Deck") -> None:
        super().__init__(name=name, size_x=2700.0, size_y=1080.0, size_z=1500.0)
        self.warehouses = {}

    def setup(self) -> None:
        # 添加仓库
        self.warehouses = {
            "io_warehouse_left": bioyond_warehouse_1x4x4("io_warehouse_left"),
            "io_warehouse_right": bioyond_warehouse_1x4x4("io_warehouse_right"),
            "solutions": bioyond_warehouse_1x4x2("warehouse_solutions"),
            "liquid_and_lid_handling": bioyond_warehouse_liquid_and_lid_handling("warehouse_liquid_and_lid_handling"),
        }
        self.warehouse_locations = {
            "io_warehouse_left": Coordinate(0.0, 650.0, 0.0),
            "io_warehouse_right": Coordinate(2550.0, 650.0, 0.0),
            "solutions": Coordinate(1915.0, 900.0, 0.0),
            "liquid_and_lid_handling": Coordinate(1330.0, 490.0, 0.0),
        }

        for warehouse_name, warehouse in self.warehouses.items():
            self.assign_child_resource(warehouse, location=self.warehouse_locations[warehouse_name])
