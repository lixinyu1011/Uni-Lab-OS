from pylabrobot.resources import Deck
from pylabrobot.resources.coordinate import Coordinate

from .racks import ReagentRack


class ResinWorkstationDeck(Deck):
    """
    树脂工作站台面类
    """
    def __init__(self, name: str = "resin_workstation_deck", 
                 size_x: float = 1620.0, size_y: float = 1270.0, size_z: float = 500.0):
        super().__init__(name=name, size_x=size_x, size_y=size_y, size_z=size_z)
        # 初始化反应试剂架和后处理试剂架
        self.reaction_reagent_rack = None
        self.post_process_reagent_rack = None
        
    def initialize_reagent_racks(self):
        """
        初始化试剂架
        """
        # 创建反应试剂架
        self.reaction_reagent_rack = ReagentRack(
            name="reaction_reagent_rack",
            size_x=210.0,
            size_y=140.0,
            size_z=100.0,
            num_rows=3,
            num_cols=4
        )
        self.assign_child_resource(self.reaction_reagent_rack, location=Coordinate(0, 0, 0))
        
        # 创建后处理试剂架
        self.post_process_reagent_rack = ReagentRack(
            name="post_process_reagent_rack",
            size_x=210.0,
            size_y=140.0,
            size_z=100.0,
            num_rows=3,
            num_cols=4
        )
        self.assign_child_resource(self.post_process_reagent_rack, location=Coordinate(0, 0, 0))