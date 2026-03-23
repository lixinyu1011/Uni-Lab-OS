from typing import List, Optional

from pylabrobot.resources import Resource as PLRResource
from pylabrobot.resources.coordinate import Coordinate
from unilabos.utils.log import logger

from .reagents import ReagentBottle


class ReagentRack(PLRResource):
    """
    试剂架类
    """
    def __init__(self, name: str, size_x: float, size_y: float, size_z: float, 
                 num_rows: int = 3, num_cols: int = 4, category: str = "reagent_rack"):
        super().__init__(name=name, size_x=size_x, size_y=size_y, size_z=size_z, category=category)
        self.num_rows = num_rows
        self.num_cols = num_cols
        self._slots = self._create_slots()
        
    def _create_slots(self) -> List[List[Optional[ReagentBottle]]]:
        """
        创建试剂架槽位
        
        Returns:
            List[List[Optional[ReagentBottle]]]: 槽位二维列表
        """
        return [[None for _ in range(self.num_cols)] for _ in range(self.num_rows)]
        
    def assign_bottle(self, bottle: ReagentBottle, row: int, col: int) -> bool:
        """
        分配试剂瓶到指定槽位
        
        Args:
            bottle: 试剂瓶对象
            row: 行号
            col: 列号
            
        Returns:
            bool: 操作成功返回True，否则返回False
        """
        if not (0 <= row < self.num_rows and 0 <= col < self.num_cols):
            logger.error(f"无效的槽位位置: ({row}, {col}), 试剂架大小: {self.num_rows}x{self.num_cols}")
            return False
        
        if self._slots[row][col] is not None:
            logger.error(f"槽位 ({row}, {col}) 已被占用")
            return False
        
        self._slots[row][col] = bottle
        self.assign_child_resource(bottle, location=Coordinate(0, 0, 0))
        return True
        
    def remove_bottle(self, row: int, col: int) -> Optional[ReagentBottle]:
        """
        从指定槽位移除试剂瓶
        
        Args:
            row: 行号
            col: 列号
            
        Returns:
            Optional[ReagentBottle]: 移除的试剂瓶对象，若槽位为空则返回None
        """
        if not (0 <= row < self.num_rows and 0 <= col < self.num_cols):
            logger.error(f"无效的槽位位置: ({row}, {col}), 试剂架大小: {self.num_rows}x{self.num_cols}")
            return None
        
        bottle = self._slots[row][col]
        if bottle is not None:
            self.unassign_child_resource(bottle)
            self._slots[row][col] = None
        
        return bottle
        
    def get_bottle(self, reagent_id: int) -> Optional[ReagentBottle]:
        """
        根据试剂ID查找试剂瓶
        
        Args:
            reagent_id: 试剂ID
            
        Returns:
            Optional[ReagentBottle]: 找到的试剂瓶对象，若未找到则返回None
        """
        for row in self._slots:
            for bottle in row:
                if bottle is not None and bottle._unilabos_state.reagent_id == reagent_id:
                    return bottle
        
        return None
        
    def get_bottle_by_position(self, row: int, col: int) -> Optional[ReagentBottle]:
        """
        根据位置查找试剂瓶
        
        Args:
            row: 行号
            col: 列号
            
        Returns:
            Optional[ReagentBottle]: 找到的试剂瓶对象，若未找到则返回None
        """
        if not (0 <= row < self.num_rows and 0 <= col < self.num_cols):
            logger.error(f"无效的槽位位置: ({row}, {col}), 试剂架大小: {self.num_rows}x{self.num_cols}")
            return None
        
        return self._slots[row][col]