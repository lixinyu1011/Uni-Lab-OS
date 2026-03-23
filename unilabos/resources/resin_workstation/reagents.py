from dataclasses import dataclass
from typing import Any, Dict, Optional

from pylabrobot.resources import Resource as PLRResource
from unilabos.utils.log import logger


@dataclass
class ReagentState:
    """
    试剂状态类
    """
    reagent_id: int  # 试剂编号
    name: str  # 试剂名称
    volume: float  # 当前体积
    max_volume: float  # 最大体积
    concentration: str  # 浓度
    category: str = "reaction"  # 类别：reaction/post_process
    status: str = "idle"  # 状态：idle, in_use, empty


class ReagentBottle(PLRResource):
    """
    试剂瓶类
    """
    def __init__(self, name: str, size_x: float, size_y: float, size_z: float,
                 reagent_state: ReagentState = {}, category: str = "reagent_bottle"):
        super().__init__(name=name, size_x=size_x, size_y=size_y, size_z=size_z, category=category)
        self._unilabos_state = reagent_state
        
    def add_reagent(self, volume: float) -> bool:
        """
        添加试剂
        
        Args:
            volume: 要添加的体积
            
        Returns:
            bool: 操作成功返回True，否则返回False
        """
        if self._unilabos_state.volume + volume > self._unilabos_state.max_volume:
            logger.error(f"试剂瓶 {self.name} 容量不足，当前体积: {self._unilabos_state.volume}, 要添加: {volume}, 最大容量: {self._unilabos_state.max_volume}")
            return False
        
        self._unilabos_state.volume += volume
        self._unilabos_state.status = "in_use"
        return True
        
    def remove_reagent(self, volume: float) -> bool:
        """
        移除试剂
        
        Args:
            volume: 要移除的体积
            
        Returns:
            bool: 操作成功返回True，否则返回False
        """
        if self._unilabos_state.volume < volume:
            logger.error(f"试剂瓶 {self.name} 体积不足，当前体积: {self._unilabos_state.volume}, 要移除: {volume}")
            return False
        
        self._unilabos_state.volume -= volume
        self._unilabos_state.status = "in_use"
        
        # 检查是否为空
        if self._unilabos_state.volume <= 0:
            self._unilabos_state.volume = 0
            self._unilabos_state.status = "empty"
        
        return True
        
    def serialize_state(self) -> Dict[str, Any]:
        """
        序列化状态
        
        Returns:
            Dict[str, Any]: 序列化后的状态字典
        """
        return {
            "reagent_id": self._unilabos_state.reagent_id,
            "name": self._unilabos_state.name,
            "volume": self._unilabos_state.volume,
            "max_volume": self._unilabos_state.max_volume,
            "concentration": self._unilabos_state.concentration,
            "category": self._unilabos_state.category,
            "status": self._unilabos_state.status
        }
        
    def load_state(self, state: Dict[str, Any]) -> None:
        """
        加载状态
        
        Args:
            state: 状态字典
        """
        self._unilabos_state.reagent_id = state.get("reagent_id", self._unilabos_state.reagent_id)
        self._unilabos_state.name = state.get("name", self._unilabos_state.name)
        self._unilabos_state.volume = state.get("volume", self._unilabos_state.volume)
        self._unilabos_state.max_volume = state.get("max_volume", self._unilabos_state.max_volume)
        self._unilabos_state.concentration = state.get("concentration", self._unilabos_state.concentration)
        self._unilabos_state.category = state.get("category", self._unilabos_state.category)
        self._unilabos_state.status = state.get("status", self._unilabos_state.status)