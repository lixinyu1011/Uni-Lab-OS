from __future__ import annotations
"""
样品制备工作站台面配置
Sample Preparation Station Deck Configuration

基于自动化液体处理工作站布局图创建的实际配置
包含粉末堆栈、溶液堆栈、试剂堆栈等
支持索引赋值操作，如 rack[0] = bottle
"""

import json
from typing import Optional, List
from pylabrobot.resources import Coordinate, Resource
from pylabrobot.resources.carrier import Carrier, PlateHolder, ResourceHolder
from pylabrobot.resources.deck import Deck

"""
纽扣电池组装工作站物料类定义
Button Battery Assembly Station Resource Classes
"""


from collections import OrderedDict
from typing import Any, Dict, List, Optional, TypedDict, Union, cast

from pylabrobot.resources.coordinate import Coordinate
from pylabrobot.resources.container import Container
from pylabrobot.resources.deck import Deck
from pylabrobot.resources.itemized_resource import ItemizedResource
from pylabrobot.resources.resource import Resource
from pylabrobot.resources.resource_stack import ResourceStack
from pylabrobot.resources.tip_rack import TipRack, TipSpot
from pylabrobot.resources.trash import Trash
from pylabrobot.resources.utils import create_ordered_items_2d


class BottleState(TypedDict):
    diameter: float = 11.2
    height: float = 10.5
    electrolyte_name: str
    electrolyte_volume: float
    max_volume: float

class Bottle(Resource):
    """瓶子类 - 容纳电解液"""

    def __init__(
        self,
        name: str,
        category: str = "bottle",
    ):
        """初始化瓶子

        Args:
            name: 瓶子名称
            diameter: 直径 (mm)
            height: 高度 (mm)
            max_volume: 最大体积 (μL)
            barcode: 二维码
            category: 类别
            model: 型号
        """
        super().__init__(
            name=name,
            size_x=13,
            size_y=12,
            size_z=11,
            category=category,
        )
        self._unilabos_state: BottleState = BottleState()
        self._unilabos_state["electrolyte_volume"] = 20.2
        self._unilabos_state["max_volume"] = 50.0
    def aspirate_electrolyte(self, volume: float) -> bool:
        current_volume = self._unilabos_state["electrolyte_volume"]
        assert current_volume > volume, f"Cannot aspirate {volume}μL, only {current_volume}μL available."
        self._unilabos_state["electrolyte_volume"] -= volume
        return True

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data


class BottleRackState(TypedDict):
    """     bottle_diameter: 瓶子直径 (mm)
            bottle_height: 瓶子高度 (mm)
            position_spacing: 位置间距 (mm)"""
    bottle_diameter: float
    bottle_height: float
    name_to_index: dict

class BottleRack(Resource):
    """瓶架类 - 12个瓶子位置"""
    children: List[Bottle] = []

    def __init__(
        self,
        name: str,
        size_x: float = 18.0,
        size_y: float = 12.0,
        size_z: float = 15.0 ,
        category: str = "bottle_rack",
        model: Optional[str] = None,
    ):
        """初始化瓶架

        Args:
            name: 瓶架名称
            size_x: 长度 (mm)
            size_y: 宽度 (mm)
            size_z: 高度 (mm)
            category: 类别
            model: 型号
        """
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            category=category,
            model=model,
        )
        self._unilabos_state: BottleRackState = BottleRackState()

        # TODO: 添加瓶位坐标映射
        self.index_to_pos = {
            0: Coordinate.zero(),
            1: Coordinate(x=1, y=2, z=3)  # 添加
        }
        self.name_to_index = {}
        self.name_to_pos = {}

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data

    def assign_child_resource(self, resource: Bottle, location=Coordinate.zero(), reassign = True):
        assert len(self.children) <= 12, "瓶架已满，无法添加更多瓶子"
        index = len(self.children)
        location = Coordinate(x=20 + (index % 4) * 15, y=20 + (index // 4) * 15, z=0)
        self.name_to_pos[resource.name] = location
        self.name_to_index[resource.name] = index
        return super().assign_child_resource(resource, location, reassign)
    
    def assign_child_resource_by_index(self, resource: Bottle, index: int):
        assert 0 <= index < 12, "无效的瓶子索引"
        self.name_to_index[resource.name] = index
        location = self.index_to_pos[index]
        return super().assign_child_resource(resource, location)

    def unassign_child_resource(self, resource: Bottle):
        super().unassign_child_resource(resource)
        self.index_to_pos.pop(self.name_to_index.pop(resource.name, None), None)

    # def serialize(self):
    #     self.children.sort(key=lambda x: self.name_to_index.get(x.name, 0))
    #     return super().serialize()



# TODO:这个移液枪架子看一下从哪继承
class TipBox64State(TypedDict):
    """电池状态字典"""
    tip_diameter: float = 5.0
    tip_length: float = 50.0
    with_tips: bool = True

class TipBox64(TipRack):
    """64孔枪头盒类"""

    children: List[TipSpot] = []

    def __init__(
        self,
        name: str,
        size_x: float = 127.8,
        size_y: float = 85.5,
        size_z: float = 60.0,
        category: str = "tip_box_64",
        model: Optional[str] = None,
    ):
        """初始化64孔枪头盒

        Args:
            name: 枪头盒名称
            size_x: 长度 (mm)
            size_y: 宽度 (mm)
            size_z: 高度 (mm)
            tip_diameter: 枪头直径 (mm)
            tip_length: 枪头长度 (mm)
            category: 类别
            model: 型号
            with_tips: 是否带枪头
        """
        from pylabrobot.resources.tip import Tip

        # 创建8x8=64个枪头位
        def make_tip():
            return Tip(
                has_filter=False,
                total_tip_length= 20.0,
                maximal_volume=1000,  # 1mL
                fitting_depth=8.0,
            )

        tip_spots = create_ordered_items_2d(
            klass=TipSpot,
            num_items_x=8,
            num_items_y=8,
            dx=8.0,
            dy=8.0,
            dz=0.0,
            item_dx=9.0,
            item_dy=9.0,
            size_x=10.0,
            size_y=10.0,
            size_z=0.0,
            make_tip=make_tip,
        )

        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            ordered_items=tip_spots,
            category=category,
            model=model,
            with_tips=True,
        )
        self._unilabos_state: TipBox64State = TipBox64State()
    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data
    

class BlockState(TypedDict):
    diameter: float = 10.5
    height: float = 12.5

class Block(Resource):
    """块类"""

    def __init__(
        self,
        name: str,
        category: str = "bottle",
    ):


        super().__init__(
            name=name,
            size_x=22,
            size_y=21,
            size_z=20,
            category=category,
        )
        self._unilabos_state: BlockState = BlockState()
    

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data
    

class StackState(TypedDict):
    location: Coordinate = Coordinate(x=100, y=100, z=0)

class Stack(Resource):
    """堆栈类"""

    def __init__(
        self,
        name: str,
        location: Coordinate,
        category: str = "bottle",
    ):


        super().__init__(
            name=name,
            size_x=50,
            size_y=50,
            size_z=30,
            category=category,
        )
        self._unilabos_state: StackState = StackState()
        self.location: Coordinate = location

    def load_state(self, state: Dict[str, Any]) -> None:
        """格式不变"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        """格式不变"""
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data


class StackCarrier4x4(TypedDict):
    diameter: float = 10.5
    height: float = 12.5

class StackCarrier4x4(Carrier[ResourceHolder]):
    """4x4x1堆栈载体类 - 可容纳16个板位的载体（4层x4行x1列）"""

    def __init__(
        self,
        name: str,
        size_x: float = 140.0,
        size_y: float = 360.0,
        size_z: float = 600.0,
        layer_spacing: float = 60.0,
        position_spacing_x: float = 140.0,
        position_spacing_y: float = 95.0,
        category: str = "stack_carrier_4x4x1",
        model: Optional[str] = None,
    ):
        # 创建16个板架位 (4层 x 4位置)
        sites = {}
        site_index = 0

        for layer in range(4):  # 4层
            for row in range(4):  # 4行
                for col in range(1):  # 1列 (每层4x1=4个位置)
                    holder = ResourceHolder(
                        name="holder_L{}_R{}_C{}".format(layer, row, col),
                        size_x=130.0,  # 板架位尺寸
                        size_y=90.0,
                        size_z=50.0,
                        # pedestal_size_z=5.0,
                    )

                    # 计算位置
                    x = 10.0 + col * position_spacing_x
                    y = 10.0 + row * position_spacing_y
                    z = 10.0 + layer * layer_spacing

                    holder.location = Coordinate(x, y, z)
                    sites[site_index] = holder
                    site_index += 1

        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            sites=sites,
            category=category,
            model=model,
        )

        self.layer_spacing = layer_spacing
        self.position_spacing_x = position_spacing_x
        self.position_spacing_y = position_spacing_y
        self._unilabos_state: StackCarrier4x4 = StackCarrier4x4()

    def get_site_by_layer_position(self, layer: int, row: int, col: int) -> PlateHolder:
        if not (0 <= layer < 4 and 0 <= row < 4 and 0 <= col < 1):
            raise ValueError("无效的位置: layer={}, row={}, col={}".format(layer, row, col))

        site_index = layer * 4 + row * 1 + col
        return self.sites[site_index]

    def add_rack_to_position(self, layer: int, row: int, col: int, rack) -> None:
        site = self.get_site_by_layer_position(layer, row, col)
        site.assign_child_resource(rack)

    def get_rack_at_position(self, layer: int, row: int, col: int):
        site = self.get_site_by_layer_position(layer, row, col)
        return site.resource


class PreparationDeck(Deck):
    """样品制备工作站台面类"""

    def __init__(
        self,
        name: str = "preparation_deck",
        size_x: float = 2000.0,
        size_y: float = 1500.0,
        size_z: float = 800.0,
        origin: Coordinate = Coordinate(0, 0, 0),
        category: str = "preparation_deck",
    ):
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            origin=origin,
            category=category,
        )

    def setup_default_layout(self) -> None:
        """设置默认布局"""
        print("正在设置样品制备工作站默认布局...")

        # 创建粉末堆栈载体
        powder_stack = StackCarrier4x4(
            name="powder_stack_carrier",
            category="powder_stack_carrier",
        )
        self.assign_child_resource(powder_stack, location=Coordinate(100, 200, 0))
        print("✓ 创建粉末堆栈载体: {}".format(powder_stack.name))

        # 创建溶液堆栈载体
        solution_stack = StackCarrier4x4(
            name="solution_stack_carrier",
            category="solution_stack_carrier",
        )
        self.assign_child_resource(solution_stack, location = Coordinate(800, 200, 0))
        print("✓ 创建溶液堆栈载体: {}".format(solution_stack.name))

        # 创建试剂堆栈载体
        reagent_stack = StackCarrier4x4(
            name="reagent_stack_carrier",
            category="reagent_stack_carrier",
        )
        self.assign_child_resource(reagent_stack, location=Coordinate(1500, 200, 0))
        print("✓ 创建试剂堆栈载体: {}".format(reagent_stack.name))


if __name__ == "__main__":
    Stack1 = Stack(name="Stack1", location=Coordinate(100, 100, 0))
    Stack2 = Stack(name="Stack2", location=Coordinate(200, 100, 0))
    Stack3 = Stack(name="Stack3", location=Coordinate(300, 100, 0))

    print(Stack1.location)
