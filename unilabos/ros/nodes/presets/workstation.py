import collections
from typing import Union, Dict, Any, Optional

from unilabos_msgs.msg import Resource
from pylabrobot.resources import Resource as PLRResource, Plate, TipRack, Coordinate
from unilabos.ros.nodes.presets.protocol_node import ROS2ProtocolNode
from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker


class WorkStationContainer(Plate, TipRack):
    """
    WorkStation 专用 Container 类，继承自 Plate和TipRack
    注意这个物料必须通过plr_additional_res_reg.py注册到edge，才能正常序列化
    """

    def __init__(self, name: str, size_x: float, size_y: float, size_z: float, category: str, ordering: collections.OrderedDict, model: Optional[str] = None,):
        """
        这里的初始化入参要和plr的保持一致
        """
        super().__init__(name, size_x, size_y, size_z, category=category, ordering=ordering, model=model)
        self._unilabos_state = {}  # 必须有此行，自己的类描述的是物料的

    def load_state(self, state: Dict[str, Any]) -> None:
        """从给定的状态加载工作台信息。"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        data = super().serialize_state()
        data.update(self._unilabos_state)  # Container自身的信息，云端物料将保存这一data，本地也通过这里的data进行读写，当前类用来表示这个物料的长宽高大小的属性，而data（state用来表示物料的内容，细节等）
        return data


def get_workstation_plate_resource(name: str) -> PLRResource:  # 要给定一个返回plr的方法
    """
    用于获取一些模板，例如返回一个带有特定信息/子物料的 Plate，这里需要到注册表注册，例如unilabos/registry/resources/organic/workstation.yaml
    可以直接运行该函数或者利用注册表补全机制，来检查是否资源出错
    :param name: 资源名称
    :return: Resource对象
    """
    plate = WorkStationContainer(name, size_x=50, size_y=50, size_z=10, category="plate", ordering=collections.OrderedDict())
    tip_rack = WorkStationContainer("tip_rack_inside_plate", size_x=50, size_y=50, size_z=10, category="tip_rack", ordering=collections.OrderedDict())
    plate.assign_child_resource(tip_rack, Coordinate.zero())
    return plate


class WorkStationExample(ROS2ProtocolNode):
    def __init__(self,
                 # 你可以在这里增加任意的参数，对应启动json填写相应的参数内容
                 device_id: str,
                 children: dict,
                 protocol_type: Union[str, list[str]],
                 resource_tracker: DeviceNodeResourceTracker
                 ):
        super().__init__(device_id, children, protocol_type, resource_tracker)

    def create_resource(
        self,
        resource_tracker: DeviceNodeResourceTracker,
        resources: list[Resource],
        bind_parent_id: str,
        bind_location: dict[str, float],
        liquid_input_slot: list[int],
        liquid_type: list[str],
        liquid_volume: list[int],
        slot_on_deck: int,
    ) -> Dict[str, Any]:
        return {  # edge侧返回给前端的创建物料的结果。云端调用初始化瓶子等。执行该函数时，物料已经上报给云端，一般不需要继承使用

        }

    def transfer_bottle(self, tip_rack: PLRResource, base_plate: PLRResource):  # 使用自定义物料的举例
        """
        将tip_rack assign给base_plate，两个入参都得是PLRResource，unilabos会代替当前物料操作，自动刷新他们的父子关系等状态
        """
        pass

    def trigger_resource_update(self, from_plate: PLRResource, to_base_plate: PLRResource):
        """
        有些时候物料发生了子设备的迁移，一般对该设备的最大一级的物料进行操作，例如要将A物料搬移到B物料上，他们不共同拥有一个物料
        该步骤操作结束后，会主动刷新from_plate的父物料，与to_base_plate的父物料（如没有则刷新自身）

        """
        to_base_plate.assign_child_resource(from_plate, Coordinate.zero())
        pass

