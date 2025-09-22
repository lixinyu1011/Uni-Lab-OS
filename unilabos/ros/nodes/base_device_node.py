import copy
import io
import json
import threading
import time
import traceback
import uuid
from typing import get_type_hints, TypeVar, Generic, Dict, Any, Type, TypedDict, Optional, List, Union

from concurrent.futures import ThreadPoolExecutor
import asyncio

import rclpy
import yaml
from msgcenterpy import ROS2MessageInstance
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.client import Client
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
from unilabos_msgs.action import SendCmd
from unilabos_msgs.srv._serial_command import SerialCommand_Request, SerialCommand_Response

from unilabos.resources.container import RegularContainer
from unilabos.resources.graphio import (
    convert_resources_to_type,
    convert_resources_from_type,
    resource_ulab_to_plr,
    initialize_resources,
    dict_to_tree,
    resource_plr_to_ulab,
    tree_to_list,
)
from unilabos.resources.plr_additional_res_reg import register
from unilabos.ros.msgs.message_converter import (
    convert_to_ros_msg,
    convert_from_ros_msg,
    convert_from_ros_msg_with_mapping,
    convert_to_ros_msg_with_mapping,
)
from unilabos_msgs.srv import (
    ResourceAdd,
    ResourceGet,
    ResourceDelete,
    ResourceUpdate,
    ResourceList,
    SerialCommand,
)  # type: ignore
from unilabos_msgs.msg import Resource  # type: ignore

from unilabos.ros.nodes.resource_tracker import DeviceNodeResourceTracker
from unilabos.ros.x.rclpyx import get_event_loop
from unilabos.ros.utils.driver_creator import WorkstationNodeCreator, PyLabRobotCreator, DeviceClassCreator
from unilabos.utils.async_util import run_async_func
from unilabos.utils.log import info, debug, warning, error, critical, logger, trace
from unilabos.utils.type_check import get_type_class, TypeEncoder, serialize_result_info, get_result_info_str

T = TypeVar("T")


# 在线设备注册表
registered_devices: Dict[str, "DeviceInfoType"] = {}


# 实现同时记录自定义日志和ROS2日志的适配器
class ROSLoggerAdapter:
    """同时向自定义日志和ROS2日志发送消息的适配器"""

    @property
    def identifier(self):
        return f"{self.namespace}"

    def __init__(self, ros_logger, namespace):
        """
        初始化日志适配器

        Args:
            ros_logger: ROS2日志记录器
            namespace: 命名空间
        """
        self.ros_logger = ros_logger
        self.namespace = namespace
        self.level_2_logger_func = {
            "info": info,
            "debug": debug,
            "trace": trace,
            "warning": warning,
            "error": error,
            "critical": critical,
        }

    def _log(self, level, msg, *args, **kwargs):
        """实际执行日志记录的内部方法"""
        # 添加前缀，使日志更易识别
        msg = f"[{self.identifier}] {msg}"
        # 向ROS2日志发送消息（标准库logging不支持stack_level参数）
        ros_log_func = getattr(self.ros_logger, "debug")  # 默认发送debug，这样不会显示在控制台
        ros_log_func(msg)
        self.level_2_logger_func[level](msg, *args, stack_level=1, **kwargs)

    def trace(self, msg, *args, **kwargs):
        """记录TRACE级别日志"""
        self._log("trace", msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        """记录DEBUG级别日志"""
        self._log("debug", msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        """记录INFO级别日志"""
        self._log("info", msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        """记录WARNING级别日志"""
        self._log("warning", msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        """记录ERROR级别日志"""
        self._log("error", msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        """记录CRITICAL级别日志"""
        self._log("critical", msg, *args, **kwargs)


def init_wrapper(
    self,
    device_id: str,
    driver_class: type[T],
    device_config: Dict[str, Any],
    status_types: Dict[str, Any],
    action_value_mappings: Dict[str, Any],
    hardware_interface: Dict[str, Any],
    print_publish: bool,
    children: Optional[list] = None,
    driver_params: Optional[Dict[str, Any]] = None,
    driver_is_ros: bool = False,
    *args,
    **kwargs,
):
    """初始化设备节点的包装函数，和ROS2DeviceNode初始化保持一致"""
    if driver_params is None:
        driver_params = kwargs.copy()
    if children is None:
        children = []
    kwargs["device_id"] = device_id
    kwargs["driver_class"] = driver_class
    kwargs["device_config"] = device_config
    kwargs["driver_params"] = driver_params
    kwargs["status_types"] = status_types
    kwargs["action_value_mappings"] = action_value_mappings
    kwargs["hardware_interface"] = hardware_interface
    kwargs["children"] = children
    kwargs["print_publish"] = print_publish
    kwargs["driver_is_ros"] = driver_is_ros
    super(type(self), self).__init__(*args, **kwargs)


class PropertyPublisher:
    def __init__(
        self,
        node: "BaseROS2DeviceNode",
        name: str,
        get_method,
        msg_type,
        initial_period: float = 5.0,
        print_publish=True,
    ):
        self.node = node
        self.name = name
        self.msg_type = msg_type
        self.get_method = get_method
        self.timer_period = initial_period
        self.print_publish = print_publish

        self._value = None
        try:
            self.publisher_ = node.create_publisher(msg_type, f"{name}", 10)
        except AttributeError as ex:
            self.node.lab_logger().error(f"创建发布者 {name} 失败，可能由于注册表有误，类型: {msg_type}，错误: {ex}\n{traceback.format_exc()}")
        self.timer = node.create_timer(self.timer_period, self.publish_property)
        self.__loop = get_event_loop()
        str_msg_type = str(msg_type)[8:-2]
        self.node.lab_logger().trace(f"发布属性: {name}, 类型: {str_msg_type}, 周期: {initial_period}秒")

    def get_property(self):
        if asyncio.iscoroutinefunction(self.get_method):
            # 如果是异步函数，运行事件循环并等待结果
            self.node.lab_logger().trace(f"【PropertyPublisher.get_property】获取异步属性: {self.name}")
            loop = self.__loop
            if loop:
                future = asyncio.run_coroutine_threadsafe(self.get_method(), loop)
                self._value = future.result()
                return self._value
            else:
                self.node.lab_logger().error(f"【PropertyPublisher.get_property】事件循环未初始化")
                return None
        else:
            # 如果是同步函数，直接调用并返回结果
            self.node.lab_logger().trace(f"【PropertyPublisher.get_property】获取同步属性: {self.name}")
            self._value = self.get_method()
            return self._value

    async def get_property_async(self):
        try:
            # 获取异步属性值
            self.node.lab_logger().trace(f"【PropertyPublisher.get_property_async】异步获取属性: {self.name}")
            self._value = await self.get_method()
        except Exception as e:
            self.node.lab_logger().error(f"【PropertyPublisher.get_property_async】获取异步属性出错: {str(e)}")

    def publish_property(self):
        try:
            self.node.lab_logger().trace(f"【PropertyPublisher.publish_property】开始发布属性: {self.name}")
            value = self.get_property()
            if self.print_publish:
                self.node.lab_logger().trace(f"【PropertyPublisher.publish_property】发布 {self.msg_type}: {value}")
            if value is not None:
                msg = convert_to_ros_msg(self.msg_type, value)
                self.publisher_.publish(msg)
                self.node.lab_logger().trace(f"【PropertyPublisher.publish_property】属性 {self.name} 发布成功")
        except Exception as e:
            self.node.lab_logger().error(f"【PropertyPublisher.publish_property】发布属性 {self.publisher_.topic} 出错: {str(e)}\n{traceback.format_exc()}")

    def change_frequency(self, period):
        # 动态改变定时器频率
        self.timer_period = period
        self.node.get_logger().info(
            f"【PropertyPublisher.change_frequency】修改 {self.name} 定时器周期为: {self.timer_period} 秒"
        )

        # 重置定时器
        self.timer.cancel()
        self.timer = self.node.create_timer(self.timer_period, self.publish_property)


class BaseROS2DeviceNode(Node, Generic[T]):
    """
    ROS2设备节点基类

    这个类提供了ROS2设备节点的基本功能，包括属性发布、动作服务等。
    通过泛型参数T来指定具体的设备类型。
    """

    @property
    def identifier(self):
        return f"{self.namespace}/{self.device_id}"

    node_name: str
    namespace: str
    # TODO 要删除，添加时间相关的属性，避免动态添加属性的警告
    time_spent = 0.0
    time_remaining = 0.0
    create_action_server = True

    def __init__(
        self,
        driver_instance: T,
        device_id: str,
        status_types: Dict[str, Any],
        action_value_mappings: Dict[str, Any],
        hardware_interface: Dict[str, Any],
        print_publish=True,
        resource_tracker: Optional["DeviceNodeResourceTracker"] = None,
    ):
        """
        初始化ROS2设备节点

        Args:
            driver_instance: 设备实例
            device_id: 设备标识符
            status_types: 需要发布的状态和传感器信息
            action_value_mappings: 设备动作
            hardware_interface: 硬件接口配置
            print_publish: 是否打印发布信息
        """
        self.driver_instance = driver_instance
        self.device_id = device_id
        self.uuid = str(uuid.uuid4())
        self.publish_high_frequency = False
        self.callback_group = ReentrantCallbackGroup()
        self.resource_tracker = resource_tracker

        # 初始化ROS节点
        self.node_name = f'{device_id.split("/")[-1]}'
        self.namespace = f"/devices/{device_id}"
        Node.__init__(self, self.node_name, namespace=self.namespace)  # type: ignore
        if self.resource_tracker is None:
            self.lab_logger().critical("资源跟踪器未初始化，请检查")

        # 创建自定义日志记录器
        self._lab_logger = ROSLoggerAdapter(self.get_logger(), self.namespace)

        self._action_servers: Dict[str, ActionServer] = {}
        self._property_publishers = {}
        self._status_types = status_types
        self._action_value_mappings = action_value_mappings
        self._hardware_interface = hardware_interface
        self._print_publish = print_publish

        # 创建属性发布者
        for attr_name, msg_type in self._status_types.items():
            if isinstance(attr_name, (int, float)):
                if "param" in msg_type.keys():
                    pass
                else:
                    for k, v in msg_type.items():
                        self.create_ros_publisher(k, v, initial_period=5.0)
            else:
                self.create_ros_publisher(attr_name, msg_type)

        # 创建动作服务
        if self.create_action_server:
            for action_name, action_value_mapping in self._action_value_mappings.items():
                if action_name.startswith("auto-") or str(action_value_mapping.get("type", "")).startswith("UniLabJsonCommand"):
                    continue
                self.create_ros_action_server(action_name, action_value_mapping)

        # 创建线程池执行器
        self._executor = ThreadPoolExecutor(
            max_workers=max(len(action_value_mappings), 1), thread_name_prefix=f"ROSDevice{self.device_id}"
        )

        # 创建资源管理客户端
        self._resource_clients: Dict[str, Client] = {
            "resource_add": self.create_client(ResourceAdd, "/resources/add"),
            "resource_get": self.create_client(ResourceGet, "/resources/get"),
            "resource_delete": self.create_client(ResourceDelete, "/resources/delete"),
            "resource_update": self.create_client(ResourceUpdate, "/resources/update"),
            "resource_list": self.create_client(ResourceList, "/resources/list"),
        }

        def query_host_name_cb(req, res):
            self.register_device()
            self.lab_logger().info("Host要求重新注册当前节点")
            res.response = ""
            return res

        async def append_resource(req: SerialCommand_Request, res: SerialCommand_Response):
            # 物料传输到对应的node节点
            rclient = self.create_client(ResourceAdd, "/resources/add")
            rclient.wait_for_service()
            request = ResourceAdd.Request()
            
            command_json = json.loads(req.command)
            namespace = command_json["namespace"]
            bind_parent_id = command_json["bind_parent_id"]
            location = command_json["bind_location"]
            other_calling_param = command_json["other_calling_param"]
            resources = command_json["resource"]
            initialize_full = other_calling_param.pop("initialize_full", False)
            # 用来增加液体
            ADD_LIQUID_TYPE = other_calling_param.pop("ADD_LIQUID_TYPE", [])
            LIQUID_VOLUME = other_calling_param.pop("LIQUID_VOLUME", [])
            LIQUID_INPUT_SLOT = other_calling_param.pop("LIQUID_INPUT_SLOT", [])
            slot = other_calling_param.pop("slot", "-1")
            parent = None
            if slot != "-1":  # slot为负数的时候采用assign方法
                other_calling_param["slot"] = slot
            # 本地拿到这个物料，可能需要先做初始化?
            if isinstance(resources, list):
                if (
                    len(resources) == 1 and isinstance(resources[0], list) and not initialize_full
                ):  # 取消，不存在的情况
                    # 预先initialize过，以整组的形式传入
                    request.resources = [convert_to_ros_msg(Resource, resource_) for resource_ in resources[0]]
                elif initialize_full:
                    resources = initialize_resources(resources)
                    request.resources = [convert_to_ros_msg(Resource, resource) for resource in resources]
                else:
                    request.resources = [convert_to_ros_msg(Resource, resource) for resource in resources]
            else:
                if initialize_full:
                    resources = initialize_resources([resources])
                request.resources = [convert_to_ros_msg(Resource, resources)]
            if len(LIQUID_INPUT_SLOT) and LIQUID_INPUT_SLOT[0] == -1:
                container_instance = request.resources[0]
                container_query_dict: dict = resources
                found_resources = self.resource_tracker.figure_resource({"id": container_query_dict["name"]}, try_mode=True)
                if not len(found_resources):
                    self.resource_tracker.add_resource(container_instance)
                    logger.info(f"添加物料{container_query_dict['name']}到资源跟踪器")
                else:
                    assert len(found_resources) == 1, f"找到多个同名物料: {container_query_dict['name']}, 请检查物料系统"
                    parent = found_resources[0]
                    if isinstance(parent, Resource):
                        regular_container = RegularContainer(parent.id)
                        regular_container.ulr_resource = parent
                        regular_container.ulr_resource_data.update(json.loads(container_instance.data))
                        logger.info(f"更新物料{container_query_dict['name']}的数据{parent.data} ULR")
                    elif isinstance(parent, dict):
                        if "data" not in parent:
                            parent["data"] = {}
                        parent["data"].update(json.loads(container_instance.data))
                        request.resources[0].name = parent["name"]
                        logger.info(f"更新物料{container_query_dict['name']}的数据{parent['data']} dict")
                    else:
                        logger.info(f"更新物料{container_query_dict['name']}出现不支持的数据类型{type(parent)} {parent}")
            response: ResourceAdd.Response = await rclient.call_async(request)
            # 应该先add_resource了
            final_response = {
                "created_resources": [ROS2MessageInstance(i).get_python_dict() for i in request.resources],
                "liquid_input_resources": []
            }
            res.response = json.dumps(final_response)
            # 如果driver自己就有assign的方法，那就使用driver自己的assign方法
            if hasattr(self.driver_instance, "create_resource"):
                create_resource_func = getattr(self.driver_instance, "create_resource")
                try:
                    ret = create_resource_func(
                        resource_tracker=self.resource_tracker,
                        resources=request.resources,
                        bind_parent_id=bind_parent_id,
                        bind_location=location,
                        liquid_input_slot=LIQUID_INPUT_SLOT,
                        liquid_type=ADD_LIQUID_TYPE,
                        liquid_volume=LIQUID_VOLUME,
                        slot_on_deck=slot,
                    )
                    res.response = get_result_info_str("", True, ret)
                except Exception as e:
                    self.lab_logger().error(f"运行设备的create_resource出错：{create_resource_func}\n{traceback.format_exc()}")
                    res.response = get_result_info_str(traceback.format_exc(), False, {})
                return res
            # 接下来该根据bind_parent_id进行assign了，目前只有plr可以进行assign，不然没有办法输入到物料系统中
            if bind_parent_id != self.node_name:
                parent = self.resource_tracker.figure_resource({"name": bind_parent_id})  # 拿到父节点，进行具体assign等操作
            # request.resources = [convert_to_ros_msg(Resource, resources)]

            try:
                from pylabrobot.resources.resource import Resource as ResourcePLR
                from pylabrobot.resources.deck import Deck
                from pylabrobot.resources import Coordinate, OTDeck, Plate

                contain_model = not isinstance(parent, Deck)
                if isinstance(parent, ResourcePLR):
                    # resources.list()
                    resources_tree = dict_to_tree(copy.deepcopy({r["id"]: r for r in resources}))
                    plr_instance = resource_ulab_to_plr(resources_tree[0], contain_model)

                    if isinstance(plr_instance, Plate):
                        empty_liquid_info_in = [(None, 0)] * plr_instance.num_items
                        for liquid_type, liquid_volume, liquid_input_slot in zip(
                            ADD_LIQUID_TYPE, LIQUID_VOLUME, LIQUID_INPUT_SLOT
                        ):
                            empty_liquid_info_in[liquid_input_slot] = (liquid_type, liquid_volume)
                        plr_instance.set_well_liquids(empty_liquid_info_in)
                        input_wells_ulr = [
                            convert_to_ros_msg(Resource, resource_plr_to_ulab(plr_instance.get_well(LIQUID_INPUT_SLOT), with_children=False)) for r in LIQUID_INPUT_SLOT
                        ]
                        final_response["liquid_input_resources"] = [ROS2MessageInstance(i).get_python_dict() for i in input_wells_ulr]
                        res.response = json.dumps(final_response)
                    if isinstance(parent, OTDeck) and "slot" in other_calling_param:
                        other_calling_param["slot"] = int(other_calling_param["slot"])
                        parent.assign_child_at_slot(plr_instance, **other_calling_param)
                    else:
                        _discard_slot = other_calling_param.pop("slot", "-1")
                        parent.assign_child_resource(
                            plr_instance,
                            Coordinate(location["x"], location["y"], location["z"]),
                            **other_calling_param,
                        )
                    
                    request2 = ResourceAdd.Request()
                    request2.resources = [
                        convert_to_ros_msg(Resource, r) for r in tree_to_list([resource_plr_to_ulab(parent)])
                    ]
                    rclient.call(request2)
                # 发送给ResourceMeshManager
                action_client = ActionClient(
                    self,
                    SendCmd,
                    "/devices/resource_mesh_manager/add_resource_mesh",
                    callback_group=self.callback_group,
                )
                goal = SendCmd.Goal()
                goal.command = json.dumps(
                    {
                        "resources": resources,
                        "bind_parent_id": bind_parent_id,
                    }
                )
                future = action_client.send_goal_async(goal)

                def done_cb(*args):
                    self.lab_logger().info(f"向meshmanager发送新增resource完成")

                future.add_done_callback(done_cb)
            except ImportError:
                self.lab_logger().error("Host请求添加物料时，本环境并不存在pylabrobot")
            except Exception as e:
                self.lab_logger().error("Host请求添加物料时出错")
                self.lab_logger().error(traceback.format_exc())
            return res

        # noinspection PyTypeChecker
        self._service_server: Dict[str, Service] = {
            "query_host_name": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/query_host_name",
                query_host_name_cb,
                callback_group=self.callback_group,
            ),
            "append_resource": self.create_service(
                SerialCommand,
                f"/srv{self.namespace}/append_resource",
                append_resource,
                callback_group=self.callback_group,
            ),
        }

        # 向全局在线设备注册表添加设备信息
        self.register_device()
        rclpy.get_global_executor().add_node(self)
        self.lab_logger().debug(f"ROS节点初始化完成")

    async def update_resource(self, resources: List[Any]):
        r = ResourceUpdate.Request()
        unique_resources = []
        for resource in resources:  # resource是list[ResourcePLR]
            # 目前更新资源只支持传入plr的对象，后面要更新convert_resources_from_type函数
            converted_list = convert_resources_from_type([resource], resource_type=[object], is_plr=True)
            unique_resources.extend([convert_to_ros_msg(Resource, converted) for converted in converted_list])
        r.resources = unique_resources
        response = await self._resource_clients["resource_update"].call_async(r)
        self.lab_logger().debug(f"资源更新结果: {response}")

    def register_device(self):
        """向注册表中注册设备信息"""
        topics_info = self._property_publishers.copy()
        actions_info = self._action_servers.copy()
        # 创建设备信息
        device_info = DeviceInfoType(
            id=self.device_id,
            uuid=self.uuid,
            node_name=self.node_name,
            namespace=self.namespace,
            driver_instance=self.driver_instance,
            status_publishers=topics_info,
            actions=actions_info,
            hardware_interface=self._hardware_interface,
            base_node_instance=self,
        )
        # 加入全局注册表
        registered_devices[self.device_id] = device_info
        from unilabos.config.config import BasicConfig
        from unilabos.ros.nodes.presets.host_node import HostNode

        if not BasicConfig.is_host_mode:
            sclient = self.create_client(SerialCommand, "/node_info_update")
            # 启动线程执行发送任务
            threading.Thread(
                target=self.send_slave_node_info,
                args=(sclient,),
                daemon=True,
                name=f"ROSDevice{self.device_id}_send_slave_node_info",
            ).start()
        else:
            host_node = HostNode.get_instance(0)
            if host_node is not None:
                host_node.device_machine_names[self.device_id] = "本地"

    def send_slave_node_info(self, sclient):
        sclient.wait_for_service()
        request = SerialCommand.Request()
        from unilabos.config.config import BasicConfig

        request.command = json.dumps(
            {
                "SYNC_SLAVE_NODE_INFO": {
                    "machine_name": BasicConfig.machine_name,
                    "type": "slave",
                    "edge_device_id": self.device_id,
                }
            },
            ensure_ascii=False,
            cls=TypeEncoder,
        )

        # 发送异步请求并等待结果
        future = sclient.call_async(request)
        response = future.result()

    def lab_logger(self):
        """
        获取实验室自定义日志记录器

        这个日志记录器会同时向ROS2日志和自定义日志发送消息，
        并使用node_name和namespace作为标识。

        Returns:
            日志记录器实例
        """
        return self._lab_logger

    def create_ros_publisher(self, attr_name, msg_type, initial_period=5.0):
        """创建ROS发布者"""

        # 获取属性值的方法
        def get_device_attr():
            try:
                if hasattr(self.driver_instance, f"get_{attr_name}"):
                    return getattr(self.driver_instance, f"get_{attr_name}")()
                else:
                    return getattr(self.driver_instance, attr_name)
            except AttributeError as ex:
                if ex.args[0].startswith(f"AttributeError: '{self.driver_instance.__class__.__name__}' object"):
                    self.lab_logger().error(
                        f"publish error, {str(type(self.driver_instance))[8:-2]} has no attribute '{attr_name}'"
                    )
                else:
                    self.lab_logger().error(
                        f"publish error, when {str(type(self.driver_instance))[8:-2]} getting attribute '{attr_name}'"
                    )
                    self.lab_logger().error(traceback.format_exc())

        self._property_publishers[attr_name] = PropertyPublisher(
            self, attr_name, get_device_attr, msg_type, initial_period, self._print_publish
        )

    def create_ros_action_server(self, action_name, action_value_mapping):
        """创建ROS动作服务器"""
        action_type = action_value_mapping["type"]
        str_action_type = str(action_type)[8:-2]

        self._action_servers[action_name] = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._create_execute_callback(action_name, action_value_mapping),
            callback_group=ReentrantCallbackGroup(),
        )

        self.lab_logger().trace(f"发布动作: {action_name}, 类型: {str_action_type}")

    def get_real_function(self, instance, attr_name):
        if hasattr(instance.__class__, attr_name):
            obj = getattr(instance.__class__, attr_name)
            if isinstance(obj, property):
                return lambda *args, **kwargs: obj.fset(instance, *args, **kwargs), get_type_hints(obj.fset)
            obj = getattr(instance, attr_name)
            return obj, get_type_hints(obj)
        else:
            obj = getattr(instance, attr_name)
            return obj, get_type_hints(obj)

    def _create_execute_callback(self, action_name, action_value_mapping):
        """创建动作执行回调函数"""

        async def execute_callback(goal_handle: ServerGoalHandle):
            # 初始化结果信息变量
            execution_error = ""
            execution_success = False
            action_return_value = None

        #####    self.lab_logger().info(f"执行动作: {action_name}")
            goal = goal_handle.request

            # 从目标消息中提取参数, 并调用对应的方法
            if "sequence" in action_value_mapping:
                # 如果一个指令对应函数的连续调用，如启动和等待结果，默认参数应该属于第一个函数调用
                def ACTION(**kwargs):
                    for i, action in enumerate(action_value_mapping["sequence"]):
                        if i == 0:
                            self.lab_logger().info(f"执行序列动作第一步: {action}")
                            self.get_real_function(self.driver_instance, action)[0](**kwargs)
                        else:
                            self.lab_logger().info(f"执行序列动作后续步骤: {action}")
                            self.get_real_function(self.driver_instance, action)[0]()

                action_paramtypes = self.get_real_function(self.driver_instance, action_value_mapping["sequence"][0])[1]
            else:
                ACTION, action_paramtypes = self.get_real_function(self.driver_instance, action_name)

            action_kwargs = convert_from_ros_msg_with_mapping(goal, action_value_mapping["goal"])
            self.lab_logger().debug(f"任务 {ACTION.__name__} 接收到原始目标: {action_kwargs}")
            error_skip = False
            # 向Host查询物料当前状态，如果是host本身的增加物料的请求，则直接跳过
            if action_name not in ["create_resource_detailed", "create_resource"]:
                for k, v in goal.get_fields_and_field_types().items():
                    if v in ["unilabos_msgs/Resource", "sequence<unilabos_msgs/Resource>"]:
                        self.lab_logger().info(f"查询资源状态: Key: {k} Type: {v}")
                        current_resources: Union[List[Resource], List[List[Resource]]] = []
                        # TODO: resource后面需要分组
                        only_one_resource = False
                        try:
                            if isinstance(action_kwargs[k], list) and len(action_kwargs[k]) > 1:
                                for i in action_kwargs[k]:
                                    r = ResourceGet.Request()
                                    r.id = i["id"]  # splash optional
                                    r.with_children = True
                                    response = await self._resource_clients["resource_get"].call_async(r)
                                    current_resources.append(response.resources)
                            else:
                                only_one_resource = True
                                r = ResourceGet.Request()
                                r.id = (
                                    action_kwargs[k]["id"]
                                    if v == "unilabos_msgs/Resource"
                                    else action_kwargs[k][0]["id"]
                                )
                                r.with_children = True
                                response = await self._resource_clients["resource_get"].call_async(r)
                                current_resources.extend(response.resources)
                        except Exception:
                            logger.error(f"资源查询失败，默认使用本地资源")
                        # 删除对response.resources的检查，因为它总是存在
                        type_hint = action_paramtypes[k]
                        final_type = get_type_class(type_hint)
                        if only_one_resource:
                            resources_list: List[Dict[str, Any]] = [convert_from_ros_msg(rs) for rs in current_resources]  # type: ignore
                            self.lab_logger().debug(f"资源查询结果: {len(resources_list)} 个资源")
                            final_resource = convert_resources_to_type(resources_list, final_type)
                        # 判断 ACTION 是否需要特殊的物料类型如 pylabrobot.resources.Resource，并做转换
                        else:
                            resources_list: List[List[Dict[str, Any]]] = [[convert_from_ros_msg(rs) for rs in sub_res_list] for sub_res_list in current_resources]  # type: ignore
                            final_resource = [convert_resources_to_type(sub_res_list, final_type)[0] for sub_res_list in resources_list]
                        try:
                            action_kwargs[k] = self.resource_tracker.figure_resource(final_resource, try_mode=False)
                        except Exception as e:
                            self.lab_logger().error(f"物料实例获取失败: {e}\n{traceback.format_exc()}")
                            error_skip = True
                            execution_error = traceback.format_exc()
                            break

            ##### self.lab_logger().info(f"准备执行: {action_kwargs}, 函数: {ACTION.__name__}")
            time_start = time.time()
            time_overall = 100
            future = None
            if not error_skip:
                # 将阻塞操作放入线程池执行
                if asyncio.iscoroutinefunction(ACTION):
                    try:
                        ##### self.lab_logger().info(f"异步执行动作 {ACTION}")
                        future = ROS2DeviceNode.run_async_func(ACTION, trace_error=False, **action_kwargs)

                        def _handle_future_exception(fut):
                            nonlocal execution_error, execution_success, action_return_value
                            try:
                                action_return_value = fut.result()
                                execution_success = True
                            except Exception as e:
                                execution_error = traceback.format_exc()
                                error(f"异步任务 {ACTION.__name__} 报错了\n{traceback.format_exc()}\n原始输入：{action_kwargs}")
                                error(traceback.format_exc())

                        future.add_done_callback(_handle_future_exception)
                    except Exception as e:
                        execution_error = traceback.format_exc()
                        execution_success = False
                        self.lab_logger().error(f"创建异步任务失败: {traceback.format_exc()}")
                else:
                #####    self.lab_logger().info(f"同步执行动作 {ACTION}")
                    future = self._executor.submit(ACTION, **action_kwargs)

                    def _handle_future_exception(fut):
                        nonlocal execution_error, execution_success, action_return_value
                        try:
                            action_return_value = fut.result()
                            execution_success = True
                        except Exception as e:
                            error(f"同步任务 {ACTION.__name__} 报错了\n{traceback.format_exc()}\n原始输入：{action_kwargs}")

                    future.add_done_callback(_handle_future_exception)

            action_type = action_value_mapping["type"]
            feedback_msg_types = action_type.Feedback.get_fields_and_field_types()
            result_msg_types = action_type.Result.get_fields_and_field_types()

            while future is not None and not future.done():
                if goal_handle.is_cancel_requested:
                    self.lab_logger().info(f"取消动作: {action_name}")
                    future.cancel()  # 尝试取消线程池中的任务
                    goal_handle.canceled()
                    return action_type.Result()

                self.time_spent = time.time() - time_start
                self.time_remaining = time_overall - self.time_spent

                # 发布反馈
                feedback_values = {}
                for msg_name, attr_name in action_value_mapping["feedback"].items():
                    if hasattr(self.driver_instance, f"get_{attr_name}"):
                        method = getattr(self.driver_instance, f"get_{attr_name}")
                        if not asyncio.iscoroutinefunction(method):
                            feedback_values[msg_name] = method()
                    elif hasattr(self.driver_instance, attr_name):
                        feedback_values[msg_name] = getattr(self.driver_instance, attr_name)

                if self._print_publish:
                    self.lab_logger().info(f"反馈: {feedback_values}")

                feedback_msg = convert_to_ros_msg_with_mapping(
                    ros_msg_type=action_type.Feedback(),
                    obj=feedback_values,
                    value_mapping=action_value_mapping["feedback"],
                )

                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)

            if future is not None and future.cancelled():
                self.lab_logger().info(f"动作 {action_name} 已取消")
                return action_type.Result()

            ##### self.lab_logger().info(f"动作执行完成: {action_name}")
            del future

            # 向Host更新物料当前状态
            if action_name not in ["create_resource_detailed", "create_resource"]:
                for k, v in goal.get_fields_and_field_types().items():
                    if v not in ["unilabos_msgs/Resource", "sequence<unilabos_msgs/Resource>"]:
                        continue
                    self.lab_logger().info(f"更新资源状态: {k}")
                    r = ResourceUpdate.Request()
                    # 仅当action_kwargs[k]不为None时尝试转换
                    akv = action_kwargs[k]  # 已经是完成转换的物料了，只需要转换成ros msg Resource了
                    apv = action_paramtypes[k]
                    final_type = get_type_class(apv)
                    if final_type is None:
                        continue
                    try:
                        seen = set()
                        unique_resources = []
                        for rs in akv:
                            res = self.resource_tracker.parent_resource(rs)  # 获取 resource 对象
                            if id(res) not in seen:
                                seen.add(id(res))
                                converted_list = convert_resources_from_type([res], final_type)
                                unique_resources.extend([convert_to_ros_msg(Resource, converted) for converted in converted_list])

                        r.resources = unique_resources

                        response = await self._resource_clients["resource_update"].call_async(r)
                        self.lab_logger().debug(f"资源更新结果: {response}")
                    except Exception as e:
                        self.lab_logger().error(f"资源更新失败: {e}")
                        self.lab_logger().error(traceback.format_exc())

            # 发布结果
            goal_handle.succeed()
            ##### self.lab_logger().info(f"设置动作成功: {action_name}")

            result_values = {}
            for msg_name, attr_name in action_value_mapping["result"].items():
                if hasattr(self.driver_instance, f"get_{attr_name}"):
                    result_values[msg_name] = getattr(self.driver_instance, f"get_{attr_name}")()
                elif hasattr(self.driver_instance, attr_name):
                    result_values[msg_name] = getattr(self.driver_instance, attr_name)

            result_msg = convert_to_ros_msg_with_mapping(
                ros_msg_type=action_type.Result(), obj=result_values, value_mapping=action_value_mapping["result"]
            )

            for attr_name in result_msg_types.keys():
                if attr_name in ["success", "reached_goal"]:
                    setattr(result_msg, attr_name, True)
                elif attr_name == "return_info":
                    setattr(result_msg, attr_name, get_result_info_str(execution_error, execution_success, action_return_value))

            ##### self.lab_logger().info(f"动作 {action_name} 完成并返回结果")
            return result_msg

        return execute_callback

    # 异步上下文管理方法
    async def __aenter__(self):
        """进入异步上下文"""
        self.lab_logger().info(f"进入异步上下文: {self.device_id}")
        if hasattr(self.driver_instance, "__aenter__"):
            await self.driver_instance.__aenter__()  # type: ignore
        self.lab_logger().info(f"异步上下文初始化完成: {self.device_id}")
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """退出异步上下文"""
        self.lab_logger().info(f"退出异步上下文: {self.device_id}")
        if hasattr(self.driver_instance, "__aexit__"):
            await self.driver_instance.__aexit__(exc_type, exc_val, exc_tb)  # type: ignore
        self.lab_logger().info(f"异步上下文清理完成: {self.device_id}")


class DeviceInitError(Exception):
    pass

class JsonCommandInitError(Exception):
    pass

class ROS2DeviceNode:
    """
    ROS2设备节点类

    这个类封装了设备类实例和ROS2节点的功能，提供ROS2接口。
    它不继承设备类，而是通过代理模式访问设备类的属性和方法。
    """

    # 类变量，用于循环管理
    _loop = None
    _loop_running = False
    _loop_thread = None

    @classmethod
    def get_loop(cls):
        return cls._loop

    @classmethod
    def run_async_func(cls, func, trace_error=True, **kwargs):
        return run_async_func(func, loop=cls._loop, trace_error=trace_error, **kwargs)

    @property
    def driver_instance(self):
        return self._driver_instance

    @property
    def ros_node_instance(self):
        return self._ros_node

    def __init__(
        self,
        device_id: str,
        driver_class: Type[T],
        device_config: Dict[str, Any],
        driver_params: Dict[str, Any],
        status_types: Dict[str, Any],
        action_value_mappings: Dict[str, Any],
        hardware_interface: Dict[str, Any],
        children: Dict[str, Any],
        print_publish: bool = True,
        driver_is_ros: bool = False,
    ):
        """
        初始化ROS2设备节点

        Args:
            device_id: 设备标识符
            driver_class: 设备类
            device_config: 原始初始化的json
            driver_params: driver初始化的参数
            status_types: 状态类型映射
            action_value_mappings: 动作值映射
            hardware_interface: 硬件接口配置
            children:
            print_publish: 是否打印发布信息
            driver_is_ros:
        """
        # 在初始化时检查循环状态
        if ROS2DeviceNode._loop_running and ROS2DeviceNode._loop_thread is not None:
            pass
        elif ROS2DeviceNode._loop_thread is None:
            self._start_loop()

        # 保存设备类是否支持异步上下文
        self._has_async_context = hasattr(driver_class, "__aenter__") and hasattr(driver_class, "__aexit__")
        self._driver_class = driver_class
        self.device_config = device_config
        self.driver_is_ros = driver_is_ros
        self.driver_is_workstation = False
        self.resource_tracker = DeviceNodeResourceTracker()

        # use_pylabrobot_creator 使用 cls的包路径检测
        use_pylabrobot_creator = (
            driver_class.__module__.startswith("pylabrobot")
            or driver_class.__name__ == "LiquidHandlerAbstract"
            or driver_class.__name__ == "LiquidHandlerBiomek"
            or driver_class.__name__ == "PRCXI9300Handler"
        )

        # TODO: 要在创建之前预先请求服务器是否有当前id的物料，放到resource_tracker中，让pylabrobot进行创建
        # 创建设备类实例
        if use_pylabrobot_creator:
            # 先对pylabrobot的子资源进行加载，不然subclass无法认出
            # 在下方对于加载Deck等Resource要手动import
            register()
            self._driver_creator = PyLabRobotCreator(
                driver_class, children=children, resource_tracker=self.resource_tracker
            )
        else:
            from unilabos.devices.workstation.workstation_base import WorkstationBase

            if issubclass(self._driver_class, WorkstationBase):  # 是WorkstationNode的子节点，就要调用WorkstationNodeCreator
                self.driver_is_workstation = True
                self._driver_creator = WorkstationNodeCreator(driver_class, children=children, resource_tracker=self.resource_tracker)
            else:
                self._driver_creator = DeviceClassCreator(driver_class, children=children, resource_tracker=self.resource_tracker)

        if driver_is_ros:
            driver_params["device_id"] = device_id
            driver_params["resource_tracker"] = self.resource_tracker
        self._driver_instance = self._driver_creator.create_instance(driver_params)
        if self._driver_instance is None:
            logger.critical(f"设备实例创建失败 {driver_class}, params: {driver_params}")
            raise DeviceInitError("错误: 设备实例创建失败")
        
        # 创建ROS2节点
        if driver_is_ros:
            self._ros_node = self._driver_instance  # type: ignore
        elif self.driver_is_workstation:
            from unilabos.ros.nodes.presets.workstation import ROS2WorkstationNode
            self._ros_node = ROS2WorkstationNode(
                protocol_type=driver_params["protocol_type"],
                children=children,
                driver_instance=self._driver_instance,  # type: ignore
                device_id=device_id,
                status_types=status_types,
                action_value_mappings=action_value_mappings,
                hardware_interface=hardware_interface,
                print_publish=print_publish,
                resource_tracker=self.resource_tracker,
            )
        else:
            self._ros_node = BaseROS2DeviceNode(
                driver_instance=self._driver_instance,
                device_id=device_id,
                status_types=status_types,
                action_value_mappings=action_value_mappings,
                hardware_interface=hardware_interface,
                print_publish=print_publish,
                resource_tracker=self.resource_tracker,
            )
        self._ros_node: BaseROS2DeviceNode
        self._ros_node.lab_logger().info(f"初始化完成 {self._ros_node.uuid} {self.driver_is_ros}")
        self.driver_instance._ros_node = self._ros_node  # type: ignore
        self.driver_instance._execute_driver_command = self._execute_driver_command  # type: ignore
        self.driver_instance._execute_driver_command_async = self._execute_driver_command_async  # type: ignore
        if hasattr(self.driver_instance, "post_init"):
            try:
                self.driver_instance.post_init(self._ros_node)  # type: ignore
            except Exception as e:
                self._ros_node.lab_logger().error(f"设备后初始化失败: {e}")

    def _execute_driver_command(self, string: str):
        try:
            target = json.loads(string)
        except Exception as ex:
            try:
                target = yaml.safe_load(io.StringIO(string))
            except Exception as ex2:
                raise JsonCommandInitError(f"执行动作时JSON/YAML解析失败: \n{ex}\n{ex2}\n原内容: {string}\n{traceback.format_exc()}")
        try:
            function_name = target["function_name"]
            function_args = target["function_args"]
            assert isinstance(function_args, dict), "执行动作时JSON必须为dict类型\n原JSON: {string}"
            function = getattr(self.driver_instance, function_name)
            assert callable(function), f"执行动作时JSON中的function_name对应的函数不可调用: {function_name}\n原JSON: {string}"
            return function(**function_args)
        except KeyError as ex:
            raise JsonCommandInitError(f"执行动作时JSON缺少function_name或function_args: {ex}\n原JSON: {string}\n{traceback.format_exc()}")

    async def _execute_driver_command_async(self, string: str):
        try:
            target = json.loads(string)
        except Exception as ex:
            try:
                target = yaml.safe_load(io.StringIO(string))
            except Exception as ex2:
                raise JsonCommandInitError(f"执行动作时JSON/YAML解析失败: \n{ex}\n{ex2}\n原内容: {string}\n{traceback.format_exc()}")
        try:
            function_name = target["function_name"]
            function_args = target["function_args"]
            assert isinstance(function_args, dict), "执行动作时JSON必须为dict类型\n原JSON: {string}"
            function = getattr(self.driver_instance, function_name)
            assert callable(function), f"执行动作时JSON中的function_name对应的函数不可调用: {function_name}\n原JSON: {string}"
            assert asyncio.iscoroutinefunction(function), f"执行动作时JSON中的function并非异步: {function_name}\n原JSON: {string}"
            return await function(**function_args)
        except KeyError as ex:
            raise JsonCommandInitError(f"执行动作时JSON缺少function_name或function_args: {ex}\n原JSON: {string}\n{traceback.format_exc()}")

    def _start_loop(self):
        def run_event_loop():
            loop = asyncio.new_event_loop()
            ROS2DeviceNode._loop = loop
            asyncio.set_event_loop(loop)
            loop.run_forever()

        ROS2DeviceNode._loop_thread = threading.Thread(target=run_event_loop, daemon=True, name="ROS2DeviceNode")
        ROS2DeviceNode._loop_thread.start()
        logger.info(f"循环线程已启动")


class DeviceInfoType(TypedDict):
    id: str
    uuid: str
    node_name: str
    namespace: str
    driver_instance: Any
    status_publishers: Dict[str, PropertyPublisher]
    actions: Dict[str, ActionServer]
    hardware_interface: Dict[str, Any]
    base_node_instance: BaseROS2DeviceNode
