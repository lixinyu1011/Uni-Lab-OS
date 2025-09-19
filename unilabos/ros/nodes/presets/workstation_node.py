import json
import time
import traceback
from pprint import pprint, saferepr, pformat
from typing import Union, Dict, Any

import rclpy
from rosidl_runtime_py import message_to_ordereddict

from unilabos.messages import *  # type: ignore  # protocol names
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from unilabos_msgs.msg import Resource  # type: ignore
from unilabos_msgs.srv import ResourceGet, ResourceUpdate  # type: ignore

from unilabos.compile import action_protocol_generators
from unilabos.resources.graphio import list_to_nested_dict, nested_dict_to_list
from unilabos.ros.initialize_device import initialize_device_from_dict
from unilabos.ros.msgs.message_converter import (
    get_action_type,
    convert_to_ros_msg,
    convert_from_ros_msg,
    convert_from_ros_msg_with_mapping, String,
)
from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode, DeviceNodeResourceTracker, ROS2DeviceNode
from unilabos.utils.log import error
from unilabos.utils.type_check import serialize_result_info, get_result_info_str


class ROS2WorkstationNode(BaseROS2DeviceNode):
    """
    ROS2WorkstationNode代表管理ROS2环境中设备通信和动作的协议节点。
    它初始化设备节点，处理动作客户端，并基于指定的协议执行工作流。
    它还物理上代表一组协同工作的设备，如带夹持器的机械臂，带传送带的CNC机器等。
    """

    # create_action_server = False  # Action Server要自己创建

    def __init__(
        self,
        device_id: str,
        children: dict,
        protocol_type: Union[str, list[str]],
        resource_tracker: DeviceNodeResourceTracker,
        workstation_config: dict = {},
        workstation_instance: object = None,
        *args,
        **kwargs,
    ):
        self._setup_protocol_names(protocol_type)

        # 初始化其它属性
        self.children = children
        self.workstation_config = workstation_config or {}  # 新增：保存工作站配置
        self.communication_interfaces = self.workstation_config.get('communication_interfaces', {})  # 从工作站配置获取通信接口
        
        # 新增：获取工作站实例（如果存在）
        self.workstation_instance = workstation_instance
        
        self._busy = False
        self.sub_devices = {}
        self.communication_devices = {}
        self.logical_devices = {}
        self._goals = {}
        self._protocol_servers = {}
        self._action_clients = {}

        # 初始化基类，让基类处理常规动作
        # 如果有工作站实例，使用工作站实例作为driver_instance
        driver_instance = self.workstation_instance if self.workstation_instance else self
        
        super().__init__(
            driver_instance=driver_instance,
            device_id=device_id,
            status_types={},
            action_value_mappings=self.protocol_action_mappings,
            hardware_interface={"name": "hardware_interface", "write": "send_command", "read": "read_data", "extra_info": []},
            print_publish=False,
            resource_tracker=resource_tracker,
        )

        # 初始化子设备
        self._initialize_child_devices()
        
        if isinstance(getattr(driver_instance, "hardware_interface", None), str):
            self.logical_devices[device_id] = driver_instance
        else:
            self.communication_devices[device_id] = driver_instance

        # 设置硬件接口代理
        for device_id, device_node in self.logical_devices.items():
            if device_node and hasattr(device_node, 'ros_node_instance'):
                self._setup_device_hardware_proxy(device_id, device_node)

        # 新增：如果有工作站实例，建立双向引用和硬件接口设置
        if self.workstation_instance:
            self._setup_workstation_integration()

    def _setup_workstation_integration(self):
        """设置工作站集成 - 统一设备处理模式"""
        # 1. 建立协议节点引用
        self.workstation_instance.set_workstation_node(self)
        
        self.lab_logger().info(f"ROS2WorkstationNode {self.device_id} 与工作站实例 {type(self.workstation_instance).__name__} 集成完成")

    def _initialize_child_devices(self):
        """初始化子设备 - 重构为更清晰的方法"""
        # 设备分类字典 - 统一管理
        
        for device_id, device_config in self.children.items():
            if device_config.get("type", "device") != "device":
                self.lab_logger().debug(
                    f"[Protocol Node] Skipping type {device_config['type']} {device_id} already existed, skipping."
                )
                continue
                
            try:
                d = self.initialize_device(device_id, device_config)
                if d is None:
                    continue

                # 统一的设备分类逻辑
                device_type = device_config.get("device_type", "logical")
                
                # 兼容旧的ID匹配方式和新的配置方式
                if device_type == "communication" or "serial_" in device_id or "io_" in device_id:
                    self.communication_devices[device_id] = d  # 新的统一方式
                    self.lab_logger().info(f"通信设备 {device_id} 初始化并分类成功")
                elif device_type == "logical":
                    self.logical_devices[device_id] = d
                    self.lab_logger().info(f"逻辑设备 {device_id} 初始化并分类成功")
                else:
                    # 默认作为逻辑设备处理
                    self.logical_devices[device_id] = d
                    self.lab_logger().info(f"设备 {device_id} 作为逻辑设备处理")
                    
            except Exception as ex:
                self.lab_logger().error(f"[Protocol Node] Failed to initialize device {device_id}: {ex}\n{traceback.format_exc()}")

    def _setup_device_hardware_proxy(self, device_id: str, device: ROS2DeviceNode):
        """统一的设备硬件接口代理设置方法
        
        Args:
            device_id: 设备ID
            device: 设备实例
        """
        hardware_interface = device.ros_node_instance._hardware_interface
        if not self._validate_hardware_interface(device, hardware_interface):
            return
            
        # 获取硬件接口名称
        interface_name = getattr(device.driver_instance, hardware_interface["name"])
        
        # 情况1: 如果interface_name是字符串，说明需要转发到其他设备
        if isinstance(interface_name, str):
            # 查找目标设备
            communication_device = self.communication_devices.get(device_id, None)
            if not communication_device:
                self.lab_logger().error(f"转发目标设备 {device_id} 不存在")
                return

            read_method = hardware_interface.get("read", None)
            write_method = hardware_interface.get("write", None)

            # 设置传统硬件代理
            communicate_hardware_info = communication_device.ros_node_instance._hardware_interface
            self._setup_hardware_proxy(device, communication_device, read_method, write_method)
            self.lab_logger().info(
                f"传统通信代理：为子设备{device.device_id} "
                f"添加了{read_method}方法(来源：{communication_device.device_id} {communicate_hardware_info['read']}) "
                f"添加了{write_method}方法(来源：{communication_device.device_id} {communicate_hardware_info['write']})"
            )
            self.lab_logger().info(f"字符串转发代理：设备 {device.device_id} -> {device_id}")
            
        # 情况2: 如果设备有communication_interface配置，设置协议代理
        elif hasattr(self, 'communication_interfaces') and device_id in self.communication_interfaces:
            interface_config = self._get_communication_interface_config(device_id)
            protocol_type = interface_config.get('protocol_type', 'modbus')
            self._setup_communication_proxy(device, interface_config, protocol_type)
            
        # 情况3: 其他情况，使用默认处理
        else:
            self.lab_logger().debug(f"设备 {device_id} 使用默认硬件接口处理")

    def _get_communication_interface_config(self, device_id: str) -> dict:
        """获取设备的通信接口配置"""
        # 优先从工作站配置获取
        if hasattr(self, 'communication_interfaces') and device_id in self.communication_interfaces:
            return self.communication_interfaces[device_id]
        
        # 从设备自身配置获取
        device_node = self.logical_devices[device_id]
        if device_node and hasattr(device_node.driver_instance, 'communication_interface'):
            return getattr(device_node.driver_instance, 'communication_interface')
            
        return {}

    def _validate_hardware_interface(self, device: ROS2DeviceNode, hardware_interface: dict) -> bool:
        """验证硬件接口配置"""
        return (
            hasattr(device.driver_instance, hardware_interface["name"])
            and hasattr(device.driver_instance, hardware_interface["write"])
            and (hardware_interface["read"] is None or hasattr(device.driver_instance, hardware_interface["read"]))
        )

    def _setup_communication_proxy(self, logical_device: ROS2DeviceNode, interface_config, protocol_type):
        """为逻辑设备设置通信代理 - 统一方法"""
        try:
            # 获取通信设备
            comm_device_id = interface_config.get('device_id')
            comm_device = self.communication_devices.get(comm_device_id)
            
            if not comm_device:
                self.lab_logger().error(f"通信设备 {comm_device_id} 不存在")
                return
            
            # 根据协议类型设置不同的代理方法
            if protocol_type == 'modbus':
                self._setup_modbus_proxy(logical_device, comm_device, interface_config)
            elif protocol_type == 'opcua':
                self._setup_opcua_proxy(logical_device, comm_device, interface_config)
            elif protocol_type == 'http':
                self._setup_http_proxy(logical_device, comm_device, interface_config)
            elif protocol_type == 'serial':
                self._setup_serial_proxy(logical_device, comm_device, interface_config)
            else:
                self.lab_logger().warning(f"不支持的协议类型: {protocol_type}")
                return
            
            self.lab_logger().info(f"通信代理：为逻辑设备 {logical_device.device_id} 设置{protocol_type}通信代理 -> {comm_device_id}")
            
        except Exception as e:
            self.lab_logger().error(f"设置通信代理失败: {e}")

    def _setup_protocol_names(self, protocol_type):
        # 处理协议类型
        if isinstance(protocol_type, str):
            if "," not in protocol_type:
                self.protocol_names = [protocol_type]
            else:
                self.protocol_names = [protocol.strip() for protocol in protocol_type.split(",")]
        else:
            self.protocol_names = protocol_type
        # 准备协议相关的动作值映射
        self.protocol_action_mappings = {}
        for protocol_name in self.protocol_names:
            protocol_type = globals()[protocol_name]
            self.protocol_action_mappings[protocol_name] = get_action_type(protocol_type)

    def initialize_device(self, device_id, device_config):
        """初始化设备并创建相应的动作客户端"""
        # device_id_abs = f"{self.device_id}/{device_id}"
        device_id_abs = f"{device_id}"
        self.lab_logger().info(f"初始化子设备: {device_id_abs}")
        d = self.sub_devices[device_id] = initialize_device_from_dict(device_id_abs, device_config)

        # 为子设备的每个动作创建动作客户端
        if d is not None and hasattr(d, "ros_node_instance"):
            node = d.ros_node_instance
            node.resource_tracker = self.resource_tracker  # 站内应当共享资源跟踪器
            for action_name, action_mapping in node._action_value_mappings.items():
                if action_name.startswith("auto-") or str(action_mapping.get("type", "")).startswith("UniLabJsonCommand"):
                    continue
                action_id = f"/devices/{device_id_abs}/{action_name}"
                if action_id not in self._action_clients:
                    try:
                        self._action_clients[action_id] = ActionClient(
                            self, action_mapping["type"], action_id, callback_group=self.callback_group
                        )
                    except Exception as ex:
                        self.lab_logger().error(f"创建动作客户端失败: {action_id}, 错误: {ex}")
                        continue
                    self.lab_logger().trace(f"为子设备 {device_id} 创建动作客户端: {action_name}")
        return d

    def create_ros_action_server(self, action_name, action_value_mapping):
        """创建ROS动作服务器"""
        # 和Base创建的路径是一致的
        protocol_name = action_name
        action_type = action_value_mapping["type"]
        str_action_type = str(action_type)[8:-2]
        protocol_type = globals()[protocol_name]
        protocol_steps_generator = action_protocol_generators[protocol_type]

        self._action_servers[action_name] = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._create_protocol_execute_callback(action_name, protocol_steps_generator),
            callback_group=ReentrantCallbackGroup(),
        )

        self.lab_logger().trace(f"发布动作: {action_name}, 类型: {str_action_type}")

    def _create_protocol_execute_callback(self, protocol_name, protocol_steps_generator):
        async def execute_protocol(goal_handle: ServerGoalHandle):
            """执行完整的工作流"""
            # 初始化结果信息变量
            execution_error = ""
            execution_success = False
            protocol_return_value = None
            self.get_logger().info(f"Executing {protocol_name} action...")
            action_value_mapping = self._action_value_mappings[protocol_name]
            step_results = []
            try:
                print("+" * 30)
                print(protocol_steps_generator)
                # 从目标消息中提取参数, 并调用Protocol生成器(根据设备连接图)生成action步骤
                goal = goal_handle.request
                protocol_kwargs = convert_from_ros_msg_with_mapping(goal, action_value_mapping["goal"])
                
                # # 🔧 添加调试信息
                # print(f"🔍 转换后的 protocol_kwargs: {protocol_kwargs}")
                # print(f"🔍 vessel 在转换后: {protocol_kwargs.get('vessel', 'NOT_FOUND')}")

                # # 🔧 完全禁用Host查询，直接使用转换后的数据
                # print(f"🔧 跳过Host查询，直接使用转换后的数据")
                # 向Host查询物料当前状态
                for k, v in goal.get_fields_and_field_types().items():
                    if v in ["unilabos_msgs/Resource", "sequence<unilabos_msgs/Resource>"]:
                        r = ResourceGet.Request()
                        resource_id = (
                            protocol_kwargs[k]["id"] if v == "unilabos_msgs/Resource" else protocol_kwargs[k][0]["id"]
                        )
                        r.id = resource_id
                        r.with_children = True
                        response = await self._resource_clients["resource_get"].call_async(r)
                        protocol_kwargs[k] = list_to_nested_dict(
                            [convert_from_ros_msg(rs) for rs in response.resources]
                        )

                self.lab_logger().info(f"🔍 最终的 vessel: {protocol_kwargs.get('vessel', 'NOT_FOUND')}")

                from unilabos.resources.graphio import physical_setup_graph

                self.lab_logger().info(f"Working on physical setup: {physical_setup_graph}")
                protocol_steps = protocol_steps_generator(G=physical_setup_graph, **protocol_kwargs)
                logs = []
                for step in protocol_steps:
                    if isinstance(step, dict) and "log_message" in step.get("action_kwargs", {}):
                        logs.append(step)
                    elif isinstance(step, list):
                        logs.append(step)
                self.lab_logger().info(f"Goal received: {protocol_kwargs}, running steps: "
                                       f"{json.dumps(logs, indent=4, ensure_ascii=False)}")

                time_start = time.time()
                time_overall = 100
                self._busy = True

                # 逐步执行工作流
                for i, action in enumerate(protocol_steps):
                    # self.get_logger().info(f"Running step {i + 1}: {action}")
                    if isinstance(action, dict):
                        # 如果是单个动作，直接执行
                        if action["action_name"] == "wait":
                            time.sleep(action["action_kwargs"]["time"])
                            step_results.append({"step": i + 1, "action": "wait", "result": "completed"})
                        else:
                            result = await self.execute_single_action(**action)
                            step_results.append({"step": i + 1, "action": action["action_name"], "result": result})
                            ret_info = json.loads(getattr(result, "return_info", "{}"))
                            if not ret_info.get("suc", False):
                                raise RuntimeError(f"Step {i + 1} failed.")
                    elif isinstance(action, list):
                        # 如果是并行动作，同时执行
                        actions = action
                        futures = [
                            rclpy.get_global_executor().create_task(self.execute_single_action(**a)) for a in actions
                        ]
                        results = [await f for f in futures]
                        step_results.append(
                            {
                                "step": i + 1,
                                "parallel_actions": [a["action_name"] for a in actions],
                                "results": results,
                            }
                        )

                # 向Host更新物料当前状态
                for k, v in goal.get_fields_and_field_types().items():
                    if v in ["unilabos_msgs/Resource", "sequence<unilabos_msgs/Resource>"]:
                        r = ResourceUpdate.Request()
                        r.resources = [
                            convert_to_ros_msg(Resource, rs) for rs in nested_dict_to_list(protocol_kwargs[k])
                        ]
                        response = await self._resource_clients["resource_update"].call_async(r)

                # 设置成功状态和返回值
                execution_success = True
                protocol_return_value = {
                    "protocol_name": protocol_name,
                    "steps_executed": len(protocol_steps),
                    "step_results": step_results,
                    "total_time": time.time() - time_start,
                }

                goal_handle.succeed()

            except Exception as e:
                # 捕获并记录错误信息
                str_step_results = [{k: dict(message_to_ordereddict(v)) if k == "result" and hasattr(v, "SLOT_TYPES") else v for k, v in i.items()} for i in step_results]
                execution_error = f"{traceback.format_exc()}\n\nStep Result: {pformat(str_step_results)}"
                execution_success = False
                self.lab_logger().error(f"协议 {protocol_name} 执行出错: {str(e)} \n{traceback.format_exc()}")

                # 设置动作失败
                goal_handle.abort()

            finally:
                self._busy = False

            # 创建结果消息
            result = action_value_mapping["type"].Result()
            result.success = execution_success

            # 获取结果消息类型信息，检查是否有return_info字段
            result_msg_types = action_value_mapping["type"].Result.get_fields_and_field_types()

            # 设置return_info字段（如果存在）
            for attr_name in result_msg_types.keys():
                if attr_name in ["success", "reached_goal"]:
                    setattr(result, attr_name, execution_success)
                elif attr_name == "return_info":
                    setattr(
                        result,
                        attr_name,
                        get_result_info_str(execution_error, execution_success, protocol_return_value),
                    )

            self.lab_logger().info(f"协议 {protocol_name} 完成并返回结果")
            return result

        return execute_protocol

    async def execute_single_action(self, device_id, action_name, action_kwargs):
        """执行单个动作"""
        # 构建动作ID
        if device_id in ["", None, "self"]:
            action_id = f"/devices/{self.device_id}/{action_name}"
        else:
            action_id = f"/devices/{device_id}/{action_name}"  # 执行时取消了主节点信息 /{self.device_id}

        # 检查动作客户端是否存在
        if action_id not in self._action_clients:
            self.lab_logger().error(f"找不到动作客户端: {action_id}")
            return None

        # 发送动作请求
        action_client = self._action_clients[action_id]
        goal_msg = convert_to_ros_msg(action_client._action_type.Goal(), action_kwargs)

        ##### self.lab_logger().info(f"发送动作请求到: {action_id}")
        action_client.wait_for_server()

        # 等待动作完成
        request_future = action_client.send_goal_async(goal_msg)
        handle = await request_future

        if not handle.accepted:
            self.lab_logger().error(f"动作请求被拒绝: {action_name}")
            return None

        result_future = await handle.get_result_async()
        ##### self.lab_logger().info(f"动作完成: {action_name}")

        return result_future.result

    """还没有改过的部分"""

    def _setup_modbus_proxy(self, logical_device: ROS2DeviceNode, comm_device: ROS2DeviceNode, interface_config):
        """设置Modbus通信代理"""
        config = interface_config.get('config', {})
        
        # 设置Modbus读写方法
        def modbus_read(address, count=1, function_code=3):
            """Modbus读取方法"""
            return comm_device.driver_instance.read_holding_registers(
                address=address,
                count=count,
                slave_id=config.get('slave_id', 1)
            )
        
        def modbus_write(address, value, function_code=6):
            """Modbus写入方法"""
            if isinstance(value, (list, tuple)):
                return comm_device.driver_instance.write_multiple_registers(
                    address=address,
                    values=value,
                    slave_id=config.get('slave_id', 1)
                )
            else:
                return comm_device.driver_instance.write_single_register(
                    address=address,
                    value=value,
                    slave_id=config.get('slave_id', 1)
                )
        
        # 绑定方法到逻辑设备
        setattr(logical_device.driver_instance, 'comm_read', modbus_read)
        setattr(logical_device.driver_instance, 'comm_write', modbus_write)
        setattr(logical_device.driver_instance, 'comm_config', config)
        setattr(logical_device.driver_instance, 'comm_protocol', 'modbus')

    def _setup_opcua_proxy(self, logical_device: ROS2DeviceNode, comm_device: ROS2DeviceNode, interface_config):
        """设置OPC UA通信代理"""
        config = interface_config.get('config', {})
        
        def opcua_read(node_id):
            """OPC UA读取方法"""
            return comm_device.driver_instance.read_node_value(node_id)
        
        def opcua_write(node_id, value):
            """OPC UA写入方法"""
            return comm_device.driver_instance.write_node_value(node_id, value)
        
        # 绑定方法到逻辑设备
        setattr(logical_device.driver_instance, 'comm_read', opcua_read)
        setattr(logical_device.driver_instance, 'comm_write', opcua_write)
        setattr(logical_device.driver_instance, 'comm_config', config)
        setattr(logical_device.driver_instance, 'comm_protocol', 'opcua')

    def _setup_http_proxy(self, logical_device: ROS2DeviceNode, comm_device: ROS2DeviceNode, interface_config):
        """设置HTTP/RPC通信代理"""
        config = interface_config.get('config', {})
        base_url = config.get('base_url', 'http://localhost:8080')
        
        def http_read(endpoint, params=None):
            """HTTP GET请求"""
            url = f"{base_url.rstrip('/')}/{endpoint.lstrip('/')}"
            return comm_device.driver_instance.get_request(url, params=params)
        
        def http_write(endpoint, data):
            """HTTP POST请求"""
            url = f"{base_url.rstrip('/')}/{endpoint.lstrip('/')}"
            return comm_device.driver_instance.post_request(url, data=data)
        
        # 绑定方法到逻辑设备
        setattr(logical_device.driver_instance, 'comm_read', http_read)
        setattr(logical_device.driver_instance, 'comm_write', http_write)
        setattr(logical_device.driver_instance, 'comm_config', config)
        setattr(logical_device.driver_instance, 'comm_protocol', 'http')

    def _setup_serial_proxy(self, logical_device: ROS2DeviceNode, comm_device: ROS2DeviceNode, interface_config):
        """设置串口通信代理"""
        config = interface_config.get('config', {})
        
        def serial_read(timeout=1.0):
            """串口读取方法"""
            return comm_device.driver_instance.read_data(timeout=timeout)
        
        def serial_write(data):
            """串口写入方法"""
            if isinstance(data, str):
                data = data.encode('utf-8')
            return comm_device.driver_instance.write_data(data)
        
        # 绑定方法到逻辑设备
        setattr(logical_device.driver_instance, 'comm_read', serial_read)
        setattr(logical_device.driver_instance, 'comm_write', serial_write)
        setattr(logical_device.driver_instance, 'comm_config', config)
        setattr(logical_device.driver_instance, 'comm_protocol', 'serial')

    def _setup_hardware_proxy(
        self, device: ROS2DeviceNode, communication_device: ROS2DeviceNode, read_method, write_method
    ):
        """为设备设置硬件接口代理"""
        # extra_info = [getattr(device.driver_instance, info) for info in communication_device.ros_node_instance._hardware_interface.get("extra_info", [])]
        write_func = getattr(
            communication_device.driver_instance, communication_device.ros_node_instance._hardware_interface["write"]
        )
        read_func = getattr(
            communication_device.driver_instance, communication_device.ros_node_instance._hardware_interface["read"]
        )

        def _read(*args, **kwargs):
            return read_func(*args, **kwargs)

        def _write(*args, **kwargs):
            return write_func(*args, **kwargs)

        if read_method:
            # bound_read = MethodType(_read, device.driver_instance)
            setattr(device.driver_instance, read_method, _read)

        if write_method:
            # bound_write = MethodType(_write, device.driver_instance)
            setattr(device.driver_instance, write_method, _write)

    async def _update_resources(self, goal, protocol_kwargs):
        """更新资源状态"""
        for k, v in goal.get_fields_and_field_types().items():
            if v in ["unilabos_msgs/Resource", "sequence<unilabos_msgs/Resource>"]:
                if protocol_kwargs[k] is not None:
                    try:
                        r = ResourceUpdate.Request()
                        r.resources = [
                            convert_to_ros_msg(Resource, rs) for rs in nested_dict_to_list(protocol_kwargs[k])
                        ]
                        await self._resource_clients["resource_update"].call_async(r)
                    except Exception as e:
                        self.lab_logger().error(f"更新资源失败: {e}")
