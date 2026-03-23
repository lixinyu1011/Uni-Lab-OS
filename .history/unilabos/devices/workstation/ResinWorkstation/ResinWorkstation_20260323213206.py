import json
import socket
import threading
import time
from dataclasses import dataclass, asdict
from typing import Any, Dict, Optional

from unilabos.devices.workstation.workstation_base import WorkstationBase
from unilabos.ros.nodes.presets.workstation import ROS2WorkstationNode
from unilabos.utils.log import logger

from unilabos.resources.resin_workstation import (
    ReagentState,
    ResinWorkstationDeck,
    ReagentBottle,
    ReagentRack
)


@dataclass
class ReactorState:
    """
    反应器状态类
    """
    reactor_id: int  # 反应器编号
    current_temperature: float = 0.0  # 当前温度
    target_temperature: float = 0.0  # 目标温度
    stirring_status: bool = False  # 搅拌状态
    stirring_speed: float = 0.0  # 搅拌转速
    n2_status: bool = False  # 氮气状态
    air_status: bool = False  # 空气状态
    status: str = "idle"  # 运行状态：idle, running, error
    error_message: str = ""  # 错误信息


@dataclass
class PostProcessState:
    """
    后处理系统状态类
    """
    post_process_id: int  # 后处理编号
    cleaning_status: bool = False  # 清洗状态
    discharge_status: bool = False  # 排液状态
    transferring_status: bool = False  # 溶液转移状态
    start_bottle: str = ""  # 当前转移起始瓶
    end_bottle: str = ""  # 当前转移终点瓶
    current_volume: float = 0.0  # 当前转移体积
    target_volume: float = 0.0  # 目标转移体积
    status: str = "idle"  # 运行状态：idle, running, error
    error_message: str = ""  # 错误信息


@dataclass
class DeviceState:
    """
    设备整体状态类
    """
    connected: bool = False  # 连接状态
    operation_mode: str = "local"  # 操作模式：local, remote
    device_status: str = "idle"  # 设备状态：idle, running, error
    reactors: Dict[int, ReactorState] = None  # 反应器状态字典
    post_processes: Dict[int, PostProcessState] = None  # 后处理系统状态字典
    reagents: Dict[int, ReagentState] = None  # 试剂状态字典
    last_updated: str = ""  # 最后更新时间
    error_message: str = ""  # 设备级错误信息
    solution_add_status: str = "idle"  # 溶液添加状态：idle, running, error
    current_solution_id: int = 0  # 当前添加的溶液编号
    current_volume: float = 0.0  # 当前添加的体积
    target_volume: float = 0.0  # 目标添加体积
    current_reactor_id: int = 0  # 当前添加的反应器编号
    
    def __post_init__(self):
        if self.reactors is None:
            self.reactors = {}
        if self.post_processes is None:
            self.post_processes = {}
        if self.reagents is None:
            self.reagents = {}
        self.last_updated = time.strftime("%Y-%m-%d %H:%M:%S")


class UDPClient:
    """
    UDP客户端类，用于与设备进行通信
    """
    def __init__(self, address: str = "127.0.0.1", port: int = 8888, timeout: float = 5.0):
        self.address = address
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.connected = False
        self.lock = threading.Lock()
        self.status_callback = None  # 状态更新回调函数
        self.listen_thread = None  # 状态监听线程
        self.listen_running = False  # 监听线程运行状态
        
        # 命令类型配置：立即响应/长时间运行
        self._immediate_response_commands = {
            "TOGGLE_LOCAL_REMOTE_CONTROL",
            "GET_DEVICE_STATE",
            "GET_REACTOR_STATE",
            "GET_POST_PROCESS_STATE",
            "GET_REAGENT_STATE",
            "GET_ALL_REAGENTS_STATE",
            "REACTOR_N2_ON",
            "REACTOR_N2_OFF",
            "REACTOR_AIR_ON",
            "REACTOR_AIR_OFF",
            "TEMP_SET",
            "START_STIR",
            "STOP_STIR",
            "POST_PROCESS_DISCHARGE_ON",
            "POST_PROCESS_DISCHARGE_OFF"
        }
        
        self._long_running_commands = {
            "REACTOR_SOLUTION_ADD",
            "POST_PROCESS_SOLUTION_ADD",
            "POST_PROCESS_CLEAN",
            "WAIT",
            "UPDATE_REAGENT_VOLUME"
        }
    
    def connect(self) -> bool:
        """
        连接到UDP服务器
        
        Returns:
            bool: 连接成功返回True，否则返回False
        """
        try:
            # UDP是无连接协议，这里只是初始化socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(self.timeout)
            self.connected = True
            logger.info(f"UDP客户端已初始化，目标地址: {self.address}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"UDP客户端初始化失败: {e}")
            self.connected = False
            return False
    
    def set_status_callback(self, callback):
        """
        设置状态更新回调函数
        
        Args:
            callback: 回调函数，接收状态数据作为参数
        """
        self.status_callback = callback
    
    def start_listen(self):
        """
        启动状态监听线程
        """
        if self.listen_running:
            logger.warning("状态监听线程已在运行")
            return
            
        self.listen_running = True
        self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listen_thread.start()
        logger.info("UDP状态监听线程已启动")
    
    def stop_listen(self):
        """
        停止状态监听线程
        """
        self.listen_running = False
        if self.listen_thread:
            self.listen_thread.join(timeout=1.0)
            self.listen_thread = None
        logger.info("UDP状态监听线程已停止")
    
    def _listen_loop(self):
        """
        状态监听循环，接收服务器主动推送的状态更新
        """
        while self.listen_running and self.connected and self.socket:
            try:
                # 设置较短的超时，以便定期检查listen_running状态
                self.socket.settimeout(0.5)
                response_data, _ = self.socket.recvfrom(1024)
                
                # 尝试解析JSON响应
                try:
                    response = json.loads(response_data.decode('utf-8'))
                    logger.debug(f"收到UDP状态更新: {response}")
                    
                    # 如果是状态更新消息，调用回调函数
                    if response.get("type") == "status_update" and self.status_callback:
                        self.status_callback(response.get("data", {}))
                except json.JSONDecodeError:
                    logger.error(f"UDP响应格式错误: {response_data}")
            except socket.timeout:
                # 超时是正常的，继续监听
                continue
            except Exception as e:
                logger.error(f"UDP监听错误: {e}")
                # 短暂暂停后继续监听
                time.sleep(0.5)
    
    def disconnect(self) -> bool:
        """
        断开UDP连接
        
        Returns:
            bool: 断开成功返回True，否则返回False
        """
        try:
            # 停止监听线程
            self.stop_listen()
            
            if self.socket:
                self.socket.close()
                self.socket = None
            self.connected = False
            logger.info("UDP客户端已断开连接")
            
            return True
        except Exception as e:
            logger.error(f"UDP客户端断开连接失败: {e}")
            return False
    
    def send_command(self, command: str, params: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        发送命令到UDP服务器
        
        Args:
            command: 命令名称
            params: 命令参数
            
        Returns:
            Dict[str, Any]: 服务器响应
        """
        with self.lock:
            if not self.connected or not self.socket:
                logger.error("UDP客户端未连接")
                return {"status": "error", "message": "UDP客户端未连接"}
            
            try:
                # 构建函数调用格式命令
                params = params or {}
                
                # 处理参数，转换为函数调用格式
                param_list = []
                for key, value in params.items():
                    # 处理反应器ID，转换为reactor_1格式
                    if key == "reactor_id" or key == "post_process_id":
                        param_list.append(f"{key[:-3]}_{value}")
                    else:
                        # 根据参数类型格式化
                        if isinstance(value, str):
                            param_list.append(value)
                        else:
                            param_list.append(str(value))
                
                # 构建命令字符串，格式：S COMMAND_NAME(param1,param2,...)
                cmd_str = f"S {command}({','.join(param_list)})"
                
                # 发送命令
                data = cmd_str.encode('utf-8')
                self.socket.sendto(data, (self.address, self.port))
                logger.debug(f"发送UDP命令: {cmd_str}")
                
                # 根据命令类型决定是否等待响应
                if command in self._immediate_response_commands:
                    # 立即响应命令，设置合理的超时时间
                    try:
                        self.socket.settimeout(5.0)
                        response_data, _ = self.socket.recvfrom(1024)
                        
                        # 尝试解析响应，假设响应仍然是JSON格式
                        try:
                            response = json.loads(response_data.decode('utf-8'))
                            logger.debug(f"收到UDP响应: {response}")
                        except json.JSONDecodeError:
                            # 如果响应不是JSON格式，返回成功状态
                            response = {"status": "success", "message": "命令执行成功"}
                            logger.debug(f"收到UDP响应: {response_data.decode('utf-8')}")
                        
                        # 恢复监听线程的超时时间
                        self.socket.settimeout(2)
                        return response
                    except socket.timeout:
                        logger.error(f"UDP命令超时: {command}")
                        # 恢复监听线程的超时时间
                        self.socket.settimeout(2)
                        return {"status": "error", "message": "命令超时"}
                elif command in self._long_running_commands:
                    # 长时间运行命令，发送后立即返回成功，不等待响应
                    # 服务器会通过状态更新推送执行结果
                    logger.debug(f"长时间运行命令已发送，等待状态更新: {command}")
                    # 恢复监听线程的超时时间
                    self.socket.settimeout(2)
                    return {"status": "success", "message": "命令已接收，正在执行", "type": "async"}
                else:
                    # 未知命令类型，默认按立即响应处理
                    try:
                        self.socket.settimeout(2.0)
                        response_data, _ = self.socket.recvfrom(1024)
                        
                        # 尝试解析响应
                        try:
                            response = json.loads(response_data.decode('utf-8'))
                            logger.debug(f"收到UDP响应: {response}")
                        except json.JSONDecodeError:
                            response = {"status": "success", "message": "命令执行成功"}
                            logger.debug(f"收到UDP响应: {response_data.decode('utf-8')}")
                        
                        # 恢复监听线程的超时时间
                        self.socket.settimeout(2)
                        return response
                    except socket.timeout:
                        logger.error(f"UDP命令超时: {command}")
                        # 恢复监听线程的超时时间
                        self.socket.settimeout(2)
                        return {"status": "error", "message": "命令超时"}
            except Exception as e:
                logger.error(f"UDP命令执行失败: {command}, 错误: {e}")
                # 恢复监听线程的超时时间
                self.socket.settimeout(2)
                return {"status": "error", "message": str(e)}


class ResinWorkstation(WorkstationBase):
    """
    Resin工作站驱动类
    """
    def __init__(self, config: dict = None, deck=None, address: str = "127.0.0.1",
                 port: int = 8889, debug_mode: bool = False, *args, **kwargs):
        if deck is None:
            if config and 'deck' in config:
                deck = config.get('deck')
            else:
                # 创建默认的树脂工作站台面
                deck = ResinWorkstationDeck()
                # 初始化试剂架
                deck.initialize_reagent_racks()
        else:
            # 如果提供了deck，确保试剂架已经初始化
            if isinstance(deck, ResinWorkstationDeck):
                if not deck.reaction_reagent_rack or not deck.post_process_reagent_rack:
                    logger.info("提供的deck对象试剂架未初始化，正在初始化...")
                    deck.initialize_reagent_racks()
            else:
                logger.warning(f"提供的deck对象不是ResinWorkstationDeck类型: {type(deck)}, 无法初始化试剂架")
        
        super().__init__(deck=deck, *args, **kwargs)
        self.debug_mode = debug_mode

        # UDP客户端初始化
        self.udp_client = UDPClient(address, port)
        self.connected = False
        
        # 设备状态
        self.success = False
        self.operation_mode = "local"  # local or remote
        
        # 初始化设备状态对象
        self._device_state = DeviceState()
        # 设置UDP客户端的状态更新回调
        self.udp_client.set_status_callback(self._handle_status_update)
        # 初始化连接
        self.connect_device(address, port)
        
        # 初始化物料对象映射
        self._reagent_bottles = {}
        # 同步初始状态
        self._sync_state_to_material_objects()
        
        logger.info("树脂工作站初始化完成")

    def _sync_state_to_material_objects(self):
        """
        将设备状态同步到物料对象
        """
        if not self.deck:
            logger.error("Deck未初始化，无法同步物料状态")
            return
        
        # 遍历所有试剂状态，创建或更新试剂瓶对象
        for reagent_id, reagent_state_data in self._device_state.reagents.items():
            # 检查试剂瓶是否已存在
            if reagent_id not in self._reagent_bottles:
                # 创建新的试剂瓶对象
                reagent_bottle = ReagentBottle(
                    name=f"reagent_bottle_{reagent_id}",
                    size_x=50.0,
                    size_y=50.0,
                    size_z=100.0,
                    reagent_state=ReagentState(**reagent_state_data)
                )
                
                # 根据试剂类别选择对应的试剂架
                category = reagent_state_data.get("category", "reaction")
                rack_name = "reaction_reagent_rack" if category == "reaction" else "post_process_reagent_rack"
                reagent_rack = getattr(self.deck, rack_name, None)
                
                if not reagent_rack:
                    logger.error(f"试剂架 {rack_name} 不存在，无法分配试剂瓶")
                    continue
                
                # 寻找空槽位
                assigned = False
                for row in range(reagent_rack.num_rows):
                    for col in range(reagent_rack.num_cols):
                        if not reagent_rack.get_bottle_by_position(row, col):
                            success = reagent_rack.assign_bottle(reagent_bottle, row, col)
                            if success:
                                logger.info(f"试剂瓶 {reagent_id} 已分配到 {rack_name}({row}, {col})")
                                assigned = True
                                break
                    if assigned:
                        break
                
                if not assigned:
                    logger.error(f"试剂架 {rack_name} 已满，无法分配试剂瓶 {reagent_id}")
                    continue
                
                # 添加到映射
                self._reagent_bottles[reagent_id] = reagent_bottle
                logger.info(f"试剂瓶 {reagent_id} 已成功创建并分配")
            else:
                # 更新现有试剂瓶的状态
                reagent_bottle = self._reagent_bottles[reagent_id]
                old_category = reagent_bottle._unilabos_state.category
                new_category = reagent_state_data.get("category", "reaction")
                
                # 加载新状态
                reagent_bottle.load_state(reagent_state_data)
                logger.debug(f"试剂瓶 {reagent_id} 状态已更新")
                
                # 如果类别发生变化，需要重新分配试剂架
                if old_category != new_category:
                    logger.info(f"试剂 {reagent_id} 类别变化: {old_category} -> {new_category}，重新分配试剂架")
                    # 从旧试剂架中移除
                    self._remove_reagent_bottle_from_rack(reagent_bottle, old_category)
                    # 分配到新试剂架
                    self._assign_reagent_bottle_to_rack(reagent_bottle, new_category)
        
        # 2. 清理不再存在于设备状态中的试剂瓶对象
        current_reagent_ids = set(self._device_state.reagents.keys())
        existing_bottle_ids = set(self._reagent_bottles.keys())
        
        # 找出需要移除的试剂瓶ID
        reagent_ids_to_remove = existing_bottle_ids - current_reagent_ids
        
        for reagent_id in reagent_ids_to_remove:
            reagent_bottle = self._reagent_bottles[reagent_id]
            # 从试剂架中移除试剂瓶
            self._remove_reagent_bottle_from_rack(reagent_bottle)
            # 从映射中移除
            del self._reagent_bottles[reagent_id]
            logger.info(f"移除试剂瓶对象: {reagent_id}")
        
        # 3. 更新ROS资源
        if hasattr(self, '_ros_node'):
            ROS2WorkstationNode.run_async_func(self._ros_node.update_resource, True, **{
                "resources": [self.deck]
            })
    
    def _assign_reagent_bottle_to_rack(self, reagent_bottle, category):
        """
        将试剂瓶分配到相应的试剂架
        
        Args:
            reagent_bottle: 试剂瓶对象
            category: 试剂类别
        """
        rack_name = "reaction_reagent_rack" if category == "reaction" else "post_process_reagent_rack"
        reagent_rack = getattr(self.deck, rack_name, None)
        
        if not reagent_rack:
            logger.error(f"试剂架 {rack_name} 不存在，无法分配试剂瓶")
            return False
        
        # 寻找空槽位
        assigned = False
        for row in range(reagent_rack.num_rows):
            for col in range(reagent_rack.num_cols):
                if not reagent_rack.get_bottle_by_position(row, col):
                    success = reagent_rack.assign_bottle(reagent_bottle, row, col)
                    if success:
                        logger.info(f"试剂瓶 {reagent_bottle._unilabos_state.reagent_id} 已重新分配到 {rack_name}({row}, {col})")
                        assigned = True
                        break
            if assigned:
                break
        
        if not assigned:
            logger.error(f"试剂架 {rack_name} 已满，无法重新分配试剂瓶 {reagent_bottle._unilabos_state.reagent_id}")
            return False
        
        return True
    
    def _remove_reagent_bottle_from_rack(self, reagent_bottle, category=None):
        """
        从试剂架中移除试剂瓶
        
        Args:
            reagent_bottle: 试剂瓶对象
            category: 试剂类别（可选，用于指定试剂架）
        """
        # 如果没有指定类别，使用当前试剂瓶的类别
        if not category:
            category = reagent_bottle._unilabos_state.category
        
        # 确定要检查的试剂架
        rack_names = []
        if category:
            rack_names = ["reaction_reagent_rack" if category == "reaction" else "post_process_reagent_rack"]
        else:
            rack_names = ["reaction_reagent_rack", "post_process_reagent_rack"]
        
        for rack_name in rack_names:
            reagent_rack = getattr(self.deck, rack_name, None)
            if not reagent_rack:
                continue
            
            # 在试剂架中查找试剂瓶并移除
            for row in range(reagent_rack.num_rows):
                for col in range(reagent_rack.num_cols):
                    bottle = reagent_rack.get_bottle_by_position(row, col)
                    if bottle == reagent_bottle:
                        removed_bottle = reagent_rack.remove_bottle(row, col)
                        if removed_bottle:
                            logger.info(f"从试剂架 {rack_name}({row}, {col}) 移除试剂瓶 {reagent_bottle._unilabos_state.reagent_id}")
                        return
    
    def get_reagent_bottle(self, reagent_id: int) -> Optional[ReagentBottle]:
        """
        通过试剂ID获取试剂瓶对象
        
        Args:
            reagent_id: 试剂ID
            
        Returns:
            Optional[ReagentBottle]: 试剂瓶对象，若不存在则返回None
        """
        reagent_bottle = self._reagent_bottles.get(reagent_id)
        if not reagent_bottle:
            logger.debug(f"未找到试剂瓶对象: {reagent_id}")
        return reagent_bottle
    
    def get_reagent_bottle_by_position(self, rack_name: str, row: int, col: int) -> Optional[ReagentBottle]:
        """
        通过位置获取试剂瓶对象
        
        Args:
            rack_name: 试剂架名称 (reaction_reagent_rack 或 post_process_reagent_rack)
            row: 行号
            col: 列号
            
        Returns:
            Optional[ReagentBottle]: 试剂瓶对象，若不存在则返回None
        """
        if not self.deck:
            logger.error("Deck未初始化，无法获取试剂瓶对象")
            return None
        
        reagent_rack = getattr(self.deck, rack_name, None)
        if not reagent_rack:
            logger.error(f"试剂架 {rack_name} 不存在")
            return None
        
        # 验证行号和列号
        if not isinstance(row, int) or not isinstance(col, int):
            logger.error(f"无效的行号或列号类型: row={type(row)}, col={type(col)}")
            return None
        
        if row < 0 or row >= reagent_rack.num_rows or col < 0 or col >= reagent_rack.num_cols:
            logger.error(f"无效的试剂架位置: ({row}, {col}), 试剂架大小: {reagent_rack.num_rows}x{reagent_rack.num_cols}")
            return None
        
        reagent_bottle = reagent_rack.get_bottle_by_position(row, col)
        if not reagent_bottle:
            logger.debug(f"试剂架 {rack_name} 位置 ({row}, {col}) 没有试剂瓶")
        
        return reagent_bottle
    
    def get_reagent_rack(self, rack_name: str) -> Optional[ReagentRack]:
        """
        获取试剂架对象
        
        Args:
            rack_name: 试剂架名称 (reaction_reagent_rack 或 post_process_reagent_rack)
            
        Returns:
            Optional[ReagentRack]: 试剂架对象，若不存在则返回None
        """
        if not self.deck:
            logger.error("Deck未初始化，无法获取试剂架对象")
            return None
        
        reagent_rack = getattr(self.deck, rack_name, None)
        if not reagent_rack:
            logger.error(f"试剂架 {rack_name} 不存在")
        return reagent_rack

    # ====================== 设备连接管理 ======================
    def connect_device(self, address: str = None, port: int = None) -> bool:
        """
        连接设备
        
        Args:
            address: 设备IP地址
            port: 设备端口
            
        Returns:
            bool: 连接成功返回True，否则返回False
        """
        if address:
            self.udp_client.address = address
        if port:
            self.udp_client.port = port
        
        self.connected = self.udp_client.connect()
        
        # 如果连接成功，启动状态监听
        if self.connected:
            self.udp_client.start_listen()
            # 更新设备状态
            self._device_state.connected = True
            self._device_state.operation_mode = self.operation_mode
        
        return self.connected

    def disconnect_device(self) -> bool:
        """
        断开设备连接
        
        Returns:
            bool: 断开成功返回True，否则返回False
        """
        success = self.udp_client.disconnect()
        self.connected = False  # 断开连接后，connected状态应该为False
        
        # 更新设备状态
        self._device_state.connected = False
        self._device_state.device_status = "idle"
        
        return success

    def toggle_local_remote_control(self, mode: str) -> bool:
        """
        切换本地/远程控制模式
        
        Args:
            mode: 控制模式，"local"或"remote"
            
        Returns:
            bool: 切换成功返回True，否则返回False
        """
        if mode not in ["local", "remote"]:
            logger.error(f"无效的控制模式: {mode}")
            return False
        
        try:
            response = self.udp_client.send_command("TOGGLE_LOCAL_REMOTE_CONTROL", {"mode": mode})
            if response.get("status") == "success":
                self.operation_mode = mode
                return True
            else:
                logger.error(f"切换控制模式失败: {response.get('message')}")
                return False
        except Exception as e:
            logger.error(f"切换控制模式异常: {e}")
            return False
    
    # ====================== 状态查询方法 ======================
    def _get_device_state(self) -> DeviceState:
        """
        获取设备整体状态
        
        Returns:
            DeviceState: 设备整体状态对象
        """
        # 发送状态查询命令
        response = self.udp_client.send_command("GET_DEVICE_STATE")
        if response.get("status") == "success":
            # 更新设备状态
            self.update_state(response.get("data", {}))
        
        return self._device_state
    
    def _get_reagent_state(self, reagent_id: int) -> Optional[ReagentState]:
        """
        获取单个试剂状态
        
        Args:
            reagent_id: 试剂编号
            
        Returns:
            Optional[ReagentState]: 试剂状态对象，若不存在则返回None
        """
        # 发送试剂状态查询命令
        response = self.udp_client.send_command("GET_REAGENT_STATE", {"reagent_id": reagent_id})
        if response.get("status") == "success":
            # 更新试剂状态
            self.update_state({"reagents": {reagent_id: response.get("data", {})}})
        
        return self._device_state.reagents.get(reagent_id)
    
    def _get_all_reagents_state(self) -> Dict[int, ReagentState]:
        """
        获取所有试剂状态
        
        Returns:
            Dict[int, ReagentState]: 所有试剂状态字典
        """
        # 发送所有试剂状态查询命令
        response = self.udp_client.send_command("GET_ALL_REAGENTS_STATE")
        if response.get("status") == "success":
            # 更新所有试剂状态
            self.update_state({"reagents": response.get("data", {})})
        
        return self._device_state.reagents
    
    def _get_reactor_state(self, reactor_id: int) -> Optional[ReactorState]:
        """
        获取单个反应器状态
        
        Args:
            reactor_id: 反应器编号
            
        Returns:
            Optional[ReactorState]: 反应器状态对象，若不存在则返回None
        """
        # 发送反应器状态查询命令
        response = self.udp_client.send_command("GET_REACTOR_STATE", {"reactor_id": reactor_id})
        if response.get("status") == "success":
            # 更新反应器状态
            self.update_state({"reactors": {reactor_id: response.get("data", {})}})
        
        return self._device_state.reactors.get(reactor_id)
    
    def _get_post_process_state(self, post_process_id: int) -> Optional[PostProcessState]:
        """
        获取单个后处理系统状态
        
        Args:
            post_process_id: 后处理编号
            
        Returns:
            Optional[PostProcessState]: 后处理系统状态对象，若不存在则返回None
        """
        # 发送后处理状态查询命令
        response = self.udp_client.send_command("GET_POST_PROCESS_STATE", {"post_process_id": post_process_id})
        if response.get("status") == "success":
            # 更新后处理状态
            self.update_state({"post_processes": {post_process_id: response.get("data", {})}})
        
        return self._device_state.post_processes.get(post_process_id)
    
    def update_state(self, state_data: Dict[str, Any]) -> None:
        """
        更新设备状态
        
        Args:
            state_data: 状态数据字典
        """
        if not state_data:
            return
        
        # 更新设备基本状态
        if "connected" in state_data:
            self._device_state.connected = state_data["connected"]
        if "operation_mode" in state_data:
            self._device_state.operation_mode = state_data["operation_mode"]
        if "device_status" in state_data:
            self._device_state.device_status = state_data["device_status"]
        if "error_message" in state_data:
            self._device_state.error_message = state_data["error_message"]
        if "solution_add_status" in state_data:
            self._device_state.solution_add_status = state_data["solution_add_status"]
        if "current_solution_id" in state_data:
            self._device_state.current_solution_id = state_data["current_solution_id"]
        if "current_volume" in state_data:
            self._device_state.current_volume = state_data["current_volume"]
        if "target_volume" in state_data:
            self._device_state.target_volume = state_data["target_volume"]
        if "current_reactor_id" in state_data:
            self._device_state.current_reactor_id = state_data["current_reactor_id"]
        
        # 更新反应器状态
        if "reactors" in state_data:
            for reactor_id, reactor_state_data in state_data["reactors"].items():
                reactor_id = int(reactor_id)
                # 如果反应器不存在，创建新的ReactorState对象
                if reactor_id not in self._device_state.reactors:
                    self._device_state.reactors[reactor_id] = ReactorState(reactor_id=reactor_id)
                
                # 更新反应器状态属性
                reactor = self._device_state.reactors[reactor_id]
                for key, value in reactor_state_data.items():
                    if hasattr(reactor, key):
                        setattr(reactor, key, value)
        
        # 更新后处理系统状态
        if "post_processes" in state_data:
            for post_process_id, post_process_state_data in state_data["post_processes"].items():
                post_process_id = int(post_process_id)
                # 如果后处理系统不存在，创建新的PostProcessState对象
                if post_process_id not in self._device_state.post_processes:
                    self._device_state.post_processes[post_process_id] = PostProcessState(post_process_id=post_process_id)
                
                # 更新后处理系统状态属性
                post_process = self._device_state.post_processes[post_process_id]
                for key, value in post_process_state_data.items():
                    if hasattr(post_process, key):
                        setattr(post_process, key, value)
        
        # 更新试剂状态
        if "reagents" in state_data:
            for reagent_id, reagent_state_data in state_data["reagents"].items():
                reagent_id = int(reagent_id)
                # 如果试剂不存在，创建新的ReagentState对象
                if reagent_id not in self._device_state.reagents:
                    self._device_state.reagents[reagent_id] = ReagentState(
                        reagent_id=reagent_id,
                        name=reagent_state_data.get("name", ""),
                        volume=reagent_state_data.get("volume", 0.0),
                        max_volume=reagent_state_data.get("max_volume", 0.0),
                        concentration=reagent_state_data.get("concentration", "")
                    )
                
                # 更新试剂状态属性
                reagent = self._device_state.reagents[reagent_id]
                for key, value in reagent_state_data.items():
                    if hasattr(reagent, key):
                        setattr(reagent, key, value)
        
        # 更新最后更新时间
        self._device_state.last_updated = time.strftime("%Y-%m-%d %H:%M:%S")
        logger.debug(f"设备状态已更新: {self._device_state}")
        
        # 同步更新物料对象
        self._sync_state_to_material_objects()
    
    def _handle_status_update(self, state_data: Dict[str, Any]) -> None:
        """
        处理UDP服务器主动推送的状态更新
        
        Args:
            state_data: 状态数据字典
        """
        self.update_state(state_data)

    # ====================== 指令集实现 ======================
    def _send_command(self, command: str, params: Dict[str, Any] = None, blocking: bool = False, timeout: float = None) -> bool:
        """
        发送命令的通用方法
        
        Args:
            command: 命令名称
            params: 命令参数
            blocking: 是否阻塞等待命令完成，默认为False
            timeout: 阻塞等待超时时间（秒），默认为None（无限等待）
            
        Returns:
            bool: 命令执行成功返回True，否则返回False
        """
        if not self.connected:
            logger.error("设备未连接，无法发送命令")
            return False
        
        response = self.udp_client.send_command(command, params)
        
        # 对于同步命令，等待并检查响应状态
        if response.get("type") != "async":
            # 更新设备状态（无论命令是否成功，都更新状态）
            self._get_device_state()
            
            if response.get("status") == "success":
                return True
            else:
                logger.error(f"命令执行失败: {command}, 错误: {response.get('message')}")
                return False
        
        # 对于异步命令
        logger.info(f"异步命令已发送: {command}")
        
        if not blocking:
            # 非阻塞模式，发送成功即返回True
            return True
        
        # 阻塞模式，等待命令完成
        logger.info(f"阻塞等待命令完成: {command}")
        
        # 轮询设备状态，直到命令完成或超时
        start_time = time.time()
        while True:
            # 检查是否超时
            if timeout is not None and (time.time() - start_time) > timeout:
                logger.error(f"命令执行超时: {command}")
                return False
            
            # 更新设备状态
            self._get_device_state()
            
            # 检查设备整体状态
            if self._device_state.device_status == "error":
                logger.error(f"设备出错: {self._device_state.error_message}")
                return False
            
            # 检查反应器状态
            if command == "REACTOR_SOLUTION_ADD":
                reactor_id = params.get("reactor_id")
                if reactor_id is not None:
                    reactor_state = self._device_state.reactors.get(reactor_id)
                    if reactor_state and reactor_state.status == "idle" and self._device_state.solution_add_status == "idle":
                        # 反应器状态为idle且溶液添加状态为idle，命令已完成
                        return True
            
            # 检查后处理系统状态
            elif command in ["POST_PROCESS_SOLUTION_ADD", "POST_PROCESS_CLEAN"]:
                post_process_id = params.get("post_process_id", 1)
                post_process_state = self._device_state.post_processes.get(post_process_id)
                if post_process_state and post_process_state.status == "idle":
                    # 后处理系统状态为idle，命令已完成
                    return True
            
            # 短暂休眠，避免频繁查询
            time.sleep(1.0)
    
    # ========== 试剂操作指令集 ==========
    def update_reagent_volume(self, reagent_id: int, volume_change: float, blocking: bool = False, timeout: float = None) -> bool:
        """
        更新试剂体积
        
        Args:
            reagent_id: 试剂编号
            volume_change: 体积变化量（正数为增加，负数为减少）
            blocking: 是否阻塞等待命令完成，默认为False
            timeout: 阻塞等待超时时间（秒），默认为None（无限等待）
            
        Returns:
            bool: 操作成功返回True，否则返回False
        """
        if not self.connected:
            logger.error("设备未连接，无法更新试剂体积")
            return False
        
        # 先获取试剂瓶对象
        reagent_bottle = self.get_reagent_bottle(reagent_id)
        local_success = True
        
        if reagent_bottle:
            # 直接操作试剂瓶对象
            if volume_change > 0:
                local_success = reagent_bottle.add_reagent(volume_change)
            else:
                local_success = reagent_bottle.remove_reagent(abs(volume_change))
            
            if local_success:
                logger.info(f"试剂 {reagent_id} 体积已更新，变化量: {volume_change}")
                
                # 同步更新设备状态中的试剂状态
                if reagent_id in self._device_state.reagents:
                    self._device_state.reagents[reagent_id].volume = reagent_bottle._unilabos_state.volume
                    self._device_state.reagents[reagent_id].status = reagent_bottle._unilabos_state.status
                    logger.debug(f"已同步更新设备状态中的试剂 {reagent_id} 体积和状态")
            else:
                logger.error(f"无法更新试剂 {reagent_id} 体积，变化量: {volume_change}")
        else:
            logger.error(f"未找到试剂瓶对象: {reagent_id}")
            local_success = False
        
        # 发送命令到设备
        params = {
            "reagent_id": reagent_id,
            "volume_change": volume_change
        }
        
        device_success = self._send_command("UPDATE_REAGENT_VOLUME", params, blocking=blocking, timeout=timeout)
        
        # 如果设备命令成功，但本地更新失败，重新同步状态
        if device_success and not local_success:
            logger.info(f"设备更新试剂 {reagent_id} 体积成功，正在重新同步状态")
            self._get_reagent_state(reagent_id)
        
        return local_success and device_success
    
    def transfer_reagent_bottle(self, reagent_id: int, from_rack: str, from_row: int, from_col: int, 
                               to_rack: str, to_row: int, to_col: int) -> bool:
        """
        转移试剂瓶从一个试剂架到另一个试剂架
        
        Args:
            reagent_id: 试剂ID
            from_rack: 源试剂架名称
            from_row: 源行号
            from_col: 源列号
            to_rack: 目标试剂架名称
            to_row: 目标行号
            to_col: 目标列号
            
        Returns:
            bool: 转移成功返回True，否则返回False
        """
        logger.info(f"开始转移试剂瓶 {reagent_id} 从 {from_rack}({from_row}, {from_col}) 到 {to_rack}({to_row}, {to_col})")
        
        # 获取源试剂架和目标试剂架
        source_rack = self.get_reagent_rack(from_rack)
        target_rack = self.get_reagent_rack(to_rack)
        
        if not source_rack:
            logger.error(f"源试剂架 {from_rack} 不存在")
            return False
        
        if not target_rack:
            logger.error(f"目标试剂架 {to_rack} 不存在")
            return False
        
        # 获取源试剂瓶
        source_bottle = source_rack.get_bottle_by_position(from_row, from_col)
        if not source_bottle:
            logger.error(f"源位置 {from_rack}({from_row}, {from_col}) 没有试剂瓶")
            return False
        
        # 检查源试剂瓶的ID是否与请求的ID匹配
        if source_bottle._unilabos_state.reagent_id != reagent_id:
            logger.error(f"源位置 {from_rack}({from_row}, {from_col}) 中的试剂瓶ID {source_bottle._unilabos_state.reagent_id} 与请求的ID {reagent_id} 不匹配")
            return False
        
        # 检查目标位置是否为空
        if target_rack.get_bottle_by_position(to_row, to_col):
            logger.error(f"目标位置 {to_rack}({to_row}, {to_col}) 已被占用")
            return False
        
        # 验证试剂瓶类别与目标试剂架类型是否匹配
        reagent_category = source_bottle._unilabos_state.category
        expected_rack_type = "reaction" if reagent_category == "reaction" else "post_process"
        actual_rack_type = "reaction" if "reaction" in to_rack.lower() else "post_process"
        
        if expected_rack_type != actual_rack_type:
            logger.error(f"试剂瓶类别 {reagent_category} 与目标试剂架类型不匹配，应使用 {expected_rack_type}_reagent_rack")
            return False
        
        # 从源试剂架移除试剂瓶
        removed_bottle = source_rack.remove_bottle(from_row, from_col)
        if not removed_bottle:
            logger.error(f"无法从源位置 {from_rack}({from_row}, {from_col}) 移除试剂瓶")
            return False
        
        # 将试剂瓶添加到目标试剂架
        success = target_rack.assign_bottle(removed_bottle, to_row, to_col)
        if not success:
            logger.error(f"无法将试剂瓶 {reagent_id} 添加到目标位置 {to_rack}({to_row}, {to_col})")
            # 尝试将试剂瓶放回源位置
            revert_success = source_rack.assign_bottle(removed_bottle, from_row, from_col)
            if revert_success:
                logger.info(f"已将试剂瓶 {reagent_id} 放回源位置 {from_rack}({from_row}, {from_col})")
            else:
                logger.error(f"无法将试剂瓶 {reagent_id} 放回源位置 {from_rack}({from_row}, {from_col})")
            return False
        
        # 更新试剂瓶映射
        self._reagent_bottles[reagent_id] = removed_bottle
        
        logger.info(f"试剂瓶 {reagent_id} 已成功从 {from_rack}({from_row}, {from_col}) 转移到 {to_rack}({to_row}, {to_col})")
        
        # 更新ROS资源
        if hasattr(self, '_ros_node'):
            ROS2WorkstationNode.run_async_func(self._ros_node.update_resource, True, **{
                "resources": [self.deck]
            })
        
        return True
    def get_reagent_info(self, reagent_id: int) -> Optional[Dict[str, Any]]:
        """
        获取试剂信息
        
        Args:
            reagent_id: 试剂编号
            
        Returns:
            Optional[Dict[str, Any]]: 试剂信息字典，若不存在则返回None
        """
        # 直接从试剂瓶对象获取信息
        reagent_bottle = self.get_reagent_bottle(reagent_id)
        if reagent_bottle:
            return reagent_bottle.serialize_state()
        
        # 如果没有找到试剂瓶对象，回退到使用设备状态
        reagent_state = self._get_reagent_state(reagent_id)
        if reagent_state:
            return {
                "reagent_id": reagent_state.reagent_id,
                "name": reagent_state.name,
                "volume": reagent_state.volume,
                "max_volume": reagent_state.max_volume,
                "concentration": reagent_state.concentration,
                "category": reagent_state.category,
                "status": reagent_state.status
            }
        return None
    
    def get_all_reagents_info(self) -> Dict[int, Dict[str, Any]]:
        """
        获取所有试剂信息
        
        Returns:
            Dict[int, Dict[str, Any]]: 所有试剂信息字典
        """
        # 优先从试剂瓶对象获取信息
        if self._reagent_bottles:
            return {
                reagent_id: reagent_bottle.serialize_state()
                for reagent_id, reagent_bottle in self._reagent_bottles.items()
            }
        
        # 如果没有试剂瓶对象，回退到使用设备状态
        reagents_state = self._get_all_reagents_state()
        return {
            reagent_id: {
                "reagent_id": reagent_state.reagent_id,
                "name": reagent_state.name,
                "volume": reagent_state.volume,
                "max_volume": reagent_state.max_volume,
                "concentration": reagent_state.concentration,
                "category": reagent_state.category,
                "status": reagent_state.status
            }
            for reagent_id, reagent_state in reagents_state.items()
        }

    # ========== 移液操作指令集 ==========
    def reactor_solution_add(self, solution_id: int, volume: float, reactor_id: int, blocking: bool = False, timeout: float = None) -> bool:
        """
        向反应器添加溶液
        
        Args:
            solution_id: 溶液编号
            volume: 加入体积
            reactor_id: 反应器编号
            blocking: 是否阻塞等待命令完成，默认为False
            timeout: 阻塞等待超时时间（秒），默认为None（无限等待）
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "solution_id": solution_id,
            "volume": volume,
            "reactor_id": reactor_id
        }
        return self._send_command("REACTOR_SOLUTION_ADD", params, blocking=blocking, timeout=timeout)

    def post_process_solution_add(
        self, start_bottle: str, end_bottle: str, volume: float,
        inject_speed: float, suck_speed: float = 4.0, blocking: bool = False,
        timeout: float = None
    ) -> bool:
        """
        后处理溶液转移
        
        Args:
            start_bottle: 出发瓶
            end_bottle: 终点瓶
            volume: 加入体积
            inject_speed: 注入速度
            suck_speed: 吸入速度，默认4.0
            blocking: 是否阻塞等待命令完成，默认为False
            timeout: 阻塞等待超时时间（秒），默认为None（无限等待）
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "start_bottle": start_bottle,
            "end_bottle": end_bottle,
            "volume": volume,
            "inject_speed": inject_speed,
            "suck_speed": suck_speed
        }
        return self._send_command("POST_PROCESS_SOLUTION_ADD", params, blocking=blocking, timeout=timeout)

    def post_process_clean(self, post_process_id: int, blocking: bool = False, timeout: float = None) -> bool:
        """
        自动清洗程序
        
        Args:
            post_process_id: 后处理编号
            blocking: 是否阻塞等待命令完成，默认为False
            timeout: 阻塞等待超时时间（秒），默认为None（无限等待）
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "post_process_id": post_process_id
        }
        return self._send_command("POST_PROCESS_CLEAN", params, blocking=blocking, timeout=timeout)

    # ========== 反应器操作指令集 ==========
    def reactor_n2_on(self, reactor_id: int) -> bool:
        """
        打开反应器氮气
        
        Args:
            reactor_id: 反应器编号
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "reactor_id": reactor_id
        }
        return self._send_command("REACTOR_N2_ON", params)

    def reactor_n2_off(self, reactor_id: int) -> bool:
        """
        关闭反应器氮气
        
        Args:
            reactor_id: 反应器编号
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "reactor_id": reactor_id
        }
        return self._send_command("REACTOR_N2_OFF", params)

    def reactor_air_on(self, reactor_id: int) -> bool:
        """
        打开反应器空气
        
        Args:
            reactor_id: 反应器编号
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "reactor_id": reactor_id
        }
        return self._send_command("REACTOR_AIR_ON", params)

    def reactor_air_off(self, reactor_id: int) -> bool:
        """
        关闭反应器空气
        
        Args:
            reactor_id: 反应器编号
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "reactor_id": reactor_id
        }
        return self._send_command("REACTOR_AIR_OFF", params)

    def temp_set(self, reactor_id: int, temperature: float) -> bool:
        """
        设置温度
        
        Args:
            reactor_id: 反应器编号
            temperature: 温度
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "reactor_id": reactor_id,
            "temperature": temperature
        }
        return self._send_command("TEMP_SET", params)

    # ========== 搅拌器操作指令集 ==========
    def start_reactor_stirrer(self, reactor_id: int, speed: float) -> bool:
        """
        启动反应器搅拌器
        
        Args:
            reactor_id: 反应器编号
            speed: 转速
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "reactor_id": reactor_id,
            "speed": speed
        }
        return self._send_command("START_STIR", params)

    def stop_reactor_stirrer(self, reactor_id: int) -> bool:
        """
        停止反应器搅拌器
        
        Args:
            reactor_id: 反应器编号
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "reactor_id": reactor_id
        }
        return self._send_command("STOP_STIR", params)

    # ========== 后处理排液操作指令集 ==========
    def post_process_discharge_on(self, post_process_id: int) -> bool:
        """
        打开后处理排液
        
        Args:
            post_process_id: 后处理编号
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "post_process_id": post_process_id
        }
        return self._send_command("POST_PROCESS_DISCHARGE_ON", params)

    def post_process_discharge_off(self, post_process_id: int) -> bool:
        """
        关闭后处理排液
        
        Args:
            post_process_id: 后处理编号
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "post_process_id": post_process_id
        }
        return self._send_command("POST_PROCESS_DISCHARGE_OFF", params)

    # ========== 其他指令 ==========
    def wait(self, seconds: int, blocking: bool = False, timeout: float = None) -> bool:
        """
        等待指定时间
        
        Args:
            seconds: 等待时间（秒）
            blocking: 是否阻塞等待命令完成，默认为False
            timeout: 阻塞等待超时时间（秒），默认为None（无限等待）
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        params = {
            "seconds": seconds
        }
        # 如果是wait命令，阻塞等待时间应该至少比seconds大
        if blocking and timeout is None:
            timeout = seconds + 10  # 增加10秒的缓冲时间
        return self._send_command("WAIT", params, blocking=blocking, timeout=timeout)

    # ====================== 设备状态查询 ======================
    @property
    def device_status(self) -> Dict[str, Any]:
        """
        获取设备状态（返回缓存的状态，不主动查询设备）
        
        Returns:
            Dict[str, Any]: 设备状态信息
        """
        # 将DeviceState对象转换为字典
        status_dict = asdict(self._device_state)
        
        # 添加额外的设备信息
        status_dict.update({
            "address": self.udp_client.address,
            "port": self.udp_client.port
        })
        
        # 如果是调试模式，覆盖相关状态
        if self.debug_mode:
            status_dict.update({
                "status": "debug",
                "connected": True
            })
        else:
            # 更新连接状态
            status_dict["connected"] = self.connected
        
        return status_dict
    
    def get_latest_device_status(self) -> Dict[str, Any]:
        """
        获取最新设备状态（主动查询设备）
        
        Returns:
            Dict[str, Any]: 最新设备状态信息
        """
        # 获取最新设备状态
        self._get_device_state()
        return self.device_status
