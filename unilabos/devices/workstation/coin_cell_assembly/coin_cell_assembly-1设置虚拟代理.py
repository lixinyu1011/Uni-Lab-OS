import csv
import json
import os
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional
from pylabrobot.resources import Resource as PLRResource
from unilabos_msgs.msg import Resource
from unilabos.device_comms.modbus_plc.client import ModbusTcpClient
from unilabos.devices.workstation.coin_cell_assembly.button_battery_station import MaterialHole, MaterialPlate
from unilabos.devices.workstation.workstation_base import ResourceSynchronizer, WorkstationBase
from unilabos.device_comms.modbus_plc.client import TCPClient, ModbusNode, PLCWorkflow, ModbusWorkflow, WorkflowAction, BaseClient
from unilabos.device_comms.modbus_plc.modbus import DeviceType, Base as ModbusNodeBase, DataType, WorderOrder
from unilabos.devices.workstation.coin_cell_assembly.button_battery_station import CoincellDeck, Battery

class CoinCellAssemblyWorkstation(WorkstationBase):
    def __init__(
        self,
        station_resource: CoincellDeck,
        address: str = "192.168.1.20",
        port: str = "502",
        debug_mode: bool = False,
        *args,
        **kwargs,
    ):
        super().__init__(
            station_resource=station_resource,
            *args,
            **kwargs,
        )
        self.debug_mode = debug_mode
        self.station_resource = station_resource
        """ 连接初始化 """
        modbus_client = TCPClient(addr=address, port=port)
        print("modbus_client", modbus_client)
        if not debug_mode:
            modbus_client.client.connect()
            count = 100
            while count >0:
                count -=1
                if modbus_client.client.is_socket_open():
                    break
                # time.sleep(2)
            if not modbus_client.client.is_socket_open():
                raise ValueError('modbus tcp connection failed')
        else:
            print("测试模式，跳过连接")

        """ 工站的配置 """
        self.nodes = BaseClient.load_csv(os.path.join(os.path.dirname(__file__), 'coin_cell_assembly_a.csv'))
        self.client  = modbus_client.register_node_list(self.nodes)
        self.success = False
        self.allow_data_read = False  #允许读取函数运行标志位
        self.csv_export_thread = None
        self.csv_export_running = False
        self.csv_expoart_file = None

    # 批量操作在这里写
    async def change_hole_sheet_to_2(self, hole: MaterialHole):
        hole._unilabos_state["max_sheets"] = 2
        return await self._ros_node.update_resource(hole)


    async def fill_plate(self):
        plate_1: MaterialPlate = self.station_resource.children[0].children[0]
        #plate_1
        return await self._ros_node.update_resource(plate_1)

    #def run_assembly(self, wf_name: str, resource: PLRResource, params: str = "\{\}"):
    #    """启动工作流"""
    #    self.current_workflow_status = WorkflowStatus.RUNNING
    #    logger.info(f"工作站 {self.device_id} 启动工作流: {wf_name}")
#
    #    # TODO: 实现工作流逻辑
#
    #    anode_sheet = self.deck.get_resource("anode_sheet")

    # ====================== 命令类指令（COIL_x_） ======================
    """ Action逻辑代码 """
    def _sys_start_cmd(self, cmd):
        """设备启动命令 (可读写)"""
        if cmd is not None:  # 写入模式
            self.success = False
            node = self.client.use_node('COIL_SYS_START_CMD')
            ret = node.write(cmd)
            print(ret)
            self.success = True
            return self.success
        else:  # 读取模式
            cmd_feedback, read_err =  self.client.use_node('COIL_SYS_START_CMD').read(1)
            return cmd_feedback[0]

    def _sys_stop_cmd(self, cmd=None):
        """设备停止命令 (可读写)"""
        if cmd is not None:  # 写入模式
            self.success = False
            node = self.client.use_node('COIL_SYS_STOP_CMD')
            node.write(cmd)
            self.success = True
            return self.success
        else:  # 读取模式
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_STOP_CMD').read(1)
            return cmd_feedback[0]

    def _sys_reset_cmd(self, cmd=None):
        """设备复位命令 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_SYS_RESET_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_RESET_CMD').read(1)
            return cmd_feedback[0]

    def _sys_hand_cmd(self, cmd=None):
        """手动模式命令 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_SYS_HAND_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_HAND_CMD').read(1)
            return cmd_feedback[0]

    def _sys_auto_cmd(self, cmd=None):
        """自动模式命令 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_SYS_AUTO_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_AUTO_CMD').read(1)
            return cmd_feedback[0]

    def _sys_init_cmd(self, cmd=None):
        """初始化命令 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_SYS_INIT_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_INIT_CMD').read(1)
            return cmd_feedback[0]

    def _unilab_send_msg_succ_cmd(self, cmd=None):
        """UNILAB发送配方完毕 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL__unilab_send_msg_SUCC_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL__unilab_send_msg_SUCC_CMD').read(1)
            return cmd_feedback[0]

    def _unilab_rec_msg_succ_cmd(self, cmd=None):
        """UNILAB接收测试电池数据完毕 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_UNILAB_REC_MSG_SUCC_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_UNILAB_REC_MSG_SUCC_CMD').read(1)
            return cmd_feedback

    def func_pack_device_init(self):
        # 切换手动模式
        self._sys_hand_cmd(True)
        while self._sys_hand_status() == False:
            print("waiting for hand_cmd")
            # time.sleep(1)
        #设备初始化
        print("func_pack_device_init")
        self._sys_init_cmd(True)
        while self._sys_init_status() == False:
            print("waiting for init_cmd")
        #手动按钮置回False
        self._sys_hand_cmd(False)
        while self._sys_hand_cmd() == True:
            print("waiting for hand_cmd to False")
        #初始化命令置回False
        self._sys_init_cmd(False)
        while self._sys_init_cmd() == True:
            print("waiting for init_cmd to False")

    def func_pack_device_auto(self):
        #切换自动
        self._sys_auto_cmd(True)
        while self._sys_auto_status() == False:
            print("waiting for auto_status")
        #自动按钮置False
        self._sys_auto_cmd(False)
        while self._sys_auto_cmd() == True:
            print("waiting for auto_cmd")

    def func_pack_device_start(self):
        #切换自动
        self._sys_start_cmd(True)
        while self._sys_start_status() == False:
            print("waiting for start_status")
        #自动按钮置False
        self._sys_start_cmd(False)
        while self._sys_start_cmd() == True:
            print("waiting for start_cmd")     

    
    def func_pack_device_stop(self):
        #Auto模式
        while self._sys_auto_status() == False: 
            print("wait for auto_status")
        self._sys_stop_cmd(True)
        while self._sys_stop_status() == False:
            print("waiting for stop_status")
        #自动按钮置False
        self._sys_stop_cmd(False)
        while self._sys_stop_cmd() == True:
            print("waiting for stop_cmd")

  # ====================== 命令类指令（REG_x_） ======================
    def _unilab_send_msg_electrolyte_num(self, num=None):
        """UNILAB写电解液使用瓶数(可读写)"""
        if num is not None:
            self.success = False
            ret = self.client.use_node('REG_MSG_ELECTROLYTE_NUM').write(num)
            print(ret)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('REG_MSG_ELECTROLYTE_NUM').read(1)
            return cmd_feedback[0]

    def _unilab_send_msg_electrolyte_use_num(self, use_num=None):
        """UNILAB写单次电解液使用瓶数(可读写)"""
        if use_num is not None:
            self.success = False
            self.client.use_node('REG_MSG_ELECTROLYTE_USE_NUM').write(use_num)
            self.success = True
            return self.success
        else:
            return False

    def _unilab_send_msg_assembly_type(self, num=None):
        """UNILAB写组装参数"""
        if num is not None:
            self.success = False
            self.client.use_node('REG_MSG_ASSEMBLY_TYPE').write(num)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('REG_MSG_ASSEMBLY_TYPE').read(1)
            return cmd_feedback[0]

    def _unilab_send_msg_electrolyte_vol(self, vol=None):
        """UNILAB写电解液吸取量参数"""
        if vol is not None:
            self.success = False
            self.client.use_node('REG_MSG_ELECTROLYTE_VOLUME').write(vol, data_type=DataType.FLOAT32, word_order=WorderOrder.LITTLE)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('REG_MSG_ELECTROLYTE_VOLUME').read(2, word_order=WorderOrder.LITTLE)
            return cmd_feedback[0]

    def _unilab_send_msg_assembly_pressure(self, vol=None):
        """UNILAB写电池压制力"""
        if vol is not None:
            self.success = False
            self.client.use_node('REG_MSG_ASSEMBLY_PRESSURE').write(vol, data_type=DataType.FLOAT32, word_order=WorderOrder.LITTLE)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('REG_MSG_ASSEMBLY_PRESSURE').read(2, word_order=WorderOrder.LITTLE)
            return cmd_feedback[0]
    # 下发参数
    def func_pack_send_msg_cmd(self, experiment_params=None):
        """UNILAB写参数"""
        while self.request_rec_msg_status() == False: 
            print("wait for res_msg")
            # time.sleep(1)
        if experiment_params is not None:
            self.success = False
            print(experiment_params)
            data = json.loads(experiment_params)
            print(data)
            self._unilab_send_msg_electrolyte_num(data['elec_num'])
            # time.sleep(1)
            self._unilab_send_msg_electrolyte_use_num(data['elec_use_num'])
            # time.sleep(1)
            self._unilab_send_msg_electrolyte_vol(data['elec_vol'])
            # time.sleep(1)
            self._unilab_send_msg_assembly_type(data['assembly_type'])
            # time.sleep(1)
            self._unilab_send_msg_assembly_pressure(data['assembly_pressure'])
            # time.sleep(1)
            self._unilab_send_msg_succ_cmd(True)
            # time.sleep(1)   
            while self.request_rec_msg_status() == True: 
                print("wait for res_msg")
                # time.sleep(1)
            self._unilab_send_msg_succ_cmd(False)
            self.success = True
            return self.success
        else:
            return False
        
    #启动读取线程
    def func_pack_read_data_cmd(self, file_path: str):
        if self._sys_auto_status and self._sys_start_status:
            self.csv_export_thread = threading.Thread(target=self.func_read_data_and_output(), args=(file_path)) # !!!!!!!!!!!!!!!!!!!!!!!!!!
            self.csv_export_thread.daemon = True
            self.csv_export_thread.start()

    # ==================== 状态类属性（COIL_x_STATUS） ====================
    def _sys_start_status(self) -> bool:
        """设备启动中( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_START_STATUS').read(1)
        return status[0]

    def _sys_stop_status(self) -> bool:
        """设备停止中( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_STOP_STATUS').read(1)
        return status[0]

    def _sys_reset_status(self) -> bool:
        """设备复位中( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_RESET_STATUS').read(1)
        return status[0]

    def _sys_init_status(self) -> bool:
        """设备初始化完成( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_INIT_STATUS').read(1)
        return status[0]
    
    def _sys_hand_status(self) -> bool:
        """设备手动模式( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_HAND_STATUS').read(1)
        return status[0]

    def _sys_auto_status(self) -> bool:
        """设备自动模式( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_AUTO_STATUS').read(1)
        return status[0]

    @property
    def sys_status(self) -> str:
        if self.debug_mode:
            return "设备调试模式"
        if self._sys_start_status():
            return "设备启动中"
        elif self._sys_stop_status():
            return "设备停止中"
        elif self._sys_reset_status():
            return "设备复位中"
        elif self._sys_init_status():
            return "设备初始化中"
        else:
            return "未知状态"

    @property
    def sys_mode(self) -> str:
        if self.debug_mode:
            return "设备调试模式"
        if self._sys_hand_status():
            return "设备手动模式"
        elif self._sys_auto_status():
            return "设备自动模式"
        else:
            return "未知模式"

    @property
    def request_rec_msg_status(self) -> bool:
        """设备请求接受配方( BOOL)"""
        if self.debug_mode:
            return True
        status, read_err = self.client.use_node('COIL_REQUEST_REC_MSG_STATUS').read(1)
        return status[0]

    @property
    def request_send_msg_status(self) -> bool:
        """设备请求发送测试数据( BOOL)"""
        if self.debug_mode:
            return True
        status, read_err = self.client.use_node('COIL_REQUEST_SEND_MSG_STATUS').read(1)
        return status[0]

    # ======================= 其他属性（特殊功能） ========================
    '''
    @property
    def warning_1(self) -> bool:
        status, read_err = self.client.use_node('COIL_WARNING_1').read(1)
        return status[0]
    '''
    # ===================== 生产数据区 ======================
    
    @property
    def data_assembly_coin_cell_num(self) -> int:
        """已完成电池数量 (INT16)"""
        if self.debug_mode:
            return 0
        num, read_err = self.client.use_node('REG_DATA_ASSEMBLY_COIN_CELL_NUM').read(1)
        return num

    @property
    def data_assembly_time(self) -> float:
        """单颗电池组装时间 (秒, REAL/FLOAT32)"""
        if self.debug_mode:
            return 0
        time, read_err =  self.client.use_node('REG_DATA_ASSEMBLY_PER_TIME').read(2, word_order=WorderOrder.LITTLE)
        return time

    @property
    def data_open_circuit_voltage(self) -> float:
        """开路电压值 (FLOAT32)"""
        if self.debug_mode:
            return 0
        vol, read_err =  self.client.use_node('REG_DATA_OPEN_CIRCUIT_VOLTAGE').read(2, word_order=WorderOrder.LITTLE)
        return vol

    @property
    def data_axis_x_pos(self) -> float:
        """分液X轴当前位置 (FLOAT32)"""
        if self.debug_mode:
            return 0
        pos, read_err =  self.client.use_node('REG_DATA_AXIS_X_POS').read(2, word_order=WorderOrder.LITTLE)
        return pos

    @property
    def data_axis_y_pos(self) -> float:
        """分液Y轴当前位置 (FLOAT32)"""
        if self.debug_mode:
            return 0
        pos, read_err =  self.client.use_node('REG_DATA_AXIS_Y_POS').read(2, word_order=WorderOrder.LITTLE)
        return pos

    @property
    def data_axis_z_pos(self) -> float:
        """分液Z轴当前位置 (FLOAT32)"""
        if self.debug_mode:
            return 0
        pos, read_err =  self.client.use_node('REG_DATA_AXIS_Z_POS').read(2, word_order=WorderOrder.LITTLE)
        return pos

    @property
    def data_pole_weight(self) -> float:
        """当前电池正极片称重数据 (FLOAT32)"""
        if self.debug_mode:
            return 0
        weight, read_err =  self.client.use_node('REG_DATA_POLE_WEIGHT').read(2, word_order=WorderOrder.LITTLE)
        return weight

    @property
    def data_assembly_pressure(self) -> int:
        """当前电池压制力 (INT16)"""
        if self.debug_mode:
            return 0
        pressure, read_err = self.client.use_node('REG_DATA_ASSEMBLY_PRESSURE').read(1)
        return pressure

    @property
    def data_electrolyte_volume(self) -> int:
        """当前电解液加注量 (INT16)"""
        if self.debug_mode:
            return 0
        vol, read_err = self.client.use_node('REG_DATA_ELECTROLYTE_VOLUME').read(1)
        return vol

    @property
    def data_coin_num(self) -> int:
        """当前电池数量 (INT16)"""
        if self.debug_mode:
            return 0
        num, read_err = self.client.use_node('REG_DATA_COIN_NUM').read(1)
        return num

    @property
    def data_coin_cell_code(self) -> str:
        """电池二维码序列号 (STRING)"""
        try:
            # 尝试不同的字节序读取
            code_little, read_err = self.client.use_node('REG_DATA_COIN_CELL_CODE').read(10, word_order=WorderOrder.LITTLE)
            print(code_little)
            clean_code = code_little[-8:][::-1]
            return clean_code
        except Exception as e:
            print(f"读取电池二维码失败: {e}")
            return "N/A"

    @property
    def data_electrolyte_code(self) -> str:
        """电解液二维码序列号 (STRING)"""
        try:
            # 尝试不同的字节序读取
            code_little, read_err = self.client.use_node('REG_DATA_ELECTROLYTE_CODE').read(10, word_order=WorderOrder.LITTLE)
            print(code_little)
            clean_code = code_little[-8:][::-1]
            return clean_code
        except Exception as e:
            print(f"读取电解液二维码失败: {e}")
            return "N/A"

#感觉像是AI写的

    # @property
    # def data_coin_cell_code(self) -> str:
    #     """电池二维码序列号 (STRING)"""
    #     if self.debug_mode:
    #         return "N/A"
    #     try:
    #         # 尝试不同的字节序读取
    #         code_little, read_err = self.client.use_node('REG_DATA_COIN_CELL_CODE').read(10, word_order=WorderOrder.LITTLE)
            
    #         # 安全处理各种数据类型
    #         if isinstance(code_little, bytes):
    #             # 尝试多种编码方式
    #             for encoding in ['utf-8', 'ascii', 'latin-1']:
    #                 try:
    #                     clean_code = code_little.decode(encoding, errors='ignore')
    #                     break
    #                 except:
    #                     continue
    #             else:
    #                 # 如果所有编码都失败，转换为十六进制字符串
    #                 clean_code = code_little.hex()
    #         elif isinstance(code_little, str):
    #             clean_code = code_little
    #         else:
    #             clean_code = str(code_little)
            
    #         # 清理空字符和控制字符
    #         clean_code = clean_code.replace('\x00', '').strip()
    #         # 过滤掉非打印字符
    #         clean_code = ''.join(char for char in clean_code if char.isprintable())
            
    #         # 如果清理后为空，返回默认值
    #         if not clean_code:
    #             return "N/A"
            
    #         # 尝试字节重排以修复字节序问题（与电解液码使用相同逻辑）
    #         if len(clean_code) >= 8 and clean_code.startswith('00000') and clean_code.endswith('00000'):
    #             # 提取中间的有效部分并重新排列
    #             middle_part = clean_code[5:-5]
    #             if len(middle_part) >= 4:
    #                 # 重新排列字节，保持与电解液码相同的处理逻辑
    #                 reordered = middle_part[2:4] + '0000' + middle_part[0:2] + '5'
    #                 return reordered
            
    #         # 如果不符合特殊模式，直接返回清理后的代码
    #         return clean_code
    #     except Exception as e:
    #         print(f"读取电池二维码失败: {e}")
    #         return "N/A"


    # @property
    # def data_electrolyte_code(self) -> str:
    #     """电解液二维码序列号 (STRING)"""
    #     if self.debug_mode:
    #         return "N/A"
    #     try:
    #         # 尝试不同的字节序读取
    #         code_little, read_err = self.client.use_node('REG_DATA_ELECTROLYTE_CODE').read(10, word_order=WorderOrder.LITTLE)
            
    #         # 安全处理各种数据类型
    #         if isinstance(code_little, bytes):
    #             # 尝试多种编码方式
    #             for encoding in ['utf-8', 'ascii', 'latin-1']:
    #                 try:
    #                     clean_code = code_little.decode(encoding, errors='ignore')
    #                     break
    #                 except:
    #                     continue
    #             else:
    #                 # 如果所有编码都失败，转换为十六进制字符串
    #                 clean_code = code_little.hex()
    #         elif isinstance(code_little, str):
    #             clean_code = code_little
    #         else:
    #             clean_code = str(code_little)
            
    #         # 清理空字符和控制字符
    #         clean_code = clean_code.replace('\x00', '').strip()
    #         # 过滤掉非打印字符
    #         clean_code = ''.join(char for char in clean_code if char.isprintable())
            
    #         # 如果清理后为空，返回默认值
    #         if not clean_code:
    #             return "N/A"
            
    #         # 尝试字节重排以修复字节序问题
    #         # 如果数据看起来像是字节序错误（如00000DB5700000DB应该是BD0000075）
    #         if len(clean_code) >= 8 and clean_code.startswith('00000') and clean_code.endswith('00000'):
    #             # 提取中间的有效部分并重新排列
    #             middle_part = clean_code[5:-5]  # 提取DB57
    #             if len(middle_part) >= 4:
    #                 # 重新排列字节：DB57 -> BD + 0000075 的逻辑
    #                 # 根据用户反馈，00000DB5700000DB 应该是 BD0000075
    #                 # 尝试提取和重排
    #                 reordered = middle_part[2:4] + '0000' + middle_part[0:2] + '5'
    #                 return reordered
            
    #         # 如果不符合特殊模式，直接返回清理后的代码
    #         return clean_code
    #     except Exception as e:
    #         print(f"读取电解液二维码失败: {e}")
    #         return "N/A"

    # ===================== 环境监控区 ======================
    @property
    def data_glove_box_pressure(self) -> float:
        """手套箱压力 (bar, FLOAT32)"""
        if self.debug_mode:
            return -1
        status, read_err = self.client.use_node('REG_DATA_GLOVE_BOX_PRESSURE').read(2, word_order=WorderOrder.LITTLE)
        return status

    @property
    def data_glove_box_o2_content(self) -> float:
        """手套箱氧含量 (ppm, FLOAT32)"""
        if self.debug_mode:
            return -1
        value, read_err = self.client.use_node('REG_DATA_GLOVE_BOX_O2_CONTENT').read(2, word_order=WorderOrder.LITTLE)
        return value

    @property
    def data_glove_box_water_content(self) -> float:
        """手套箱水含量 (ppm, FLOAT32)"""
        if self.debug_mode:
            return -1
        value, read_err = self.client.use_node('REG_DATA_GLOVE_BOX_WATER_CONTENT').read(2, word_order=WorderOrder.LITTLE)
        return value


    def start_battery_completion_export(self, file_path=None):
        """开始电池完成数据的CSV导出"""
        if self.csv_export_running:
            return False, "CSV导出已在运行中"
        
        if file_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            file_path = f"battery_completion_data_{timestamp}.csv"
        
        self.csv_export_file = file_path
        self.csv_export_running = True
        
        # 创建CSV文件并写入表头
        try:
            with open(file_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'Timestamp', 'Battery_Count', 'Assembly_Time', 
                    'Open_Circuit_Voltage', 'Pole_Weight', 'Battery_Code', 
                    'Electrolyte_Code'
                ])
        except Exception as e:
            self.csv_export_running = False
            return False, f"创建CSV文件失败: {str(e)}"
        
        # 启动数据收集线程
        self.csv_export_thread = threading.Thread(target=self._csv_export_worker)
        self.csv_export_thread.daemon = True
        self.csv_export_thread.start()
        
        return True, "CSV导出已启动"
    
    def _csv_export_worker(self):
        """CSV导出工作线程"""
        last_battery_count = 0
        
        while self.csv_export_running:
            try:
                current_battery_count = self.data_assembly_coin_cell_num
                
                # 检查是否有新的电池完成（无论设备是否停止都继续监控）
                if current_battery_count > last_battery_count:
                    # 收集当前数据 - 一次性收集所有数据以确保一致性
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    
                    # 批量收集数据以减少时间差
                    try:
                        assembly_time = self.data_assembly_time()
                        pole_weight = self.data_pole_weight()
                        assembly_pressure = self.data_assembly_pressure()
                        electrolyte_volume = self.data_electrolyte_volume()
                        open_circuit_voltage = self.data_open_circuit_voltage()
                        battery_code = self.data_coin_cell_code()
                        electrolyte_code = self.data_electrolyte_code()

                        # 检查设备状态并记录到日志
                        device_stopped = self._sys_stop_status()
                        status_info = "(设备已停止)" if device_stopped else "(设备运行中)"
                        print(f"CSV导出: 检测到新电池完成 #{current_battery_count} {status_info}")
                        
                        # 写入CSV文件
                        with open(self.csv_export_file, 'a', newline='', encoding='utf-8') as csvfile:
                            writer = csv.writer(csvfile)
                            writer.writerow([
                                timestamp, current_battery_count, assembly_time,
                                pole_weight, assembly_pressure, electrolyte_volume,
                                open_circuit_voltage, 
                                battery_code, electrolyte_code
                            ])
                        
                        last_battery_count = current_battery_count
                        print(f"CSV导出: 数据已保存到文件 {self.csv_export_file}")
                        
                    except Exception as data_error:
                        print(f"数据收集错误: {str(data_error)}")
                        # 如果数据收集失败，不更新last_battery_count，下次重试
                
                time.sleep(1)  # 每秒检查一次
                
            except Exception as e:
                print(f"CSV导出错误: {str(e)}")
                time.sleep(5)  # 出错时等待5秒再重试
    
    def stop_csv_export(self):
        """停止CSV导出"""
        if not self.csv_export_running:
            return False, "CSV导出未在运行"
        
        self.csv_export_running = False
        
        if self.csv_export_thread and self.csv_export_thread.is_alive():
            self.csv_export_thread.join(timeout=5)
        
        return True, "CSV导出已停止"
    
    def get_csv_export_status(self):
        """获取CSV导出状态"""
        try:
            device_status = "停止" if self._sys_stop_status() else "运行"
        except:
            device_status = "未知"
            
        return {
            'running': self.csv_export_running,
            'file_path': self.csv_export_file if self.csv_export_running else None,
            'thread_alive': self.csv_export_thread.is_alive() if self.csv_export_thread else False,
            'device_status': device_status,
            'battery_count': self.data_assembly_coin_cell_num if hasattr(self, 'client') else 0
        }
    
    def force_continue_csv_export(self):
        """强制继续CSV导出，即使设备处于停止状态"""
        if not self.csv_export_running:
            return False, "CSV导出未在运行中"
        
        try:
            device_status = "停止" if self._sys_stop_status() else "运行"
            print(f"强制继续CSV导出 - 当前设备状态: {device_status}")
            
            # 即使设备停止，也继续监控数据变化
            return True, f"CSV导出将继续运行，设备状态: {device_status}"
        except Exception as e:
            return False, f"无法获取设备状态: {str(e)}"

    '''
    @property
    def data_stack_vision_code(self) -> int:
        """物料堆叠复检图片编码 (INT16)"""
        code, read_err =  self.client.use_node('REG_DATA_STACK_VISON_CODE').read(1)
        return code

    # ===================== 物料管理区 ======================
    @property
    def data_material_inventory(self) -> int:
        """主物料库存 (数量, INT16)"""
        inventory, read_err =  self.client.use_node('REG_DATA_MATERIAL_INVENTORY').read(1)
        return inventory

    @property
    def data_tips_inventory(self) -> int:
        """移液枪头库存 (数量, INT16)"""
        inventory, read_err = self.client.register_node_list(self.nodes).use_node('REG_DATA_TIPS_INVENTORY').read(1)
        return inventory
        
    '''

# # 对照参考一下0820文件的547-551行
#     # 数据读取与输出
#     def func_read_data_and_output(self, file_path: str="./coin_cell_data"):
#         # 检查CSV导出是否正在运行，已运行则跳出，防止同时启动两个while循环
#         if self.csv_export_running:
#             return False, "读取已在运行中"
        
#         #若不存在该目录则创建
#         if not os.path.exists(file_path):
#             os.makedirs(file_path)
#             print(f"创建目录: {file_path}")

#         # 只要允许读取标志位为true，就持续运行该函数，直到触发停止条件
#         while self.allow_data_read:

#             #函数运行标志位，确保只同时启动一个导出函数
#             self.csv_export_running = True

#             #等待接收结果标志位置True
#             while self.request_send_msg_status == False:
#                 print("waiting for send_msg_status to True")
#                 time.sleep(1)
#             #日期时间戳用于按天存放csv文件
#             time_date = datetime.now().strftime("%Y%m%d")
#             #秒级时间戳用于标记每一行电池数据
#             timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#             #生成输出文件的变量
#             self.csv_export_file = os.path.join(file_path, f"date_{time_date}.csv")   
            
#             #接收信息
#             data_open_circuit_voltage = self.data_open_circuit_voltage()
#             data_pole_weight = self.data_pole_weight()
#             data_assembly_time = self.data_assembly_time()
#             data_assembly_pressure = self.data_assembly_pressure()
#             data_electrolyte_volume = self.data_electrolyte_volume()
#             data_coin_num = self.data_coin_num()
#             data_electrolyte_code = self.data_electrolyte_code()
#             data_coin_cell_code = self.data_coin_cell_code()
#             # 电解液瓶位置
#             elec_bottle_site = 2
#             # 极片夹取位置(应当通过寄存器读光标)
#             Pos_elec_site = 0
#             Al_elec_site = 0
#             Gasket_site = 0

#             #接收完信息后，读取完毕标志位置True
#             self._unilab_rec_msg_succ_cmd()# = True
#             #等待允许读取标志位置False
#             while self.request_send_msg_status == True:
#                 print("waiting for send_msg_status to False")
#                 time.sleep(1)
#             self._unilab_rec_msg_succ_cmd()# = False

#             #此处操作物料信息（如果中途报错停止，如何）
#             #报错怎么办（加个判断标志位，如果发生错误，则根据停止位置扣除物料）
#             #根据物料光标判断取哪个物料（人工摆盘，电解液瓶，移液枪头都有光标位置，寄存器读即可）
            
#             #物料读取操作写在这里
#             #在这里进行物料调取
#             #转移物料瓶，elec_bottle_site对应第几瓶电解液（从依华寄存器读取）
#         #    transfer_bottle(deck, elec_bottle_site)
#         #    #找到电解液瓶的对象
#         #    electrolyte_rack = deck.get_resource("electrolyte_rack")
#         #    pending_positions = electrolyte_rack.get_pending_positions()[elec_bottle_site]
#         #    # TODO: 瓶子取液体操作需要加入
# #
# #
#         #    #找到压制工站对应的对象
#         #    battery_press_slot = deck.get_resource("battery_press_1")
#         #    #创建一个新电池
#         #    test_battery = Battery(
#         #        name=f"test_battery_{data_coin_num}",
#         #        diameter=20.0,  # 与压制槽直径匹配
#         #        height=3.0,     # 电池高度
#         #        max_volume=100.0,  # 100μL容量
#         #        barcode=data_coin_cell_code,  # 电池条码
#         #    )
#         #    if battery_press_slot.has_battery():
#         #        return False, "压制工站已有电池，无法放置新电池"
#         #    #在压制位放置电池
#         #    battery_press_slot.place_battery(test_battery)
#         #    #从第一个子弹夹中取料
#         #    clip_magazine_1_hole = self.deck.get_resource("clip_magazine_1").get_item(Pos_elec_site)
#         #    clip_magazine_2_hole = self.deck.get_resource("clip_magazine_2").get_item(Al_elec_site)
#         #    clip_magazine_3_hole = self.deck.get_resource("clip_magazine_3").get_item(Gasket_site)
#         #    
#         #    if clip_magazine_1_hole.get_sheet_count() > 0:   # 检查洞位是否有极片
#         #        electrode_sheet_1 = clip_magazine_1_hole.take_sheet()  # 从洞位取出极片
#         #        test_battery.add_electrode_sheet(electrode_sheet_1)  # 添加到电池中
#         #        print(f"已将极片 {electrode_sheet_1.name} 从子弹夹转移到电池")
#         #    else:
#         #        print("子弹夹洞位0没有极片")
# #
#         #    if clip_magazine_2_hole.get_sheet_count() > 0:   # 检查洞位是否有极片
#         #        electrode_sheet_2 = clip_magazine_2_hole.take_sheet()  # 从洞位取出极片
#         #        test_battery.add_electrode_sheet(electrode_sheet_2)  # 添加到电池中
#         #        print(f"已将极片 {electrode_sheet_2.name} 从子弹夹转移到电池")
#         #    else:
#         #        print("子弹夹洞位0没有极片")
# #
#         #    if clip_magazine_3_hole.get_sheet_count() > 0:   # 检查洞位是否有极片
#         #        electrode_sheet_3 = clip_magazine_3_hole.take_sheet()  # 从洞位取出极片
#         #        test_battery.add_electrode_sheet(electrode_sheet_3)  # 添加到电池中
#         #        print(f"已将极片 {electrode_sheet_3.name} 从子弹夹转移到电池")
#         #    else:
#         #        print("子弹夹洞位0没有极片")
#         #  
#         #    # TODO:#把电解液从瓶中取到电池夹子中
#         #    battery_site = deck.get_resource("battery_press_1")
#         #    clip_magazine_battery = deck.get_resource("clip_magazine_battery")
#         #    if battery_site.has_battery():
#         #        battery = battery_site.take_battery() #从压制槽取出电池
#         #        clip_magazine_battery.add_battery(battery) #从压制槽取出电池
# #
# #
# #
# #
#         #    # 保存配置到文件
#         #    self.deck.save("button_battery_station_layout.json", indent=2)
#         #    print("\n台面配置已保存到: button_battery_station_layout.json")
#         #
#         #    # 保存状态到文件
#         #    self.deck.save_state_to_file("button_battery_station_state.json", indent=2)
#         #    print("台面状态已保存到: button_battery_station_state.json")






#             #将数据写入csv中
#             #如当前目录下无同名文件则新建一个csv用于存放数据
#             if not os.path.exists(self.csv_export_file):
#                 #创建一个表头
#                 with open(self.csv_export_file, 'w', newline='', encoding='utf-8') as csvfile:
#                     writer = csv.writer(csvfile)
#                     writer.writerow([
#                         'Time', 'open_circuit_voltage', 'pole_weight', 
#                         'assembly_time', 'assembly_pressure', 'electrolyte_volume', 
#                         'coin_num', 'electrolyte_code', 'coin_cell_code'
#                     ])
#                     #立刻写入磁盘
#                     csvfile.flush()
#             #开始追加电池信息
#             with open(self.csv_export_file, 'a', newline='', encoding='utf-8') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow([
#                     timestamp, data_open_circuit_voltage, data_pole_weight,
#                     data_assembly_time, data_assembly_pressure, data_electrolyte_volume,
#                     data_coin_num, data_electrolyte_code, data_coin_cell_code
#                 ])
#                 #立刻写入磁盘
#                 csvfile.flush()

#             # 只要不在自动模式运行中，就将允许标志位置False
#             if self.sys_auto_status  == False or self.sys_start_status == False:
#                 self.allow_data_read = False
#                 self.csv_export_running = False
#             time.sleep(1)

#     def func_stop_read_data(self):
#         """停止CSV导出"""
#         if not self.csv_export_running:
#             return False, "read data未在运行"
        
#         self.csv_export_running = False
#         self.allow_data_read = False
        
#         if self.csv_export_thread and self.csv_export_thread.is_alive():
#             self.csv_export_thread.join(timeout=5)

#     def func_get_csv_export_status(self):
#         """获取CSV导出状态"""
#         return {
#             'allow_read': self.allow_data_read,
#             'running': self.csv_export_running,
#             'thread_alive': self.csv_export_thread.is_alive() if self.csv_export_thread else False
#         }

#     @property
#     def data_stack_vision_code(self) -> int:
#         """物料堆叠复检图片编码 (INT16)"""
#         if self.debug_mode:
#             return 0
#         code = 0
#         # code, read_err =  self.client.use_node('REG_DATA_STACK_VISON_CODE').read(1)
#         return code
#     '''
#     # ===================== 物料管理区 ======================
#     @property
#     def data_material_inventory(self) -> int:
#         """主物料库存 (数量, INT16)"""
#         inventory, read_err =  self.client.use_node('REG_DATA_MATERIAL_INVENTORY').read(1)
#         return inventory

#     @property
#     def data_tips_inventory(self) -> int:
#         """移液枪头库存 (数量, INT16)"""
#         inventory, read_err = self.client.register_node_list(self.nodes).use_node('REG_DATA_TIPS_INVENTORY').read(1)
#         return inventory
        
#     '''


if __name__ == "__main__":
    from pylabrobot.resources import Resource
    cell = CoinCellAssemblyWorkstation(Resource("1", 1, 1, 1), debug_mode = True)
    cell.start_battery_completion_export()


    print("success")