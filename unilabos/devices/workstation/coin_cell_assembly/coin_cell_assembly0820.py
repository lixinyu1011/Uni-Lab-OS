import json
import re
from pymodbus.client import ModbusTcpClient
from unilabos.device_comms.modbus_plc.node.modbus import WorderOrder, Coil, DiscreteInputs, HoldRegister, InputRegister, DataType
from pymodbus.constants import Endian
import time
import threading
import csv
import os
from datetime import datetime
from typing import Callable
from unilabos.device_comms.modbus_plc.client import TCPClient, ModbusNode, PLCWorkflow, ModbusWorkflow, WorkflowAction, BaseClient
from unilabos.device_comms.modbus_plc.node.modbus import DeviceType, Base as ModbusNodeBase, DataType, WorderOrder

class Coin_Cell_Assembly:
    """
    This provides a python class for the YIHUA COIN CELL ASSEMBLY SYSTEM. It provides functions to read and write to the system.
    """
    def __init__(self, address='192.168.1.20', port=502):
        """
        Initializer of the Coin_Cell_Assembly class.
        This function sets up the modbus tcp connection to the YIHUA COIN CELL ASSEMBLY SYSTEM
        """
        modbus_client = TCPClient(addr=address, port=port)
        modbus_client.client.connect()
        count = 100
        while count >0:
            count -=1
            if modbus_client.client.is_socket_open():
                break
            time.sleep(2)
        if not modbus_client.client.is_socket_open():
            raise ValueError('modbus tcp connection failed')
        self.nodes = BaseClient.load_csv('./coin_cell_assembly_a.csv')
        self.client  = modbus_client.register_node_list(self.nodes)
        self.success = False
        self.csv_export_thread = None
        self.csv_export_running = False
        self.csv_expoart_file = None

    # ====================== 命令类指令（COIL_x_） ======================

    def sys_start_cmd(self, cmd=None):
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

    def sys_stop_cmd(self, cmd=None):
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

    def sys_reset_cmd(self, cmd=None):
        """设备复位命令 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_SYS_RESET_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_RESET_CMD').read(1)
            return cmd_feedback[0]

    def sys_hand_cmd(self, cmd=None):
        """手动模式命令 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_SYS_HAND_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_HAND_CMD').read(1)
            return cmd_feedback[0]

    def sys_auto_cmd(self, cmd=None):
        """自动模式命令 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_SYS_AUTO_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_AUTO_CMD').read(1)
            return cmd_feedback[0]

    def sys_init_cmd(self, cmd=None):
        """初始化命令 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_SYS_INIT_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_SYS_INIT_CMD').read(1)
            return cmd_feedback[0]

    def unilab_send_msg_succ_cmd(self, cmd=None):
        """UNILAB发送配方完毕 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_UNILAB_SEND_MSG_SUCC_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_UNILAB_SEND_MSG_SUCC_CMD').read(1)
            return cmd_feedback[0]

    def unilab_rec_msg_succ_cmd(self, cmd=None):
        """UNILAB接收测试电池数据完毕 (可读写)"""
        if cmd is not None:
            self.success = False
            self.client.use_node('COIL_UNILAB_REC_MSG_SUCC_CMD').write(cmd)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('COIL_UNILAB_REC_MSG_SUCC_CMD').read(1)
            return cmd_feedback

    #def func_pack_device_start(self):
    #    """打包指令：设备启动"""
    #    self.success = False
    #    with open('action_device_start.json', 'r', encoding='utf-8') as f:
    #        action_json = json.load(f)
    #    self.client.execute_procedure_from_json(action_json)
    #    self.success = True
    #    return self.success

    def func_pack_device_init(self):
        #切换手动模式
        self.sys_hand_cmd(True)
        while (self.sys_hand_status) == False:
            print("waiting for hand_cmd")
            time.sleep(1)
        #设备初始化
        self.sys_init_cmd(True)
        #sys_init_status为bool值，不加括号
        while (self.sys_init_status)== False:
            print("waiting for init_cmd")
            time.sleep(1)
        #手动按钮置回False
        self.sys_hand_cmd(False)
        while (self.sys_hand_cmd()) == True:
            print("waiting for hand_cmd to False")
            time.sleep(1)
        #初始化命令置回False
        self.sys_init_cmd(False)
        while (self.sys_init_cmd()) == True:
            print("waiting for init_cmd to False")
            time.sleep(1)

    def func_pack_device_auto(self):
        #切换自动
        self.sys_auto_cmd(True)
        while (self.sys_auto_status) == False:
            print("waiting for auto_status")
            time.sleep(1)
        #自动按钮置False
        self.sys_auto_cmd(False)
        while (self.sys_auto_cmd()) == True:
            print("waiting for auto_cmd")
            time.sleep(1)

    def func_pack_device_start(self):
        #切换自动
        self.sys_start_cmd(True)
        while (self.sys_start_status) == False:
            print("waiting for start_status")
            time.sleep(1)
        #自动按钮置False
        self.sys_start_cmd(False)
        while (self.sys_start_cmd()) == True:
            print("waiting for start_cmd")
            time.sleep(1)        

    
    def func_pack_device_stop(self):
        #Auto模式
        while (self.sys_auto_status) == False: 
            print("wait for auto_status")
            time.sleep(1)
        self.sys_stop_cmd(True)
        while (self.sys_stop_status) == False:
            print("waiting for stop_status")
            time.sleep(1)
        #自动按钮置False
        self.sys_stop_cmd(False)
        while (self.sys_stop_cmd()) == True:
            print("waiting for stop_cmd")
            time.sleep(1)   

    #def func_pack_device_write_per_elec_param(self, params=None):
    #    """打包指令：设备单瓶电解液参数下发"""
    #    if params is not None:
    #        self.success = False
    #        params = json.load(params)
    #        with open('action_device_write_per_elec_param.json', 'r', encoding='utf-8') as f:
    #            action_json = json.load(f)
    #        self.client.execute_procedure_from_json(action_json, **params)
    #        self.success = True
    #        return self.success
    #    else:
    #        return False



    #def func_pack_device_write_per_elec_param(self, params=None):
    #    """打包指令：设备单瓶电解液参数下发"""
    #    if params is not None:
    #        self.success = False
    #        print(params)
    #        # 1. 处理 params 参数
    #        if isinstance(params, str):
    #            # 如果是 JSON 字符串则解析
    #            try:
    #                params_dict = json.loads(params)
    #            except json.JSONDecodeError:
    #                return False
    #        elif isinstance(params, dict):
    #            # 如果是字典直接使用
    #            params_dict = params
    #        else:
    #            return False
    #        # 2. 读取并处理 JSON 模板
    #        try:
    #            with open('action_device_write_per_elec_param.json', 'r', encoding='utf-8') as f:
    #                action_json = json.load(f)
    #        except FileNotFoundError:
    #            return False
    #        # 3. 替换模板变量
    #        def replace_template_vars(data, context):
    #            """递归替换模板变量"""
    #            if isinstance(data, dict):
    #                return {k: replace_template_vars(v, context) for k, v in data.items()}
    #            elif isinstance(data, list):
    #                return [replace_template_vars(item, context) for item in data]
    #            elif isinstance(data, str) and data.startswith("{{") and data.endswith("}}"):
    #                # 提取变量名：{{elec_use_num}} -> elec_use_num
    #                var_name = data[2:-2].strip()
    #                return context.get(var_name, data)
    #            return data
    #        processed_json = replace_template_vars(action_json, params_dict)
    #        print("The json is", processed_json)
    #        # 4. 执行处理后的 JSON
    #        self.client.execute_procedure_from_json(processed_json)
    #        self.success = True
    #        return self.success
    #    else:
    #        print("No params")
    #        return False



    #def func_pack_device_write_batch_elec_param(self, params=None):
    #    """打包指令：设备批量电解液参数下发"""
    #    if params is not None:
    #        self.success = False
    #        params = json.load(params)
    #        with open('action_device_write_batch_elec_param.json', 'r', encoding='utf-8') as f:
    #            action_json = json.load(f)
    #        self.client.execute_procedure_from_json(action_json, **params)
    #        self.success = True
    #        return self.success
    #    else:
    #        return False


    def func_pack_test(self, wf_name: str):
        """<UNK>"""
        if wf_name is not None:
            self.success = False
            # params = json.load(params)
            for i in range(5):
                print(wf_name)
            self.success = True
            return self.success
        else:
            print('No wf name')
            return False


  # ====================== 命令类指令（REG_x_） ======================
    def unilab_send_msg_electrolyte_num(self, num=None):
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

    def unilab_send_msg_electrolyte_use_num(self, use_num):
        """UNILAB写单次电解液使用瓶数(可读写)"""
        if use_num is not None:
            self.success = False
            self.client.use_node('REG_MSG_ELECTROLYTE_USE_NUM').write(use_num)
            self.success = True
            return self.success
        else:
            return False

    def unilab_send_msg_assembly_type(self, num=None):
        """UNILAB写组装参数"""
        if num is not None:
            self.success = False
            self.client.use_node('REG_MSG_ASSEMBLY_TYPE').write(num)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('REG_MSG_ASSEMBLY_TYPE').read(1)
            return cmd_feedback[0]

    def unilab_send_msg_electrolyte_vol(self, vol=None):
        """UNILAB写电解液吸取量参数"""
        if vol is not None:
            self.success = False
            self.client.use_node('REG_MSG_ELECTROLYTE_VOLUME').write(vol, data_type=DataType.FLOAT32, word_order=WorderOrder.LITTLE)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('REG_MSG_ELECTROLYTE_VOLUME').read(2, word_order=WorderOrder.LITTLE)
            return cmd_feedback[0]

    def unilab_send_msg_assembly_pressure(self, vol=None):
        """UNILAB写电池压制力"""
        if vol is not None:
            self.success = False
            self.client.use_node('REG_MSG_ASSEMBLY_PRESSURE').write(vol, data_type=DataType.FLOAT32, word_order=WorderOrder.LITTLE)
            self.success = True
            return self.success
        else:
            cmd_feedback, read_err = self.client.use_node('REG_MSG_ASSEMBLY_PRESSURE').read(2, word_order=WorderOrder.LITTLE)
            return cmd_feedback[0]

    def func_pack_send_msg_cmd(self, experiment_params=None):
        """UNILAB写参数"""
        while (self.request_rec_msg_status) == False: 
            print("wait for res_msg")
            time.sleep(1)
        if experiment_params is not None:
            self.success = False
            print(experiment_params)
            data = json.loads(experiment_params)
            print(data)
            self.unilab_send_msg_electrolyte_num(data['elec_num'])
            time.sleep(1)
            self.unilab_send_msg_electrolyte_use_num(data['elec_use_num'])
            time.sleep(1)
            self.unilab_send_msg_electrolyte_vol(data['elec_vol'])
            time.sleep(1)
            self.unilab_send_msg_assembly_type(data['assembly_type'])
            time.sleep(1)
            self.unilab_send_msg_assembly_pressure(data['assembly_pressure'])
            time.sleep(1)
            self.unilab_send_msg_succ_cmd(True)
            time.sleep(1)
            while (self.request_rec_msg_status) == True: 
                print("wait for res_msg")
                time.sleep(1)
            self.unilab_send_msg_succ_cmd(False)
            self.success = True
            return self.success
        else:
            return False

    # ==================== 状态类属性（COIL_x_STATUS） ====================
    @property
    def sys_start_status(self) -> bool:
        """设备启动中( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_START_STATUS').read(1)
        return status[0]

    @property
    def sys_stop_status(self) -> bool:
        """设备停止中( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_STOP_STATUS').read(1)
        return status[0]

    @property
    def sys_reset_status(self) -> bool:
        """设备复位中( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_RESET_STATUS').read(1)
        return status[0]

    @property
    def sys_hand_status(self) -> bool:
        """设备手动模式( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_HAND_STATUS').read(1)
        return status[0]

    @property
    def sys_auto_status(self) -> bool:
        """设备自动模式( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_AUTO_STATUS').read(1)
        return status[0]

    @property
    def sys_init_status(self) -> bool:
        """设备初始化完成( BOOL)"""
        status, read_err = self.client.use_node('COIL_SYS_INIT_STATUS').read(1)
        return status[0]

    @property
    def request_rec_msg_status(self) -> bool:
        """设备请求接受配方( BOOL)"""
        status, read_err = self.client.use_node('COIL_REQUEST_REC_MSG_STATUS').read(1)
        return status[0]

    @property
    def request_send_msg_status(self) -> bool:
        """设备请求发送测试数据( BOOL)"""
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
        num, read_err = self.client.use_node('REG_DATA_ASSEMBLY_COIN_CELL_NUM').read(1)
        return num

    @property
    def data_assembly_time(self) -> float:
        """单颗电池组装时间 (秒, REAL/FLOAT32)"""
        time, read_err =  self.client.use_node('REG_DATA_ASSEMBLY_PER_TIME').read(2, word_order=WorderOrder.LITTLE)
        return time

    @property
    def data_open_circuit_voltage(self) -> float:
        """开路电压值 (FLOAT32)"""
        vol, read_err =  self.client.use_node('REG_DATA_OPEN_CIRCUIT_VOLTAGE').read(2, word_order=WorderOrder.LITTLE)
        return vol

    @property
    def data_axis_x_pos(self) -> float:
        """分液X轴当前位置 (FLOAT32)"""
        pos, read_err =  self.client.use_node('REG_DATA_AXIS_X_POS').read(2, word_order=WorderOrder.LITTLE)
        return pos

    @property
    def data_axis_y_pos(self) -> float:
        """分液Y轴当前位置 (FLOAT32)"""
        pos, read_err =  self.client.use_node('REG_DATA_AXIS_Y_POS').read(2, word_order=WorderOrder.LITTLE)
        return pos

    @property
    def data_axis_z_pos(self) -> float:
        """分液Z轴当前位置 (FLOAT32)"""
        pos, read_err =  self.client.use_node('REG_DATA_AXIS_Z_POS').read(2, word_order=WorderOrder.LITTLE)
        return pos

    @property
    def data_pole_weight(self) -> float:
        """当前电池正极片称重数据 (FLOAT32)"""
        weight, read_err =  self.client.use_node('REG_DATA_POLE_WEIGHT').read(2, word_order=WorderOrder.LITTLE)
        return weight

    @property
    def data_assembly_pressure(self) -> int:
        """当前电池压制力 (INT16)"""
        pressure, read_err = self.client.use_node('REG_DATA_ASSEMBLY_PRESSURE').read(1)
        return pressure

    @property
    def data_electrolyte_volume(self) -> int:
        """当前电解液加注量 (INT16)"""
        vol, read_err = self.client.use_node('REG_DATA_ELECTROLYTE_VOLUME').read(1)
        return vol

    @property
    def data_coin_num(self) -> int:
        """当前电池数量 (INT16)"""
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

    # ===================== 环境监控区 ======================
    @property
    def data_glove_box_pressure(self) -> float:
        """手套箱压力 (bar, FLOAT32)"""
        status, read_err = self.client.use_node('REG_DATA_GLOVE_BOX_PRESSURE').read(2, word_order=WorderOrder.LITTLE)
        return status

    @property
    def data_glove_box_o2_content(self) -> float:
        """手套箱氧含量 (ppm, FLOAT32)"""
        value, read_err = self.client.use_node('REG_DATA_GLOVE_BOX_O2_CONTENT').read(2, word_order=WorderOrder.LITTLE)
        return value

    @property
    def data_glove_box_water_content(self) -> float:
        """手套箱水含量 (ppm, FLOAT32)"""
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
                        assembly_time = self.data_assembly_time
                        open_circuit_voltage = self.data_open_circuit_voltage
                        pole_weight = self.data_pole_weight
                        battery_code = self.data_coin_cell_code
                        electrolyte_code = self.data_electrolyte_code
                        
                        # 检查设备状态并记录到日志
                        device_stopped = self.sys_stop_status
                        status_info = "(设备已停止)" if device_stopped else "(设备运行中)"
                        print(f"CSV导出: 检测到新电池完成 #{current_battery_count} {status_info}")
                        
                        # 写入CSV文件
                        with open(self.csv_export_file, 'a', newline='', encoding='utf-8') as csvfile:
                            writer = csv.writer(csvfile)
                            writer.writerow([
                                timestamp, current_battery_count, assembly_time,
                                open_circuit_voltage, pole_weight, battery_code,
                                electrolyte_code
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
            device_status = "停止" if self.sys_stop_status else "运行"
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
            device_status = "停止" if self.sys_stop_status else "运行"
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

if __name__ == '__main__':
    coin_cell_assmbly = Coin_Cell_Assembly(address="192.168.1.20", port="502")

    #params = {
    #    "elec_num": 32
    #}
    #str_data = json.dumps(params, ensure_ascii=False)
    #print('param:', coin_cell_assmbly.func_pack_device_write_batch_elec_param(params))
    #time.sleep(1)

    print(coin_cell_assmbly.data_electrolyte_code)
    time.sleep(1)
    print(coin_cell_assmbly.data_coin_cell_code)
    time.sleep(1)


'''
        print('start:', coin_cell_assmbly.func_pack_device_start())
    time.sleep(1)





    

    
    print('start:', coin_cell_assmbly.func_pack_device_start())
    time.sleep(1)
    
    print('stop:', coin_cell_assmbly.func_pack_device_stop())
    time.sleep(1)
    
    while True:
        # cmd coil
        print('start cmd:', coin_cell_assmbly.sys_start_cmd(True))
        time.sleep(1)
        print('stop cmd:', coin_cell_assmbly.sys_stop_cmd(False))
        time.sleep(1)
        print('reset cmd:', coin_cell_assmbly.sys_reset_cmd(True))
        time.sleep(1)
        print('hand cmd:', coin_cell_assmbly.sys_hand_cmd(False))
        time.sleep(1)
        print('auto cmd:', coin_cell_assmbly.sys_auto_cmd(True))
        time.sleep(1)
        print('init cmd:', coin_cell_assmbly.sys_init_cmd(False))
        time.sleep(1)
        print('send msg succ cmd:', coin_cell_assmbly.unilab_send_msg_succ_cmd(False))
        time.sleep(1)
        print('rec msg succ cmd:', coin_cell_assmbly.unilab_rec_msg_succ_cmd(True))
        time.sleep(1)

        # cmd reg
        print('elec use num msg:', coin_cell_assmbly.unilab_send_msg_electrolyte_use_num(8))
        time.sleep(1)
        print('elec num msg:', coin_cell_assmbly.unilab_send_msg_electrolyte_num(4))
        time.sleep(1)
        print('elec vol msg:', coin_cell_assmbly.unilab_send_msg_electrolyte_vol(3.3))
        time.sleep(1)
        print('assembly type msg:', coin_cell_assmbly.unilab_send_msg_assembly_type(1))
        time.sleep(1)   
        print('assembly pressure msg:', coin_cell_assmbly.unilab_send_msg_assembly_pressure(1))
        time.sleep(1)   

        # status coil
        print('start status:',coin_cell_assmbly.sys_start_status)
        time.sleep(1)
        print('stop status:',coin_cell_assmbly.sys_stop_status)
        time.sleep(1)
        print('reset status:',coin_cell_assmbly.sys_reset_status)
        time.sleep(1)
        print('hand status:',coin_cell_assmbly.sys_hand_status)
        time.sleep(1)
        print('auto status:', coin_cell_assmbly.sys_auto_status)
        time.sleep(1)
        print('init ok:', coin_cell_assmbly.sys_init_status)
        time.sleep(1)
        print('request rec msg:', coin_cell_assmbly.request_rec_msg_status)
        time.sleep(1)
        print('request send msg:', coin_cell_assmbly.request_send_msg_status)
        time.sleep(1)

        # status reg
        print('assembly coin cell num:', coin_cell_assmbly.data_assembly_coin_cell_num)
        time.sleep(1)
        print('assembly coin assembly per time:', coin_cell_assmbly.data_assembly_time)
        time.sleep(1)
        print('open circuit vol:', coin_cell_assmbly.data_open_circuit_voltage)
        time.sleep(1)
        print('axis x pos:', coin_cell_assmbly.data_axis_x_pos)
        time.sleep(1)
        print('axis y pos:', coin_cell_assmbly.data_axis_y_pos)
        time.sleep(1)
        print('axis z pos:', coin_cell_assmbly.data_axis_z_pos)
        time.sleep(1)
        print('pole weight:', coin_cell_assmbly.data_pole_weight)
        time.sleep(1)
        print('assembly pressure:', coin_cell_assmbly.data_assembly_coin_cell_num)
        time.sleep(1)  
        print('assembly electrolyte vol:', coin_cell_assmbly.data_electrolyte_volume)
        time.sleep(1)  
        print('assembly coin num:', coin_cell_assmbly.data_coin_num)
        time.sleep(1)   
        print('coin cell code:', coin_cell_assmbly.data_coin_cell_code)
        time.sleep(1)
        print('elec code:', coin_cell_assmbly.data_electrolyte_code)
        time.sleep(1)
        print('glove box pressure:', coin_cell_assmbly.data_glove_box_pressure)
        time.sleep(1)
        print('glove box o2:', coin_cell_assmbly.data_glove_box_o2_content)
        time.sleep(1)
        print('glove box water:', coin_cell_assmbly.data_glove_box_water_content)
        time.sleep(1)

'''