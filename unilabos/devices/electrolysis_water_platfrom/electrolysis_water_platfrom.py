# -*- coding: utf-8 -*-
import serial
import time
import csv
import threading
import os
from collections import deque
from typing import Dict, Any, Optional

from unilabos.devices.workstation.workstation_base import WorkstationBase
from unilabos.utils.log import logger

# 串口配置常量
DEFAULT_PORT = "COM5"
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 0.2


class ElectrolysisWaterPlatform(WorkstationBase):
    """
    电解水平台工作站
    基于 WorkstationBase 的电解水实验平台，支持串口通信和数据采集
    """
    
    def __init__(
        self,
        config: dict = None,
        port: str = None,
        baudrate: int = None,
        csv_path: Optional[str] = None,
        timeout: float = None,
        # 新增参数：默认控制参数
        default_mode: int = 0,              # 默认模式：0=恒压，1=恒流
        default_voltage_mv: int = 2000,      # 默认电压：2000mV (2V)
        default_current_ma: int = 1000,      # 默认电流：1000mA (1A)
        default_temperature_c: float = 25.0, # 默认温度：25°C
        default_ki: float = 0.0,            # 默认Ki参数
        default_pump_percent: float = 50.0, # 默认泵速：50%
        sampling_interval: float = 1.0,     # 采样间隔（秒）
        auto_start_control: bool = True,    # 是否自动启动控制
        deck=None,                          # WorkstationBase 需要的 deck 参数
        *args,
        **kwargs):

        # 初始化父类 WorkstationBase（必须传递 deck 参数）
        super().__init__(deck=deck, *args, **kwargs)
        
        if config is None:
            config = {}
        
        # ========== 配置 ==========
        self.port = config.get("port", port) or DEFAULT_PORT
        self.baudrate = config.get("baudrate", baudrate) or DEFAULT_BAUDRATE
        # 如果没有指定路径，默认保存在代码文件所在目录
        if csv_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.csv_path = os.path.join(current_dir, "stm32_data.csv")
        else:
            self.csv_path = csv_path
        self.ser_timeout = config.get("timeout", timeout) or DEFAULT_TIMEOUT
        self.chunk_read = 128
        
        # ========== 默认控制参数 ==========
        self.default_mode = config.get("default_mode", default_mode)
        self.default_voltage_mv = config.get("default_voltage_mv", default_voltage_mv)
        self.default_current_ma = config.get("default_current_ma", default_current_ma)
        self.default_temperature_c = config.get("default_temperature_c", default_temperature_c)
        self.default_ki = config.get("default_ki", default_ki)
        self.default_pump_percent = config.get("default_pump_percent", default_pump_percent)
        self.sampling_interval = config.get("sampling_interval", sampling_interval)
        self.auto_start_control = config.get("auto_start_control", auto_start_control)
        
        # ========== 控制器 ==========
        self.controller = None  # 外部控制器实例（可选）
        self.use_controller = False  # 是否使用控制器
        self.control_loop_thread: Optional[threading.Thread] = None
        self.control_loop_running = False
        
        # 串口对象
        self.ser: Optional[serial.Serial] = None
        self.stop_flag = False
        
        # 线程对象
        self.rx_thread: Optional[threading.Thread] = None
        self.tx_thread: Optional[threading.Thread] = None
        
        # ========== 最新数据存储（用于状态查询）==========
        self._latest_data = {
            "timestamp": "",
            "current": "0",
            "voltage": "0",
            "temperature": "0",
            "tds": "0",
            "gas_flow": "0",
            "liquid_flow": "0",
            "ph": "0"
        }
        self._data_lock = threading.Lock()
        
        # ==== 接收（下位机->上位机）：固定 1+13+1 = 15 字节 ====
        self.RX_HEAD = 0x3E
        self.RX_TAIL = 0x3E
        self.RX_FRAME_LEN = 1 + 13 + 1  # 15
        
        # ==== 发送（上位机->下位机）：固定 1+9+1 = 11 字节 ====
        self.TX_HEAD = 0x3E
        self.TX_TAIL = 0xE3  # 协议图中标注 E3 作为帧尾
        self.TX_FRAME_LEN = 1 + 9 + 1  # 11
        
        # ========== 自动启动串口连接 ==========
        # 在后台线程中启动串口连接和数据接收
        self._init_thread = threading.Thread(target=self._auto_start, daemon=True, name="electrolysis_init")
        self._init_thread.start()
    
    def _auto_start(self):
        """自动启动串口连接（在后台线程中）"""
        import time
        # 稍微延迟，等待系统初始化完成
        time.sleep(0.5)
        
        try:
            print(f"[INFO] 正在连接电解水平台串口: {self.port} @ {self.baudrate}...")
            self.ser = self.open_serial()
            if self.ser:
                # 启动接收线程
                self.rx_thread = threading.Thread(target=self.rx_thread_fn, daemon=True, name="electrolysis_rx")
                self.rx_thread.start()
                print(f"[✓] 电解水平台串口连接成功: {self.port}")
                print(f"[✓] 数据接收线程已启动，CSV 文件: {self.csv_path}")
                
                # 如果启用了自动控制，发送初始控制指令
                if self.auto_start_control:
                    time.sleep(0.5)  # 等待串口稳定
                    mode_str = "恒压" if self.default_mode == 0 else "恒流"
                    print(f"[INFO] 发送初始控制参数: {mode_str} | "
                          f"电压:{self.default_voltage_mv}mV | "
                          f"电流:{self.default_current_ma}mA | "
                          f"泵速:{self.default_pump_percent}%")
                    self.send_command(
                        mode=self.default_mode,
                        current_ma=self.default_current_ma,
                        voltage_mv=self.default_voltage_mv,
                        temp_c=self.default_temperature_c,
                        ki=self.default_ki,
                        pump_percent=self.default_pump_percent
                    )
                    print(f"[✓] 初始控制指令已发送，采样间隔: {self.sampling_interval}秒")
            else:
                print(f"[✗] 电解水平台串口连接失败: {self.port}")
                print(f"[提示] 请检查: 1) 串口号是否正确 2) 设备是否已连接 3) 串口是否被其他程序占用")
        except Exception as e:
            print(f"[✗] 电解水平台自动启动失败: {e}")
            import traceback
            traceback.print_exc()
    
    def open_serial(self, port: Optional[str] = None, baudrate: Optional[int] = None, timeout: Optional[float] = None) -> Optional[serial.Serial]:
        """打开串口"""
        port = self.port
        baudrate = self.baudrate
        timeout = self.ser_timeout
        try:
            ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"[OK] 串口 {port} 已打开，波特率 {baudrate}")
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.ser = ser
            return ser
        except serial.SerialException as e:
            print(f"[ERR] 无法打开串口失败 {port}: {e}")
            return None

    def close_serial(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] 串口已关闭")

    @staticmethod
    def u16_be(h: int, l: int) -> int:
        """将两个字节组合成16位无符号整数（大端序）"""
        return ((h & 0xFF) << 8) | (l & 0xFF)

    @staticmethod
    def split_u16_be(val: int) -> tuple:
        """返回 (高字节, 低字节)，输入会夹到 0..65535"""
        v = int(max(0, min(65535, int(val))))
        return (v >> 8) & 0xFF, v & 0xFF

    # ================== 接收：固定15字节 ==================
    def parse_rx_payload(self, dat13: bytes) -> Optional[Dict[str, Any]]:
        """解析 13 字节数据区（下位机发送到上位机）"""
        if len(dat13) != 13:
            return None
        current_mA      = self.u16_be(dat13[0], dat13[1])
        voltage_mV      = self.u16_be(dat13[2], dat13[3])
        temperature_raw = self.u16_be(dat13[4], dat13[5])
        tds_ppm         = self.u16_be(dat13[6], dat13[7])
        gas_sccm        = self.u16_be(dat13[8], dat13[9])
        liquid_mL       = self.u16_be(dat13[10], dat13[11])
        ph_raw          = dat13[12] & 0xFF

        return {
            "Current_mA": current_mA,
            "Voltage_mV": voltage_mV,
            
            "Temperature_C": round(temperature_raw / 100.0, 2),
            "TDS_ppm": tds_ppm,
            "GasFlow_sccm": gas_sccm,
            "LiquidFlow_mL": liquid_mL,
            "pH": round(ph_raw / 10.0, 2)
        }

    def try_parse_rx_frame(self, frame15: bytes) -> Optional[Dict[str, Any]]:
        """尝试解析接收帧"""
        if len(frame15) != self.RX_FRAME_LEN:
            return None
        if frame15[0] != self.RX_HEAD or frame15[-1] != self.RX_TAIL:
            return None
        return self.parse_rx_payload(frame15[1:-1])

    def rx_thread_fn(self):
        """接收线程函数"""
        headers = ["Timestamp", "Current_mA", "Voltage_mV",
                   "Temperature_C", "TDS_ppm", "GasFlow_sccm", "LiquidFlow_mL", "pH"]

        new_file = not os.path.exists(self.csv_path)
        f = open(self.csv_path, mode='a', newline='', encoding='utf-8')
        writer = csv.writer(f)
        if new_file:
            writer.writerow(headers)
            f.flush()

        buf = deque(maxlen=8192)
        print(f"[RX] 开始接收（帧长 {self.RX_FRAME_LEN} 字节）；写入：{self.csv_path}")

        try:
            while not self.stop_flag and self.ser and self.ser.is_open:
                chunk = self.ser.read(self.chunk_read)
                if chunk:
                    buf.extend(chunk)
                    while True:
                        # 找帧头
                        try:
                            start = next(i for i, b in enumerate(buf) if b == self.RX_HEAD)
                        except StopIteration:
                            buf.clear()
                            break
                        if start > 0:
                            for _ in range(start):
                                buf.popleft()
                        if len(buf) < self.RX_FRAME_LEN:
                            break
                        candidate = bytes([buf[i] for i in range(self.RX_FRAME_LEN)])
                        if candidate[-1] == self.RX_TAIL:
                            parsed = self.try_parse_rx_frame(candidate)
                            for _ in range(self.RX_FRAME_LEN):
                                buf.popleft()
                            if parsed:
                                ts = time.strftime("%Y-%m-%d %H:%M:%S")
                                row = [ts,
                                       parsed["Current_mA"], parsed["Voltage_mV"],
                                       parsed["Temperature_C"], parsed["TDS_ppm"],
                                       parsed["GasFlow_sccm"], parsed["LiquidFlow_mL"],
                                       parsed["pH"]]
                                writer.writerow(row)
                                f.flush()
                                
                                # 更新最新数据（供状态查询使用）
                                with self._data_lock:
                                    self._latest_data["timestamp"] = ts
                                    self._latest_data["current"] = str(parsed["Current_mA"])
                                    self._latest_data["voltage"] = str(parsed["Voltage_mV"])
                                    self._latest_data["temperature"] = str(parsed["Temperature_C"])
                                    self._latest_data["tds"] = str(parsed["TDS_ppm"])
                                    self._latest_data["gas_flow"] = str(parsed["GasFlow_sccm"])
                                    self._latest_data["liquid_flow"] = str(parsed["LiquidFlow_mL"])
                                    self._latest_data["ph"] = str(parsed["pH"])
                                # 若不想打印可注释下一行
                                # print(f"[{ts}] I={parsed['Current_mA']} mA, V={parsed['Voltage_mV']} mV, "
                                #       f"T={parsed['Temperature_C']} °C, TDS={parsed['TDS_ppm']}, "
                                #       f"Gas={parsed['GasFlow_sccm']} sccm, Liq={parsed['LiquidFlow_mL']} mL, pH={parsed['pH']}")
                        else:
                            # 头不变，尾不对，丢1字节继续对齐
                            buf.popleft()
                else:
                    time.sleep(0.01)
        finally:
            f.close()
            print("[RX] 接收线程退出，CSV 已关闭")

    # ================== 发送：固定11字节 ==================
    def build_tx_frame(self, mode: int, current_ma: int, voltage_mv: int, temp_c: float, ki: float, pump_percent: float) -> bytes:
        """
        发送帧：HEAD + [mode, I_hi, I_lo, V_hi, V_lo, T_hi, T_lo, Ki_byte, Pump_byte] + TAIL
        - mode: 0=恒压, 1=恒流
        - current_ma: mA (0..65535)
        - voltage_mv: mV (0..65535)
        - temp_c: ℃，将 *100 后拆分为高/低字节
        - ki: 0.0..20.0  -> byte = round(ki * 10) 夹到 0..200
        - pump_percent: 0..100 -> byte = round(pump * 2) 夹到 0..200
        """
        mode_b = 1 if int(mode) == 1 else 0

        i_hi, i_lo = self.split_u16_be(current_ma)
        v_hi, v_lo = self.split_u16_be(voltage_mv)

        t100 = int(round(float(temp_c) * 100.0))
        t_hi, t_lo = self.split_u16_be(t100)

        ki_b = int(max(0, min(200, round(float(ki) * 10))))
        pump_b = int(max(0, min(200, round(float(pump_percent) * 2))))

        return bytes((
            self.TX_HEAD,
            mode_b,
            i_hi, i_lo,
            v_hi, v_lo,
            t_hi, t_lo,
            ki_b,
            pump_b,
            self.TX_TAIL
        ))

    def tx_thread_fn(self):
        """
        发送线程函数
        用户输入 6 个用逗号分隔的数值：
        mode,current_mA,voltage_mV,set_temp_C,Ki,pump_percent
        例如： 0,1000,500,0,0,50
        """
        print("\n输入 6 个值（用英文逗号分隔），顺序为：")
        print("mode,current_mA,voltage_mV,set_temp_C,Ki,pump_percent")
        print("示例恒压：0,500,1000,25,0,100   （stop 结束）\n")
        print("示例恒流：1,1000,500,25,0,100   （stop 结束）\n")
        print("示例恒流：1,2000,500,25,0,100   （stop 结束）\n")
        # 1,2000,500,25,0,100

        while not self.stop_flag and self.ser and self.ser.is_open:
            try:
                line = input(">>> ").strip()
            except EOFError:
                self.stop_flag = True
                break

            if not line:
                continue
            if line.lower() == "stop":
                self.stop_flag = True
                print("[SYS] 停止程序")
                break

            try:
                parts = [p.strip() for p in line.split(",")]
                if len(parts) != 6:
                    raise ValueError("需要 6 个逗号分隔的数值")
                mode = int(parts[0])
                i_ma = int(float(parts[1]))
                v_mv = int(float(parts[2]))
                t_c  = float(parts[3])
                ki   = float(parts[4])
                pump = float(parts[5])

                frame = self.build_tx_frame(mode, i_ma, v_mv, t_c, ki, pump)
                self.ser.write(frame)
                print("[TX]", " ".join(f"{b:02X}" for b in frame))
            except Exception as e:
                print("[TX] 输入/打包失败：", e)
                print("格式：mode,current_mA,voltage_mV,set_temp_C,Ki,pump_percent")
                continue
    
    def start(self):
        """启动电解水平台（用于命令行模式）"""
        # 如果串口未打开，先打开
        if not self.ser or not self.ser.is_open:
            self.ser = self.open_serial()
        
        if self.ser:
            try:
                # 如果接收线程未启动，启动它
                if not self.rx_thread or not self.rx_thread.is_alive():
                    self.rx_thread = threading.Thread(target=self.rx_thread_fn, daemon=True, name="electrolysis_rx")
                    self.rx_thread.start()
                
                # 启动发送线程（用于交互式命令输入）
                self.tx_thread = threading.Thread(target=self.tx_thread_fn, daemon=True, name="electrolysis_tx")
                self.tx_thread.start()
                print("[INFO] 电解水平台已启动（交互模式）")
                self.tx_thread.join()  # 等待用户输入线程结束（输入 stop）
            finally:
                self.close_serial()
    
    def stop(self):
        """停止电解水平台"""
        print("[INFO] 正在停止电解水平台...")
        self.stop_flag = True
        
        # 等待线程结束
        if hasattr(self, 'rx_thread') and self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=2.0)
        if hasattr(self, 'tx_thread') and self.tx_thread and self.tx_thread.is_alive():
            self.tx_thread.join(timeout=2.0)
        
        # 关闭串口
        self.close_serial()
        print("[INFO] 电解水平台已停止")
    
    def post_init(self, ros_node):
        """ROS2 系统初始化完成后的回调"""
        self._ros_node = ros_node
        print(f"[INFO] 电解水平台 ROS2 节点已就绪: {ros_node.device_id}")
        print(f"[INFO] 串口: {self.port}, 波特率: {self.baudrate}")
    
    # ================== 状态属性（供ROS2发布使用）==================
    @property
    def connection_status(self) -> str:
        """连接状态"""
        if self.ser and self.ser.is_open:
            return "Connected"
        return "Disconnected"
    
    @property
    def timestamp(self) -> str:
        """最新数据时间戳"""
        with self._data_lock:
            return self._latest_data["timestamp"]
    
    @property
    def current(self) -> str:
        """电流 (mA)"""
        with self._data_lock:
            return self._latest_data["current"]
    
    @property
    def voltage(self) -> str:
        """电压 (mV)"""
        with self._data_lock:
            return self._latest_data["voltage"]
    
    @property
    def temperature(self) -> str:
        """温度 (°C)"""
        with self._data_lock:
            return self._latest_data["temperature"]
    
    @property
    def tds(self) -> str:
        """TDS (ppm)"""
        with self._data_lock:
            return self._latest_data["tds"]
    
    @property
    def gas_flow(self) -> str:
        """气体流量 (sccm)"""
        with self._data_lock:
            return self._latest_data["gas_flow"]
    
    @property
    def liquid_flow(self) -> str:
        """液体流量 (mL)"""
        with self._data_lock:
            return self._latest_data["liquid_flow"]
    
    @property
    def ph(self) -> str:
        """pH值"""
        with self._data_lock:
            return self._latest_data["ph"]
    
    # ================== 数据采集与控制接口 ==================
    def get_sensor_data(self) -> list:
        """
        获取最新的传感器数据（从parse_rx_payload解析的数据中返回）
        
        Returns:
            list: [timestamp, current_mA, voltage_mV, temperature_C, tds_ppm, gas_flow_sccm, liquid_flow_mL, ph]
        """
        with self._data_lock:
            return [
                self._latest_data["timestamp"],
                float(self._latest_data["current"]) if self._latest_data["current"] != "0" else 0.0,
                float(self._latest_data["voltage"]) if self._latest_data["voltage"] != "0" else 0.0,
                float(self._latest_data["temperature"]) if self._latest_data["temperature"] != "0" else 0.0,
                float(self._latest_data["tds"]) if self._latest_data["tds"] != "0" else 0.0,
                float(self._latest_data["gas_flow"]) if self._latest_data["gas_flow"] != "0" else 0.0,
                float(self._latest_data["liquid_flow"]) if self._latest_data["liquid_flow"] != "0" else 0.0,
                float(self._latest_data["ph"]) if self._latest_data["ph"] != "0" else 0.0
            ]
    
    def send_command(
        self, 
        mode: int, 
        current_ma: int, 
        voltage_mv: int, 
        temp_c: float = 25.0, 
        ki: float = 0.0, 
        pump_percent: float = 0.0
    ) -> bool:
        """
        发送控制指令到电解水平台
        
        Args:
            mode: 控制模式 (0=恒压, 1=恒流)
            current_ma: 目标电流 (mA)
            voltage_mv: 目标电压 (mV)
            temp_c: 目标温度 (°C)
            ki: Ki 参数 (0.0-20.0)
            pump_percent: 泵速百分比 (0-100)
        
        Returns:
            bool: 发送是否成功
        """
        if not self.ser or not self.ser.is_open:
            print("[ERR] 串口未连接，无法发送指令")
            return False
        
        try:
            frame = self.build_tx_frame(mode, current_ma, voltage_mv, temp_c, ki, pump_percent)
            self.ser.write(frame)
            mode_str = "恒压" if mode == 0 else "恒流"
            print(f"[CMD] 发送成功 | {mode_str} | I:{current_ma}mA | V:{voltage_mv}mV | T:{temp_c}°C | Ki:{ki} | 泵:{pump_percent}%")
            return True
        except Exception as e:
            print(f"[ERR] 发送失败: {e}")
            return False
    
    # ================== 控制器接口 ==================
    def set_controller(self, controller):
        """
        设置外部控制器
        
        Args:
            controller: ElectrolysisController 实例
        """
        self.controller = controller
        self.use_controller = True
        print(f"[Platform] 控制器已设置")
    
    def start_control_loop(self):
        """
        启动控制循环
        
        使用控制器计算控制向量并执行
        """
        if not self.use_controller or self.controller is None:
            print("[Platform] 错误: 未设置控制器，无法启动控制循环")
            return
        
        if self.control_loop_running:
            print("[Platform] 控制循环已在运行")
            return
        
        self.control_loop_running = True
        self.control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="control_loop"
        )
        self.control_loop_thread.start()
        print(f"[Platform] 控制循环已启动，采样间隔: {self.sampling_interval}秒")
    
    def stop_control_loop(self):
        """停止控制循环"""
        if not self.control_loop_running:
            return
        
        self.control_loop_running = False
        if self.control_loop_thread and self.control_loop_thread.is_alive():
            self.control_loop_thread.join(timeout=2.0)
        print("[Platform] 控制循环已停止")
    
    def _control_loop(self):
        """
        控制循环主函数
        
        流程：
        1. 获取传感器数据（平台解析）
        2. 传给控制器计算
        3. 获取控制向量
        4. 解析并执行控制向量
        """
        while self.control_loop_running and self.ser and self.ser.is_open:
            try:
                # 1. 获取传感器数据
                sensor_data = self.get_sensor_data()
                
                # 解包：[timestamp, current, voltage, temp, tds, gas, liquid, ph]
                timestamp, current, voltage, temp, tds, gas_flow, liquid_flow, ph = sensor_data
                
                # 2. 准备输入数据给控制器：[current, voltage, temp, ph, gas, liquid]
                input_data = [current, voltage, temp, ph, gas_flow, liquid_flow]
                
                # 3. 调用控制器计算控制向量
                control_vector = self.controller.compute_control(input_data)
                
                # 4. 解析并执行控制向量
                self._execute_control_vector(control_vector)
                
            except Exception as e:
                print(f"[Platform] 控制循环错误: {e}")
                import traceback
                traceback.print_exc()
            
            # 等待下一个采样周期
            time.sleep(self.sampling_interval)
    
    def _execute_control_vector(self, control_vector: list):
        """
        解析控制向量并执行
        
        Args:
            control_vector: [mode, current_ma, voltage_mv, temperature_c, pump_percent]
        """
        if len(control_vector) < 5:
            print(f"[Platform] 警告: 控制向量维度不足，期望5，实际{len(control_vector)}")
            return
        
        # 解析控制向量
        mode = int(control_vector[0])
        current_ma = int(control_vector[1])
        voltage_mv = int(control_vector[2])
        temperature_c = float(control_vector[3])
        pump_percent = float(control_vector[4])
        
        # 安全限幅
        mode = max(0, min(1, mode))
        current_ma = max(0, min(2000, current_ma))
        voltage_mv = max(0, min(5000, voltage_mv))
        temperature_c = max(0, min(50, temperature_c))
        pump_percent = max(0, min(100, pump_percent))
        
        # 发送控制指令
        self.send_command(
            mode=mode,
            current_ma=current_ma,
            voltage_mv=voltage_mv,
            temp_c=temperature_c,
            ki=self.default_ki,
            pump_percent=pump_percent
        )


# ================== 使用示例 ==================
def example_with_controller():
    """
    示例1：使用控制器（平台+深度学习控制器）
    
    数据流：
        平台解析传感器数据 → 控制器计算 → 返回控制向量 → 平台执行
    """
    from electrolysis_controller import ElectrolysisController
    
    print("="*70)
    print("示例1：平台 + 控制器（深度学习）")
    print("="*70)
    
    # 1. 创建平台（不自动发送控制指令，由控制器接管）
    platform = ElectrolysisWaterPlatform(
        port="COM5",
        baudrate=115200,
        sampling_interval=1.0,
        auto_start_control=False  # 不自动控制，由控制器接管
    )
    
    # 等待平台连接
    time.sleep(3)
    
    # 2. 创建控制器（可选：加载模型）
    controller = ElectrolysisController(
        model_path=None,  # 不使用模型，使用简单规则
        model_type="dummy",
        input_dim=6,
        output_dim=5
    )
    
    # 可选：设置归一化参数
    # controller.set_normalization_params(
    #     input_mean=[1000, 2500, 25, 7, 50, 30],
    #     input_std=[500, 1000, 5, 2, 25, 15],
    #     output_mean=[0, 1000, 2500, 25, 50],
    #     output_std=[1, 500, 1000, 5, 25]
    # )
    
    # 3. 将控制器设置到平台
    platform.set_controller(controller)
    
    # 4. 启动控制循环
    platform.start_control_loop()
    
    print("\n控制循环运行中...")
    print(f"{'时间':<20} {'电流(mA)':<12} {'电压(mV)':<12} {'温度(°C)':<10}")
    print("-"*70)
    
    try:
        # 监控20秒
        for i in range(20):
            data = platform.get_sensor_data()
            timestamp, current, voltage, temp = data[0], data[1], data[2], data[3]
            if timestamp:
                print(f"{timestamp:<20} {current:<12.1f} {voltage:<12.1f} {temp:<10.2f}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        platform.stop_control_loop()
        platform.stop()
        print("\n平台已停止")


def example_simple_start():
    """
    示例2：简单启动（使用默认参数，不使用控制器）
    """
    print("="*70)
    print("示例2：简单启动 - 使用默认参数")
    print("="*70)
    
    # 使用默认参数启动：恒压2V，限流1A，泵速50%
    platform = ElectrolysisWaterPlatform(
        port="COM5",
        baudrate=115200,
        auto_start_control=True  # 自动发送初始控制指令
    )
    
    # 平台会自动：
    # 1. 连接串口
    # 2. 启动数据接收
    # 3. 发送初始控制指令（恒压2000mV，限流1000mA）
    
    # 等待连接完成
    time.sleep(3)
    
    try:
        # 持续获取数据
        print("\n开始监控数据...")
        print(f"{'时间':<20} {'电流(mA)':<12} {'电压(mV)':<12} {'温度(°C)':<10}")
        print("-"*70)
        
        for i in range(20):
            data = platform.get_sensor_data()
            timestamp, current, voltage, temp = data[0], data[1], data[2], data[3]
            if timestamp:
                print(f"{timestamp:<20} {current:<12.1f} {voltage:<12.1f} {temp:<10.2f}")
            time.sleep(platform.sampling_interval)
            
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        platform.stop()


def example_custom_start():
    """
    示例2：自定义启动参数
    """
    print("="*70)
    print("示例2：自定义启动参数")
    print("="*70)
    
    # 自定义控制参数
    platform = ElectrolysisWaterPlatform(
        port="COM5",
        baudrate=115200,
        default_mode=1,              # 恒流模式
        default_current_ma=1500,     # 1500mA
        default_voltage_mv=5000,     # 限压5000mV
        default_pump_percent=60.0,   # 泵速60%
        sampling_interval=0.5,       # 采样间隔0.5秒
        auto_start_control=True      # 自动启动控制
    )
    
    time.sleep(3)
    
    try:
        print("\n监控数据（自定义参数）...")
        for i in range(10):
            data = platform.get_sensor_data()
            if data[0]:
                print(f"[{i+1}/10] I={data[1]:.1f}mA | V={data[2]:.1f}mV | T={data[3]:.2f}°C")
            time.sleep(platform.sampling_interval)
        
        # 动态修改控制参数
        print("\n修改控制参数：切换到恒压模式")
        platform.send_command(
            mode=0,              # 恒压模式
            voltage_mv=3000,     # 3000mV
            current_ma=1000      # 限流1000mA
        )
        
        print("继续监控...")
        for i in range(5):
            data = platform.get_sensor_data()
            if data[0]:
                print(f"[{i+1}/5] I={data[1]:.1f}mA | V={data[2]:.1f}mV")
            time.sleep(platform.sampling_interval)
            
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        platform.stop()


def example_config_dict():
    """
    示例3：使用配置字典
    """
    print("="*70)
    print("示例3：使用配置字典")
    print("="*70)
    
    config = {
        "port": "COM5",
        "baudrate": 115200,
        "default_mode": 0,
        "default_voltage_mv": 2500,
        "default_current_ma": 800,
        "default_pump_percent": 55.0,
        "sampling_interval": 2.0,
        "auto_start_control": True
    }
    
    platform = ElectrolysisWaterPlatform(config=config)
    time.sleep(3)
    
    try:
        print("\n采样5次...")
        for i in range(5):
            data = platform.get_sensor_data()
            print(f"采样{i+1}: {data}")
            time.sleep(config["sampling_interval"])
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        platform.stop()


# ================== 主入口 ==================
if __name__ == "__main__":
    print("="*70)
    print("电解水平台示例程序")
    print("="*70)
    print("\n选择运行模式:")
    print("  1. 平台 + 控制器（深度学习控制）")
    print("  2. 简单启动（手动控制）")
    print()
    
    # 运行示例1：平台 + 控制器（推荐）
    example_with_controller()
    
    # 或运行示例2：简单启动
    # example_simple_start()
