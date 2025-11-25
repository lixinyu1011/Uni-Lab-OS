# -*- coding: utf-8 -*-
import serial
import time
import csv
import threading
import os
from collections import deque
from typing import Dict, Any, Optional
from pylabrobot.resources import Deck

from unilabos.devices.workstation.workstation_base import WorkstationBase
from unilabos.devices.electrolysis_water_platfrom.electrolysis_deck import create_electrolysis_deck, ElectrolysisDeck
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
        deck: Optional[Deck] = None,
        port: str = DEFAULT_PORT,
        baudrate: int = DEFAULT_BAUDRATE,
        csv_path: Optional[str] = None,
        timeout: float = DEFAULT_TIMEOUT,
        *args,
        **kwargs):

        # 处理 deck 参数
        if deck is None and config:
            deck = config.get('deck')

        
        # 如果仍然为 None，创建默认的电解水平台专用 Deck
        if deck is None:
            print("[INFO] 没有传入 deck，创建电解水平台专用 Deck（包含恒压源、恒流源、反应器）")
            deck = create_electrolysis_deck(
                deck_name="electrolysis_deck",
                size_x=800.0,  # Deck 长度
                size_y=600.0,  # Deck 宽度
                size_z=100.0,  # Deck 高度
                setup=True     # 自动配置资源
            )
        
        super().__init__(deck=deck, *args, **kwargs)
        
        
        # ========== 配置 ==========
        self.port = port
        self.baudrate = baudrate
        # 如果没有指定路径，默认保存在代码文件所在目录
        if csv_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.csv_path = os.path.join(current_dir, "stm32_data.csv")
        else:
            self.csv_path = csv_path
        self.ser_timeout = timeout
        self.chunk_read = 128
        
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
                # 只启动接收线程，不启动发送线程（发送线程需要用户输入）
                self.rx_thread = threading.Thread(target=self.rx_thread_fn, daemon=True, name="electrolysis_rx")
                self.rx_thread.start()
                print(f"[✓] 电解水平台串口连接成功: {self.port}")
                print(f"[✓] 数据接收线程已启动，CSV 文件: {self.csv_path}")
            else:
                print(f"[✗] 电解水平台串口连接失败: {self.port}")
                print(f"[提示] 请检查: 1) 串口号是否正确 2) 设备是否已连接 3) 串口是否被其他程序占用")
        except Exception as e:
            print(f"[✗] 电解水平台自动启动失败: {e}")
            import traceback
            traceback.print_exc()
    
    def open_serial(self, port: Optional[str] = None, baudrate: Optional[int] = None, timeout: Optional[float] = None) -> Optional[serial.Serial]:
        """打开串口"""
        port = port or self.port
        baudrate = baudrate or self.baudrate
        timeout = timeout or self.ser_timeout
        try:
            ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"[OK] 串口 {port} 已打开，波特率 {baudrate}")
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.ser = ser
            return ser
        except serial.SerialException as e:
            print(f"[ERR] 无法打开串口 {port}: {e}")
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
        from unilabos.ros.nodes.base_device_node import ROS2DeviceNode
        
        self._ros_node = ros_node
        print(f"[INFO] 电解水平台 ROS2 节点已就绪: {ros_node.device_id}")
        
        # 显示 Deck 上的资源信息
        if hasattr(self, 'deck') and self.deck is not None:
            if isinstance(self.deck, ElectrolysisDeck):
                print(f"[INFO] 电解水平台 Deck 资源配置:")
                print(f"  Deck 尺寸: {self.deck.get_size_x():.0f}×{self.deck.get_size_y():.0f}×{self.deck.get_size_z():.0f} mm")
                
                # 显示电源
                if hasattr(self.deck, 'power_sources') and self.deck.power_sources:
                    print(f"  电源 ({len(self.deck.power_sources)} 个):")
                    for name, source in self.deck.power_sources.items():
                        print(f"    - {name}: {source.max_voltage}V / {source.max_current}mA")
                
                # 显示反应器
                if hasattr(self.deck, 'reactors') and self.deck.reactors:
                    print(f"  反应器 ({len(self.deck.reactors)} 个):")
                    for name, reactor in self.deck.reactors.items():
                        print(f"    - {name}: {reactor.volume}mL")
            
            elif len(self.deck.children) > 0:
                print(f"[INFO] Deck 资源列表 ({len(self.deck.children)} 个):")
                for child in self.deck.children:
                    location = child.location
                    print(f"  - {child.name} ({child.category})")
                    print(f"    位置: X={location.x:.1f}mm, Y={location.y:.1f}mm, Z={location.z:.1f}mm")
        
        # 上传 deck 资源（如果存在）
        # 注意：只上传 deck，不上传设备本身，避免 PLR 资源转换错误
        if hasattr(self, 'deck') and self.deck is not None:
            try:
                print(f"[INFO] 正在上传 Deck 资源到云端...")
                ROS2DeviceNode.run_async_func(self._ros_node.update_resource, True, **{
                    "resources": [self.deck]
                })
                print(f"[✓] Deck 资源上传成功")
            except Exception as e:
                # Deck 上传失败不应该影响设备运行
                print(f"[WARN] Deck 资源上传失败（不影响设备运行）: {e}")
    
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


# ================== 主入口 ==================
if __name__ == "__main__":
    # 创建一个简单的 Deck 用于测试
    from pylabrobot.resources import Deck
    
    deck = Deck()
    platform = ElectrolysisWaterPlatform(deck)
    platform.start()
