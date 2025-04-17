import serial
import time
import struct
import tkinter as tk
from tkinter import ttk

# 配置串口
ser = serial.Serial(
    port='COM13',
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def build_modbus_request(device_id, function_code, register_address, register_count):
    request = struct.pack('>BBHH', device_id, function_code, register_address, register_count)
    crc = calculate_crc(request)
    request += struct.pack('<H', crc)
    return request

def read_modbus_response(response):
    if len(response) < 5:
        return None
    data = response[:-2]
    crc_received = struct.unpack('<H', response[-2:])[0]
    crc_calculated = calculate_crc(data)
    if crc_received == crc_calculated:
        return data[3:]
    return None

def read_temperature():
    device_id = 0xC0  # 设备地址
    function_code = 0x04  # 读输入寄存器
    register_address = 0x0003  # 温度高位寄存器地址
    register_count = 0x0002  # 读取两个寄存器

    request = build_modbus_request(device_id, function_code, register_address, register_count)
    ser.write(request)
    response = ser.read(9)  # 读取9字节的响应

    data = read_modbus_response(response)
    if data is None:
        return None

    high_value = struct.unpack('>H', data[:2])[0]
    low_value = struct.unpack('>H', data[2:])[0]

    # 组合高位和低位并计算实际温度
    raw_temperature = (high_value << 16) | low_value
    if raw_temperature & 0x8000:  # 如果低位寄存器最高位为1，表示负值
        raw_temperature -= 0x10000  # 转换为正确的负数表示

    actual_temperature = raw_temperature / 10.0
    return actual_temperature

def update_temperature_label():
    temperature = read_temperature()
    if temperature is not None:
        temperature_label.config(text=f"反应温度: {temperature:.1f} °C")
    else:
        temperature_label.config(text="Error reading temperature")
    root.after(1000, update_temperature_label)  # 每秒更新一次

# 创建主窗口
root = tk.Tk()
root.title("反应温度监控")

# 创建标签来显示温度
temperature_label = ttk.Label(root, text="反应温度: -- °C", font=("Helvetica", 20))
temperature_label.pack(padx=20, pady=20)

# 开始更新温度
update_temperature_label()

# 运行主循环
root.mainloop()
