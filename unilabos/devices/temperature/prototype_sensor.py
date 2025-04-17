import serial
import struct

class TempSensor:
    def __init__(self,port,baudrate=9600):
        
    # 配置串口
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

    def calculate_crc(self,data):
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

    def build_modbus_request(self, device_id, function_code, register_address, register_count):
        request = struct.pack('>BBHH', device_id, function_code, register_address, register_count)
        crc = self.calculate_crc(request)
        request += struct.pack('<H', crc)
        return request

    def read_modbus_response(self,response):
        if len(response) < 5:
            return None
        data = response[:-2]
        crc_received = struct.unpack('<H', response[-2:])[0]
        crc_calculated = self.calculate_crc(data)
        if crc_received == crc_calculated:
            return data[3:]
        return None

    def send_command(self ,command):
        function_code = 0x04  # 读输入寄存器
        register_address = 0x0003  # 温度高位寄存器地址
        register_count = 0x0002  # 读取两个寄存器

        request = self.build_modbus_request(command, function_code, register_address, register_count)
        self.ser.write(request)
        response = self.ser.read(9)  # 读取9字节的响应

        data = self.read_modbus_response(response)
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

    