import json
import serial

class Chiller():
# def xuanzheng_temp_set(tempset: int):
    # 设置目标温度
    def __init__(self, port,rate=9600):
        self.T_set = 24
        self.success = True
        self.ser = serial.Serial(
            port=port,  
            baudrate=rate,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=1
        )

    def modbus_crc(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 0x0001) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    def build_modbus_frame(self,device_address: int, function_code: int, register_address: int, value: int) -> bytes:
        frame = bytearray([
            device_address,
            function_code,
            (register_address >> 8) & 0xFF,
            register_address & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
        ])
        crc = self.modbus_crc(frame)
        return frame + crc

    def convert_temperature_to_modbus_value(self, temperature: float, decimal_points: int = 1) -> int:
        factor = 10 ** decimal_points
        value = int(temperature * factor)
        return value & 0xFFFF

    def set_temperature(self, command):
        T_set = json.loads(command)['temperature']
        self.T_set = int(T_set)
        self.success = False

        temperature_value = self.convert_temperature_to_modbus_value(self.T_set, decimal_points=1)
        device_address = 0x01
        function_code = 0x06  
        register_address = 0x000B  
        frame = self.build_modbus_frame(device_address, function_code, register_address, temperature_value)
        self.ser.write(frame)
        response = self.ser.read(8)
        self.success = True
    
    def stop(self):
        self.set_temperature(25)

if __name__ == '__main__':
    ch = Chiller(port='COM17')
    ch.set_temperature(20)