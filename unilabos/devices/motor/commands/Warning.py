from serial import Serial
from ..Consts import Config
from ..Utils import create_command, send_command


def create_elimate_warning_command(DEVICE_ADDRESS):
    return create_command(DEVICE_ADDRESS, 0x06, 0x0145, 0x0087)


def run_elimate_warning(ser: Serial, DEVICE_ADDRESS):
    send_command(ser, create_elimate_warning_command(DEVICE_ADDRESS))
    print("清除警报")
