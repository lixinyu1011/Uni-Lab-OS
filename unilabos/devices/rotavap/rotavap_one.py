import json
from serial import Serial
import time as time_
import threading

class RotavapOne:
    def __init__(self, port, rate=9600):
        self.ser = Serial(port,rate)
        self.pump_state       = 'h'
        self.pump_time        = 0
        self.rotate_state     = 'h'
        self.rotate_time      = 0
        self.success          = True
        if not self.ser.is_open:
            self.ser.open()
        
        # self.main_loop()
        t = threading.Thread(target=self.main_loop ,daemon=True)
        t.start()

    def cmd_write(self, cmd):
        self.ser.write(cmd)
        self.ser.read_all()

    def set_rotate_time(self, time):
        self.success          = False
        self.rotate_time = time
        self.success          = True


    def set_pump_time(self, time):
        self.success          = False
        self.pump_time = time
        self.success          = True

    def set_timer(self, command):
        self.success = False
        timer = json.loads(command)

        rotate_time = timer['rotate_time']
        pump_time = timer['pump_time']

        self.rotate_time = rotate_time
        self.pump_time = pump_time

        self.success = True


    def main_loop(self):
        param = ['rotate','pump']
        while True:
            for i in param:
                if getattr(self, f'{i}_time') <= 0:
                    setattr(self, f'{i}_state','l')
                else:
                    setattr(self, f'{i}_state','h')
                    setattr(self, f'{i}_time',getattr(self, f'{i}_time')-1)
            cmd = f'1{self.pump_state}2{self.rotate_state}3l4l\n'
            self.cmd_write(cmd.encode())

            time_.sleep(1)

if __name__ == '__main__':
    ro = RotavapOne(port='COM15')
