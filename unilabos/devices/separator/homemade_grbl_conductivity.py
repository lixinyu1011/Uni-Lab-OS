import json
import serial
import time as systime
import threading


class SeparatorController:
    """Controls the operation of a separator device using serial communication.

    This class manages the interaction with both an executor and a sensor through serial ports, allowing for stirring, settling, and extracting operations based on sensor data.

    Args:
        port_executor (str): The serial port for the executor device.
        port_sensor (str): The serial port for the sensor device.
        baudrate_executor (int, optional): The baud rate for the executor device. Defaults to 115200.
        baudrate_sensor (int, optional): The baud rate for the sensor device. Defaults to 115200.

    Attributes:
        serial_executor (Serial): The serial connection to the executor device.
        serial_sensor (Serial): The serial connection to the sensor device.
        sensordata (float): The latest sensor data read from the sensor device.
        success (bool): Indicates whether the last operation was successful.
        status (str): The current status of the controller, which can be 'Idle', 'Stirring', 'Settling', or 'Extracting'.
    """
    def __init__(
        self,
        port_executor: str,
        port_sensor: str,
        baudrate_executor: int = 115200,
        baudrate_sensor: int = 115200,
    ):

        self.serial_executor = serial.Serial(port_executor, baudrate_executor)
        self.serial_sensor = serial.Serial(port_sensor, baudrate_sensor)

        if not self.serial_executor.is_open:
            self.serial_executor.open()
        if not self.serial_sensor.is_open:
            self.serial_sensor.open()

        self.sensordata = 0.00
        self.success = False

        self.status = "Idle"  # 'Idle', 'Stirring' ,'Settling' , 'Extracting',

        systime.sleep(2)
        t = threading.Thread(target=self.read_sensor_loop, daemon=True)
        t.start()

    def write(self, data):
        self.serial_executor.write(data)
        a = self.serial_executor.read_all()

    def stir(self, stir_time: float = 10, stir_speed: float = 300, settling_time: float = 10):
        """Controls the stirring operation of the separator.

        This function initiates a stirring process for a specified duration and speed, followed by a settling phase. It updates the status of the controller and communicates with the executor to perform the stirring action.

        Args:
            stir_time (float, optional): The duration for which to stir, in seconds. Defaults to 10.
            stir_speed (float, optional): The speed of stirring, in RPM. Defaults to 300.
            settling_time (float, optional): The duration for which to settle after stirring, in seconds. Defaults to 10.

        Returns:
            None
        """
        self.success = False
        start_time = systime.time()
        self.status = "Stirring"
        
        stir_speed_second = stir_speed / 60
        cmd = f"G91 Z{stir_speed_second}\n"
        cmd_data = bytearray(cmd, "ascii")
        self.write(bytearray(f"$112={stir_speed_second}", "ascii"))
        while self.status != "Idle":
            # print(self.sensordata)
            if self.status == "Stirring":
                if systime.time() - start_time < stir_time:
                    self.write(cmd_data)
                    systime.sleep(1)
                else:
                    self.status = "Settling"
                    start_time = systime.time()

            elif self.status == "Settling":
                if systime.time() - start_time > settling_time:
                    break
        self.success = True

    def valve_open(self, condition, value):
        """Opens the valve, then wait to close the valve based on a specified condition.

        This function sends a command to open the valve and continuously monitors the sensor data until the specified condition is met. Once the condition is satisfied, it closes the valve and updates the status of the controller.

        Args:
            
            condition (str): The condition to be monitored, either 'delta' or 'time'.
            value (float): The threshold value for the condition.
            `delta > 0.05`, `time > 60`

        Returns:
            None
        """
        if condition not in ["delta", "time"]:
            raise ValueError("Invalid condition")
        elif condition == "delta":
            valve_position = 0.66
        else:
            valve_position = 0.8

        self.write((f"G91 X{valve_position}\n").encode())
        last = self.sensordata
        start_time = systime.time()
        while True:
            data = self.sensordata
            delta = abs(data - last)
            time = systime.time() - start_time

            if eval(f"{condition} > {value}"):
                break
            last = data
            systime.sleep(0.05)

        self.status = "Idle"
        
        self.write((f"G91 X-{valve_position}\n").encode())

    def valve_open_cmd(self,command:str):
        self.success = False
        try:
            cmd_dict = json.loads(command)
            self.valve_open(**cmd_dict)
            self.success = True
        except Exception as e:
            raise f"error: {e}"

    def read_sensor_loop(self):
        while True:
            msg = self.serial_sensor.readline()
            ascii_string = "".join(chr(byte) for byte in msg)
            # print(msg)
            if ascii_string.startswith("A3"):
                try:
                    self.sensordata = float(ascii_string.split(":")[1])
                except:
                    return
            self.serial_sensor.read_all()
            systime.sleep(0.05)


if __name__ == "__main__":

    e = SeparatorController(port_sensor="COM40", port_executor="COM41")
    print(e.status)
    e.stir(10, 720, 10)
    e.valve_open("delta", 0.3)
