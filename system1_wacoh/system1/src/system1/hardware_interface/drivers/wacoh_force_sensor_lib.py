import serial
import time
import numpy as np
from collections import namedtuple

class WacohForceSensor:

    Sensitivity = namedtuple("Sensitivity", ["Fx", "Fy", "Fz", "Mx", "My", "Mz"])
    DefaultSensitivity = Sensitivity(
        Fx=32.980, Fy=33.145, Fz=32.800,
        Mx=1649.000, My=1645.500, Mz=1656.500
    )

    def __init__(self, port, baudrate=921600, timeout=1.0, sensitivity: Sensitivity = None):
        if isinstance(port, int):
            self.devname = f"/dev/ttyUSB{port}"
        elif isinstance(port, str):
            self.devname = port
        else:
            raise ValueError(f"invalid port type: {port!r}")

        self.baudrate = baudrate
        self.timeout = timeout
        self.sensitivity = sensitivity or WacohForceSensor.DefaultSensitivity
        self.ser = None

    def open(self) -> bool:
        print(f"[wacoh_lib] Opening {self.devname} @ {self.baudrate} baud, timeout={self.timeout}")
        try:
            self.ser = serial.Serial(
                self.devname,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            print(f"[wacoh_lib] {self.devname} opened successfully.")
            self.stop_continuous()
            return True
        except serial.SerialException as e:
            print(f"[wacoh_lib] Failed to open {self.devname}: {e}")
            self.ser = None
            return False

    def close(self) -> None:
        if self.ser is None:
            return
        try:
            self.ser.write(b'E')
            self.ser.close()
            print("[wacoh_lib] Serial port closed.")
        except serial.SerialException as e:
            print(f"[wacoh_lib] close error: {e}")
        finally:
            self.ser = None

    def read_packet(self):
        if self.ser is None:
            return None
        try:
            packet = self.ser.readline()
        except serial.SerialException as e:
            print(f"[wacoh_lib] readline error: {e}")
            return None
        if len(packet) != 27:
            return None

        header = packet[0]
        data = [packet[1 + i*4 : 1 + (i+1)*4] for i in range(6)]
        return {"header": header, "data": data}

    def init_sensor(self, n_samples=100) -> dict:
        if self.ser is None:
            return {"err": True, "data": [0]*6}

        try:
            self.ser.write(b'S')
        except serial.SerialException as e:
            print(f"[wacoh_lib] init_sensor write S error: {e}")
            return {"err": True, "data": [0]*6}

        datas = []
        for _ in range(n_samples):
            pkt = self.read_packet()
            if pkt is not None:
                vals = [int(d, 16) for d in pkt["data"]]
                datas.append(vals)
            else:
                datas.append([0]*6)
            time.sleep(0.0005)

        arr = np.array(datas)
        mean_vals = arr.mean(axis=0).astype(int).tolist()

        try:
            self.ser.write(b'E')
        except serial.SerialException:
            pass

        print(f"[wacoh_lib] init_sensor mean: {mean_vals}")
        return {"err": False, "data": mean_vals}

    def get_sensor(self) -> dict:
        if self.ser is None:
            return {"err": True, "data": [0]*6}

        try:
            self.ser.write(b'R')
        except serial.SerialException as e:
            print(f"[wacoh_lib] get_sensor write R error: {e}")
            return {"err": True, "data": [0]*6}

        pkt = self.read_packet()
        if pkt is None:
            return {"err": True, "data": [0]*6}

        vals = [int(d, 16) for d in pkt["data"]]
        return {"err": False, "data": vals}

    def start_continuous(self) -> bool:
        if self.ser is None:
            return False
        try:
            self.ser.write(b'S')
            return True
        except serial.SerialException as e:
            print(f"[wacoh_lib] start_continuous error: {e}")
            return False

    def read_continuous(self) -> list:
        if self.ser is None:
            return [0]*6
        pkt = self.read_packet()
        if pkt is None:
            return [0]*6
        return [int(d, 16) for d in pkt["data"]]

    def stop_continuous(self) -> bool:
        if self.ser is None:
            return False
        try:
            self.ser.write(b'E')
            return True
        except serial.SerialException as e:
            print(f"[wacoh_lib] stop_continuous error: {e}")
            return False

    def snsvalue_to_force(self, data: list, meandata: list, sensitivity: Sensitivity = None) -> list:

        if len(data) != 6 or len(meandata) != 6:
            raise ValueError("data and meandata must have length 6")
        sens = sensitivity or self.sensitivity
        return [
            (data[0] - meandata[0]) / sens.Fx,
            (data[1] - meandata[1]) / sens.Fy,
            (data[2] - meandata[2]) / sens.Fz,
            (data[3] - meandata[3]) / sens.Mx,
            (data[4] - meandata[4]) / sens.My,
            (data[5] - meandata[5]) / sens.Mz,
        ]