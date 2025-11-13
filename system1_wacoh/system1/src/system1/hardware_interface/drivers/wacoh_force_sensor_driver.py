import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Optional, Dict
import threading
import time

from system1.hardware_interface.abstract_drivers import AbstractForceSensor
from ...config.config_loader import get_config_loader
from .wacoh_force_sensor_lib import WacohForceSensor

# Load YAML configuration
_config_loader = get_config_loader()
_common_config = _config_loader.get_common_config()
FORCE_SENSOR_TYPE = _common_config['FORCE_SENSOR_TYPE']
_sensor_config = _config_loader.get_sensor_config()
SENSOR_MOUNT_TRANSFORMS = _sensor_config.get('mount_transforms', {})


class WacohForceSensorDriver(AbstractForceSensor):
    """Concrete class for handling WACOH force sensor."""

    def __init__(self,
                logger_callback=None,
                **sensor_params
                ):

        # Get parameters from YAML config
        wacoh_params = _sensor_config.get('wacoh', {}).get('sensor_params', {})
        params = {**wacoh_params, **sensor_params}

        self.port              = params["port"]
        self.baudrate          = params["baudrate"]
        self.timeout           = params.get("timeout", 1.0)
        self.bias_sample_count = params.get("bias_sample_count", 100)
        sensitivity_cfg = params.get("sensitivity")
        self._sensitivity = None
        if isinstance(sensitivity_cfg, dict):
            try:
                self._sensitivity = WacohForceSensor.Sensitivity(
                    Fx=sensitivity_cfg.get("Fx", WacohForceSensor.DefaultSensitivity.Fx),
                    Fy=sensitivity_cfg.get("Fy", WacohForceSensor.DefaultSensitivity.Fy),
                    Fz=sensitivity_cfg.get("Fz", WacohForceSensor.DefaultSensitivity.Fz),
                    Mx=sensitivity_cfg.get("Mx", WacohForceSensor.DefaultSensitivity.Mx),
                    My=sensitivity_cfg.get("My", WacohForceSensor.DefaultSensitivity.My),
                    Mz=sensitivity_cfg.get("Mz", WacohForceSensor.DefaultSensitivity.Mz),
                )
            except Exception as exc:
                self.log(f"Invalid sensitivity config, using defaults: {exc}", level="warn")
                self._sensitivity = None

        self.log = logger_callback or (lambda msg, level="info":
                                        print(f"[WFS][{level}] {msg}"))

        self.sensor = WacohForceSensor(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            sensitivity=self._sensitivity,
        )

        self.zeros = np.zeros(6, dtype=int)

        mount = SENSOR_MOUNT_TRANSFORMS.get(FORCE_SENSOR_TYPE, {})
        quat = mount.get("rotation", [0, 0, 0, 1])
        self.R_tool2sensor = R.from_quat(quat).as_matrix()
        self.R_sensor2tool = self.R_tool2sensor.T

        # Background reader
        self._reader_thread = None
        self._reader_stop = threading.Event()
        self._last_raw_lock = threading.Lock()
        self._last_raw = None
        self._last_read_time = None

    def connect(self) -> bool:
        self.log(f"Opening Wacoh sensor at {self.port}@{self.baudrate}")
        if not self.sensor.open():
            self.log("Failed to open serial port", level="error")
            return False
        # Ensure we start from a clean state
        try:
            self.sensor.stop_continuous()
        except Exception:
            pass
        if hasattr(self.sensor, 'ser') and self.sensor.ser is not None:
            try:
                self.sensor.ser.reset_input_buffer()
                self.sensor.ser.timeout = max(self.timeout, 0.05)
            except Exception:
                pass

        # Perform zero calibration using single-shot reads
        self.log(f"Zero calibration: {self.bias_sample_count} samples")
        samples = []
        attempts = 0
        needed = max(1, int(self.bias_sample_count))
        max_attempts = needed * 10

        while len(samples) < needed and attempts < max_attempts:
            attempts += 1
            pkt = self.sensor.get_sensor()
            if pkt.get("err"):
                time.sleep(0.002)
                continue
            samples.append(pkt["data"])
            time.sleep(0.002)

        if len(samples) < needed:
            self.log("Zero calibration collected insufficient data; using zeros", level="warn")
            self.zeros = np.zeros(6, dtype=int)
        else:
            arr = np.array(samples, dtype=int)
            mean_vals = arr.mean(axis=0).astype(int)
            self.zeros = np.array(mean_vals, dtype=int)
            self.log(f"Zero bias = {self.zeros.tolist()}")
            with self._last_raw_lock:
                self._last_raw = arr[-1].copy()
                self._last_read_time = time.time()

        # Start background reader thread to continuously update last sample
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, name="wacoh_reader", daemon=True)
        self._reader_thread.start()

        return True

    def disconnect(self) -> None:
        # Stop reader
        try:
            self._reader_stop.set()
            if self._reader_thread is not None:
                self._reader_thread.join(timeout=1.0)
        except Exception:
            pass

        # Stop continuous mode and close serial
        try:
            self.sensor.stop_continuous()
        except Exception:
            pass
        self.sensor.close()
        self.log("Sensor disconnected")

    def get_raw_data(self) -> Optional[np.ndarray]:
        # Return last raw sample read by background reader (non-blocking).
        # If background reader hasn't produced a sample yet, try one direct non-blocking read.
        deadline = time.time() + 0.1
        while time.time() < deadline:
            with self._last_raw_lock:
                if self._last_raw is not None:
                    return np.array(self._last_raw, dtype=int)
            time.sleep(0.001)
        return None

    def get_last_read_time(self) -> Optional[float]:
        return self._last_read_time

    def _reader_loop(self) -> None:
        """Continuously read from sensor in background thread."""
        while not self._reader_stop.is_set():
            try:
                pkt = self.sensor.get_sensor()
                if pkt.get("err"):
                    time.sleep(0.001)
                    continue
                data = np.array(pkt["data"], dtype=int)
                with self._last_raw_lock:
                    self._last_raw = data
                    self._last_read_time = time.time()
            except Exception as e:
                self.log(f"Reader loop exception: {e}", level="warn")
                time.sleep(0.01)

    def get_wrench(self, current_pose: Dict = None) -> Optional[np.ndarray]:
        raw = self.get_raw_data()
        if raw is None:
            return None

        ftm = self.sensor.snsvalue_to_force(
            data=raw.tolist(),
            meandata=self.zeros.tolist(),
            sensitivity=self._sensitivity,
        )
        f_s = np.array(ftm[:3], dtype=float)
        m_s = np.array(ftm[3:], dtype=float)

        f_t = self.R_sensor2tool.dot(f_s)
        m_t = self.R_sensor2tool.dot(m_s)
        wrench_tool = np.hstack([f_t, m_t])

        if current_pose and "quat" in current_pose:
            try:
                R_base2tool = R.from_quat(current_pose["quat"])
                f_b = R_base2tool.inv().apply(f_t)
                m_b = R_base2tool.inv().apply(m_t)
                return np.hstack([f_b, m_b])
            except Exception as e:
                self.log(f"Base-frame transform failed: {e}", level="warn")

        return wrench_tool