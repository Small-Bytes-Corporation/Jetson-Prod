"""
RTK GNSS controller for Point One Navigation (Quectel LG69T).
Streams Fusion Engine protocol: Pose (position, solution type) and IMU (accel, gyro).
"""

import serial
from .config import DEFAULT_RTK_SERIAL_PORT, RTK_BAUDRATE

try:
    from fusion_engine_client.parsers.decoder import FusionEngineDecoder
    from fusion_engine_client.messages import MessageType, message_type_to_class
    _FUSION_ENGINE_AVAILABLE = True
    _FUSION_ENGINE_IMPORT_ERROR = None
except ImportError as e:
    _FUSION_ENGINE_AVAILABLE = False
    _FUSION_ENGINE_IMPORT_ERROR = e
    FusionEngineDecoder = None
    MessageType = None
    message_type_to_class = None


class RTKController:
    """
    Controller for Point One RTK GNSS device. Reads Fusion Engine stream
    and exposes latest pose (position, solution type) and IMU (accel, gyro).
    """

    def __init__(self, serial_port=DEFAULT_RTK_SERIAL_PORT, baudrate=RTK_BAUDRATE, enabled=True):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.enabled = enabled
        self.serial_conn = None
        self._initialized = False
        self._decoder = None
        self.last_pose = None
        self.last_imu = None

    def initialize(self):
        if not _FUSION_ENGINE_AVAILABLE:
            err = _FUSION_ENGINE_IMPORT_ERROR
            print("[RTK] fusion-engine-client import failed:", err if err else "pip install fusion-engine-client")
            return False
        if not self.enabled:
            print("[RTK] Disabled (mock mode)")
            self._initialized = True
            return True
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.1,
            )
            self._decoder = FusionEngineDecoder(warn_on_error=False)
            self._initialized = True
            print(f"[RTK] Initialized on {self.serial_port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"[RTK] Init failed: {e}")
            return False

    def update(self):
        """
        Read available bytes from serial and decode Fusion Engine messages.
        Updates last_pose and last_imu when POSE or IMU messages are received.
        """
        if not self._initialized or self.serial_conn is None or self._decoder is None:
            return
        try:
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                messages = self._decoder.on_data(data)
                for result in messages:
                    if len(result) < 2:
                        continue
                    header, contents = result[0], result[1]
                    if isinstance(contents, bytes):
                        continue
                    msg_type = header.message_type
                    # POSE: position, solution type, velocity, ypr
                    pose_type = getattr(MessageType, "POSE", None) or getattr(MessageType, "ROS_POSE", None)
                    if pose_type is not None and msg_type == pose_type:
                        self.last_pose = self._pose_to_dict(contents)
                    # IMU: accel, gyro
                    imu_type = (
                        getattr(MessageType, "IMU_OUTPUT", None)
                        or getattr(MessageType, "ROS_IMU", None)
                        or getattr(MessageType, "RAW_IMU_OUTPUT", None)
                    )
                    if imu_type is not None and msg_type == imu_type:
                        self.last_imu = self._imu_to_dict(contents)
        except Exception as e:
            print(f"[RTK] Error: {e}")

    def _pose_to_dict(self, msg):
        """Convert PoseMessage (or ROS pose) to a simple dict."""
        out = {}
        if hasattr(msg, "lla_deg"):
            out["lla_deg"] = list(msg.lla_deg) if hasattr(msg.lla_deg, "__iter__") else msg.lla_deg
        if hasattr(msg, "solution_type"):
            out["solution_type"] = getattr(msg.solution_type, "name", msg.solution_type)
        if hasattr(msg, "position_std_enu_m"):
            out["position_std_enu_m"] = list(msg.position_std_enu_m) if hasattr(msg.position_std_enu_m, "__iter__") else msg.position_std_enu_m
        if hasattr(msg, "gps_time"):
            out["gps_time"] = msg.gps_time
        if hasattr(msg, "p1_time"):
            out["p1_time"] = msg.p1_time
        if hasattr(msg, "ypr_deg"):
            out["ypr_deg"] = list(msg.ypr_deg) if hasattr(msg.ypr_deg, "__iter__") else msg.ypr_deg
        if hasattr(msg, "velocity_body_mps"):
            out["velocity_body_mps"] = list(msg.velocity_body_mps) if hasattr(msg.velocity_body_mps, "__iter__") else msg.velocity_body_mps
        return out

    def _imu_to_dict(self, msg):
        """Convert IMU message to dict with accel_xyz (m/s²) and gyro_xyz (rad/s)."""
        out = {}
        # Acceleration: m/s² (e.g. acceleration_mps2 or accel_xyz)
        accel = getattr(msg, "acceleration_mps2", None) or getattr(msg, "accel_xyz", None) or getattr(msg, "accel_mps2", None)
        if accel is not None:
            out["accel_xyz"] = list(accel) if hasattr(accel, "__iter__") else accel
        # Angular velocity: rad/s (e.g. angular_velocity_rps or gyro_xyz)
        gyro = getattr(msg, "angular_velocity_rps", None) or getattr(msg, "gyro_xyz", None) or getattr(msg, "gyro_rps", None)
        if gyro is not None:
            out["gyro_xyz"] = list(gyro) if hasattr(gyro, "__iter__") else gyro
        if hasattr(msg, "p1_time"):
            out["p1_time"] = msg.p1_time
        return out if out else None

    def get_latest_pose(self):
        """Return latest pose as dict (lla_deg, solution_type, gps_time, etc.) or None."""
        return self.last_pose

    def get_latest_imu(self):
        """Return latest IMU as dict (accel_xyz, gyro_xyz) or None if no IMU message yet."""
        return self.last_imu

    def is_available(self):
        return self._initialized

    def stop(self):
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
                print("[RTK] Serial connection closed")
            except Exception as e:
                print(f"[RTK] Error during stop: {e}")
            finally:
                self.serial_conn = None
        self._initialized = False
        self._decoder = None
