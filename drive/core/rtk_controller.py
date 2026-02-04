"""
RTK GNSS controller for Point One Navigation (Quectel LG69T).
Streams Fusion Engine protocol: Pose (position, solution type) and IMU (accel, gyro).

Supports two connection modes:
- Serial: direct connection to RTK device (no NTRIP corrections - will show Invalid/NaN)
- TCP: connect to p1-runner's TCP output (run init_rtk with --tcp PORT). p1-runner
  feeds Polaris NTRIP corrections to the RTK and forwards Fusion Engine over TCP.
  Use TCP when you need valid RTK fix (non-NaN values).
"""

import socket
import serial
import time
from .config import DEFAULT_RTK_SERIAL_PORT, RTK_BAUDRATE

# Connection monitoring constants
CONNECTION_TIMEOUT = 5.0  # seconds without data (serial mode)
CONNECTION_TIMEOUT_TCP = 30.0  # seconds without data (TCP mode - p1-runner can have natural gaps)
MAX_RECONNECT_ATTEMPTS = 3  # maximum reconnection attempts before giving up
RECONNECT_DELAY = 1.0  # seconds to wait before reconnecting

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

    def __init__(self, serial_port=DEFAULT_RTK_SERIAL_PORT, baudrate=RTK_BAUDRATE, enabled=True,
                 tcp_host=None, tcp_port=None):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.enabled = enabled
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        self._use_tcp = tcp_host is not None and tcp_port is not None
        self.serial_conn = None
        self._tcp_socket = None
        self._initialized = False
        self._decoder = None
        self.last_pose = None
        self.last_imu = None
        self.last_valid_pose = None  # Fallback when Invalid/NaN (Point One: use last known good)
        # Connection monitoring
        self._last_data_time = None
        self._reconnect_attempts = 0
        self._connection_lost = False

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
            # Close existing connection if any
            self._close_connection()
            
            if self._use_tcp:
                self._tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._tcp_socket.settimeout(0.5)
                self._tcp_socket.connect((self.tcp_host, self.tcp_port))
                self._tcp_socket.setblocking(False)
                print(f"[RTK] Initialized via TCP {self.tcp_host}:{self.tcp_port} (p1-runner Polaris)")
            else:
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    timeout=0.1,
                )
                print(f"[RTK] Initialized on {self.serial_port} @ {self.baudrate}")
                print("[RTK] Note: Serial-only mode has no NTRIP corrections - run init_rtk with --tcp for valid RTK fix")
            
            self._decoder = FusionEngineDecoder(warn_on_error=False)
            self._initialized = True
            self._last_data_time = time.time()
            self._reconnect_attempts = 0
            self._connection_lost = False
            return True
        except Exception as e:
            print(f"[RTK] Init failed: {e}")
            self._initialized = False
            self._close_connection()
            return False

    def _close_connection(self):
        """Close serial or TCP connection."""
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        if self._tcp_socket is not None:
            try:
                self._tcp_socket.close()
            except Exception:
                pass
            self._tcp_socket = None

    def _check_connection(self):
        """
        Check if connection is still alive based on timeout and port status.
        Returns True if connection is healthy, False if lost.
        """
        if not self._initialized:
            return False
        if self._use_tcp:
            if self._tcp_socket is None:
                return False
            # TCP: longer timeout (p1-runner stream can have natural gaps)
            if self._last_data_time is not None:
                if time.time() - self._last_data_time > CONNECTION_TIMEOUT_TCP:
                    return False
            return True
        else:
            if self.serial_conn is None or not self.serial_conn.is_open:
                return False
            if self._last_data_time is not None:
                if time.time() - self._last_data_time > CONNECTION_TIMEOUT:
                    return False
            return True
    
    def _reinitialize(self):
        """
        Attempt to reinitialize the RTK connection.
        Returns True if successful, False otherwise.
        """
        if self._reconnect_attempts >= MAX_RECONNECT_ATTEMPTS:
            print(f"[RTK] Max reconnection attempts ({MAX_RECONNECT_ATTEMPTS}) reached. Disabling RTK.")
            self._initialized = False
            self.enabled = False
            return False
        
        self._reconnect_attempts += 1
        print(f"[RTK] Attempting to reconnect (attempt {self._reconnect_attempts}/{MAX_RECONNECT_ATTEMPTS})...")
        
        # Close existing connection
        self._close_connection()
        
        # Wait before reconnecting
        time.sleep(RECONNECT_DELAY)
        
        # Try to reinitialize
        if self.initialize():
            print(f"[RTK] Reconnection successful!")
            self._connection_lost = False
            return True
        else:
            print(f"[RTK] Reconnection failed.")
            return False

    def update(self):
        """
        Read available bytes from serial and decode Fusion Engine messages.
        Updates last_pose and last_imu when POSE or IMU messages are received.
        Automatically detects connection loss and attempts reconnection.
        """
        if not self.enabled:
            return
        
        # Check connection health
        if not self._check_connection():
            if not self._connection_lost:
                self._connection_lost = True
                print(f"[RTK] Connection lost detected (timeout or port closed). Attempting reconnection...")
            # Try to reinitialize
            if not self._reinitialize():
                return  # Give up if reconnection failed
        
        if not self._initialized or self._decoder is None:
            return
        if not self._use_tcp and self.serial_conn is None:
            return
        if self._use_tcp and self._tcp_socket is None:
            return
        
        try:
            if self._use_tcp:
                try:
                    data = self._tcp_socket.recv(4096)
                    if not data:
                        raise ConnectionError("TCP connection closed by peer")
                except BlockingIOError:
                    return  # No data available yet (non-blocking socket)
                except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError, ConnectionError) as e:
                    raise  # Re-raise to trigger reconnection logic in outer except
            else:
                if self.serial_conn.in_waiting == 0:
                    return
                data = self.serial_conn.read(self.serial_conn.in_waiting)
            
            # Process data (from TCP or serial)
            self._last_data_time = time.time()  # Any data received = connection alive
            messages = self._decoder.on_data(data)
            data_received = False
            for result in messages:
                if len(result) < 2:
                    continue
                header, contents = result[0], result[1]
                if isinstance(contents, bytes):
                    continue
                msg_type = header.message_type
                # POSE: position, solution type, velocity, ypr
                pose_type = (
                    getattr(MessageType, "POSE", None) or 
                    getattr(MessageType, "ROS_POSE", None) or
                    getattr(MessageType, "POSE_MESSAGE", None)
                )
                
                # Debug: log message types received (only once, first 10 types)
                if not hasattr(self, '_logged_msg_types'):
                    self._logged_msg_types = set()
                    self._msg_type_count = {}
                if msg_type not in self._logged_msg_types:
                    msg_type_name = getattr(msg_type, 'name', str(msg_type))
                    self._logged_msg_types.add(msg_type)
                    self._msg_type_count[msg_type] = 0
                    if len(self._logged_msg_types) <= 10:
                        print(f"[RTK] Received message type: {msg_type_name} (value: {msg_type})")
                
                if msg_type in self._msg_type_count:
                    self._msg_type_count[msg_type] += 1
                
                is_pose_msg = False
                if pose_type is not None:
                    is_pose_msg = (msg_type == pose_type)
                
                if not is_pose_msg:
                    if hasattr(contents, 'lla_deg') or hasattr(contents, 'solution_type') or hasattr(contents, 'position_std_enu_m'):
                        is_pose_msg = True
                        if not hasattr(self, '_pose_fallback_logged'):
                            msg_type_name = getattr(msg_type, 'name', str(msg_type))
                            print(f"[RTK] Using fallback pose detection (msg_type={msg_type_name}, value={msg_type})")
                            print(f"[RTK] Message has pose attributes: lla_deg={hasattr(contents, 'lla_deg')}, "
                                  f"solution_type={hasattr(contents, 'solution_type')}")
                            self._pose_fallback_logged = True
                
                if is_pose_msg:
                    pose_dict = self._pose_to_dict(contents)
                    self.last_pose = pose_dict
                    data_received = True
                    solution_type = pose_dict.get("solution_type", "Unknown")
                    lla = pose_dict.get("lla_deg")
                    has_nan = False
                    if lla is not None and hasattr(lla, "__iter__"):
                        has_nan = any(x != x for x in lla if x is not None)
                    is_invalid = (solution_type == "Invalid" or solution_type == "INVALID") and has_nan
                    if is_invalid:
                        # Use last valid pose as fallback (Point One best practice)
                        if self.last_valid_pose is not None:
                            pass  # get_latest_pose will return it with solution_stale
                        # Log only when transitioning to Invalid (not every message)
                        if not getattr(self, '_was_invalid', False):
                            self._was_invalid = True
                            print(f"[RTK] Solution Invalid (vue ciel limitée). "
                                  f"Utilisation de la dernière position valide. Voir support.pointonenav.com")
                            print(f"[RTK] Antenne: plein ciel, extérieur recommandé. Polaris: corrections OK.")
                    else:
                        self._was_invalid = False
                        if not has_nan and lla is not None:
                            self.last_valid_pose = dict(pose_dict)
                            self.last_valid_pose["solution_stale"] = False
                # IMU: accel, gyro (config_tool uses 11000 for "imu")
                imu_type = (
                    getattr(MessageType, "IMU_OUTPUT", None)
                    or getattr(MessageType, "ROS_IMU", None)
                    or getattr(MessageType, "RAW_IMU_OUTPUT", None)
                    or getattr(MessageType, "IMU_MEASUREMENT", None)
                )
                is_imu_msg = False
                if imu_type is not None:
                    is_imu_msg = (msg_type == imu_type)
                if not is_imu_msg:
                    try:
                        is_imu_msg = (int(msg_type) == 11000)
                    except (TypeError, ValueError):
                        pass
                if is_imu_msg and not hasattr(self, '_imu_msg_logged'):
                    self._imu_msg_logged = True
                    print("[RTK] IMU message type detected, parsing...")
                if not is_imu_msg:
                    if (hasattr(contents, 'acceleration_mps2') or 
                        hasattr(contents, 'accel_xyz') or 
                        hasattr(contents, 'angular_velocity_rps') or 
                        hasattr(contents, 'gyro_xyz')):
                        is_imu_msg = True
                        if not hasattr(self, '_imu_fallback_logged'):
                            msg_type_name = getattr(msg_type, 'name', str(msg_type))
                            print(f"[RTK] Using fallback IMU detection (msg_type={msg_type_name}, value={msg_type})")
                            print(f"[RTK] Message has IMU attributes: accel={hasattr(contents, 'acceleration_mps2') or hasattr(contents, 'accel_xyz')}, "
                                  f"gyro={hasattr(contents, 'angular_velocity_rps') or hasattr(contents, 'gyro_xyz')}")
                            self._imu_fallback_logged = True
                if is_imu_msg:
                    imu_dict = self._imu_to_dict(contents)
                    if imu_dict is not None:
                        self.last_imu = imu_dict
                        data_received = True
                
            # Connection restored when we get any data (timestamp already updated above)
            if data_received and self._connection_lost:
                self._connection_lost = False
                self._reconnect_attempts = 0
                print("[RTK] Connection restored!")
        except serial.SerialException as e:
            print(f"[RTK] Serial error: {e}")
            self._connection_lost = True
        except (ConnectionError, ConnectionResetError, ConnectionAbortedError, BrokenPipeError) as e:
            if self._use_tcp:
                print(f"[RTK] TCP connection error: {e}")
                self._connection_lost = True
            else:
                raise
        except Exception as e:
            print(f"[RTK] Error: {e}")
            # Don't mark as connection lost for other exceptions (might be decoder errors)

    @staticmethod
    def _timestamp_to_float(t):
        """Convert Fusion Engine Timestamp (or similar) to float seconds for JSON."""
        if t is None:
            return None
        try:
            return float(t)
        except (TypeError, ValueError):
            pass
        try:
            s = getattr(t, "seconds", 0) or 0
            ns = getattr(t, "nanos", 0) or getattr(t, "nanoseconds", 0) or 0
            return float(s) + float(ns) / 1e9
        except (TypeError, ValueError):
            pass
        return None

    @staticmethod
    def _has_nan(value):
        """Check if a value or iterable contains NaN."""
        if value is None:
            return False
        try:
            if hasattr(value, "__iter__") and not isinstance(value, (str, bytes)):
                return any(x != x for x in value if x is not None)  # NaN check: x != x is True for NaN
            else:
                return value != value  # NaN check
        except (TypeError, ValueError):
            return False

    def _pose_to_dict(self, msg):
        """Convert PoseMessage (or ROS pose) to a simple dict."""
        out = {}
        # Try multiple possible attribute names for LLA
        lla = None
        for attr_name in ["lla_deg", "lla", "position_lla_deg", "position"]:
            if hasattr(msg, attr_name):
                lla = getattr(msg, attr_name)
                break
        
        if lla is not None:
            if hasattr(lla, "__iter__") and not isinstance(lla, (str, bytes)):
                out["lla_deg"] = list(lla)
            else:
                out["lla_deg"] = lla
        
        # Try multiple possible attribute names for solution_type
        solution_type = None
        for attr_name in ["solution_type", "solution", "fix_type", "gnss_fix_type"]:
            if hasattr(msg, attr_name):
                solution_type = getattr(msg, attr_name)
                break
        
        if solution_type is not None:
            out["solution_type"] = getattr(solution_type, "name", solution_type) if hasattr(solution_type, "name") else str(solution_type)
        
        # Try multiple possible attribute names for position_std_enu_m
        std = None
        for attr_name in ["position_std_enu_m", "position_std", "std_enu_m", "position_uncertainty"]:
            if hasattr(msg, attr_name):
                std = getattr(msg, attr_name)
                break
        
        if std is not None:
            if hasattr(std, "__iter__") and not isinstance(std, (str, bytes)):
                out["position_std_enu_m"] = list(std)
            else:
                out["position_std_enu_m"] = std
        
        # GPS time
        for attr_name in ["gps_time", "gps_timestamp", "time"]:
            if hasattr(msg, attr_name):
                out["gps_time"] = self._timestamp_to_float(getattr(msg, attr_name))
                break
        
        # P1 time
        for attr_name in ["p1_time", "p1_timestamp", "timestamp"]:
            if hasattr(msg, attr_name):
                out["p1_time"] = self._timestamp_to_float(getattr(msg, attr_name))
                break
        
        # YPR
        ypr = None
        for attr_name in ["ypr_deg", "ypr", "orientation_ypr", "attitude"]:
            if hasattr(msg, attr_name):
                ypr = getattr(msg, attr_name)
                break
        
        if ypr is not None:
            if hasattr(ypr, "__iter__") and not isinstance(ypr, (str, bytes)):
                out["ypr_deg"] = list(ypr)
            else:
                out["ypr_deg"] = ypr
        
        # Velocity
        vel = None
        for attr_name in ["velocity_body_mps", "velocity", "velocity_body", "vel_body"]:
            if hasattr(msg, attr_name):
                vel = getattr(msg, attr_name)
                break
        
        if vel is not None:
            if hasattr(vel, "__iter__") and not isinstance(vel, (str, bytes)):
                out["velocity_body_mps"] = list(vel)
            else:
                out["velocity_body_mps"] = vel
        
        return out

    def _imu_to_dict(self, msg):
        """Convert IMU message to dict with accel_xyz (m/s²) and gyro_xyz (rad/s)."""
        out = {}
        
        # Try multiple possible attribute names for acceleration
        accel = None
        for attr_name in ["acceleration_mps2", "accel_xyz", "accel_mps2", "acceleration", "accel", "linear_acceleration"]:
            if hasattr(msg, attr_name):
                accel = getattr(msg, attr_name)
                break
        
        if accel is not None:
            if hasattr(accel, "__iter__") and not isinstance(accel, (str, bytes)):
                out["accel_xyz"] = list(accel)
            else:
                out["accel_xyz"] = accel
        
        # Try multiple possible attribute names for angular velocity
        gyro = None
        for attr_name in ["angular_velocity_rps", "gyro_xyz", "gyro_rps", "angular_velocity", "gyro", "angular_velocity_radps"]:
            if hasattr(msg, attr_name):
                gyro = getattr(msg, attr_name)
                break
        
        if gyro is not None:
            if hasattr(gyro, "__iter__") and not isinstance(gyro, (str, bytes)):
                out["gyro_xyz"] = list(gyro)
            else:
                out["gyro_xyz"] = gyro
        
        # P1 time
        for attr_name in ["p1_time", "p1_timestamp", "timestamp", "time"]:
            if hasattr(msg, attr_name):
                p1_time = self._timestamp_to_float(getattr(msg, attr_name))
                if p1_time is not None:
                    out["p1_time"] = p1_time
                break
        
        return out if out else None

    def get_latest_pose(self):
        """
        Return latest pose as dict (lla_deg, solution_type, gps_time, etc.) or None.
        When current pose is Invalid/NaN, returns last_valid_pose with solution_stale=True
        (Point One: use last known good position during brief fix loss).
        """
        if self.last_pose is None and not hasattr(self, '_pose_none_logged'):
            print("[RTK] Warning: No pose data received yet (get_latest_pose returned None)")
            self._pose_none_logged = True
        if self.last_pose is None:
            return self.last_valid_pose
        solution_type = self.last_pose.get("solution_type", "")
        lla = self.last_pose.get("lla_deg")
        has_nan = lla is not None and hasattr(lla, "__iter__") and any(
            x != x for x in lla if x is not None
        )
        if (solution_type in ("Invalid", "INVALID") and has_nan) and self.last_valid_pose is not None:
            out = dict(self.last_valid_pose)
            out["solution_stale"] = True
            out["current_solution_type"] = solution_type
            return out
        return self.last_pose

    def get_latest_imu(self):
        """Return latest IMU as dict (accel_xyz, gyro_xyz) or None if no IMU message yet."""
        return self.last_imu

    def is_available(self):
        return self._initialized

    def get_polaris_config_info(self):
        """Return Polaris configuration info for reference."""
        from .config import POLARIS_API_KEY, POLARIS_NTRIP_HOST, POLARIS_NTRIP_MOUNT_POINT
        return {
            "api_key": POLARIS_API_KEY[:8] + "..." if POLARIS_API_KEY else None,  # Masqué pour sécurité
            "ntrip_host": POLARIS_NTRIP_HOST,
            "mount_point": POLARIS_NTRIP_MOUNT_POINT
        }

    def stop(self):
        self._close_connection()
        self._initialized = False
        self._decoder = None
