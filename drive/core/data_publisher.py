"""
Data publisher for collecting and broadcasting sensor data via socket.io.
"""

import time
import base64
import cv2
import threading
from .config import PUBLISH_RATE


class DataPublisher:
    """
    Publisher that collects data from LidarController, CameraController, and RTKController,
    formats it according to the protocol, and publishes via SocketServer.
    """
    
    def __init__(self, lidar_controller, camera_controller, socket_server, rtk_controller=None):
        """
        Initialize the data publisher.
        
        Args:
            lidar_controller: LidarController instance (can be None).
            camera_controller: CameraController instance (can be None).
            socket_server: SocketServer instance.
            rtk_controller: RTKController instance (can be None).
        """
        self.lidar_controller = lidar_controller
        self.camera_controller = camera_controller
        self.rtk_controller = rtk_controller
        self.socket_server = socket_server
        
        self.publish_thread = None
        self.running = False
        self.publish_rate = PUBLISH_RATE
        self.sleep_time = 1.0 / PUBLISH_RATE if PUBLISH_RATE > 0 else 0.1
    
    def start(self):
        """Start the data publishing loop in a separate thread."""
        if self.running:
            print("[DataPublisher] Already running")
            return
        
        self.running = True
        self.publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.publish_thread.start()
        print(f"[DataPublisher] Started publishing at {self.publish_rate} Hz")
    
    def stop(self):
        """Stop the data publishing loop."""
        if not self.running:
            return
        
        self.running = False
        if self.publish_thread is not None:
            self.publish_thread.join(timeout=2.0)
        print("[DataPublisher] Stopped")
    
    def _publish_loop(self):
        """Main publishing loop running in a separate thread."""
        while self.running:
            try:
                # Collect data from sensors
                data = self._collect_data()
                
                # Publish if we have data or status update
                if data is not None:
                    self.socket_server.emit_sensor_data(data)
                
                # Publish status periodically
                status = self._get_status()
                if status is not None:
                    self.socket_server.emit_status(status)
                
                # Sleep to control publish rate
                time.sleep(self.sleep_time)
            
            except Exception as e:
                print(f"[DataPublisher] Error in publish loop: {e}")
                time.sleep(self.sleep_time)
    
    def _collect_data(self):
        """
        Collect data from lidar, camera, and RTK controllers.
        
        Returns:
            dict: Formatted sensor data or None if no data available.
        """
        timestamp = time.time()
        data = {
            "timestamp": timestamp,
            "lidar": None,
            "camera": None,
            "rtk": None
        }
        
        # Collect lidar data
        if self.lidar_controller is not None and self.lidar_controller.is_available():
            lidar_scan = self.lidar_controller.get_scan()
            if lidar_scan is not None:
                data["lidar"] = {
                    "points": lidar_scan,
                    "scan_complete": True
                }
        
        # Collect camera data
        if self.camera_controller is not None and self.camera_controller.is_available():
            frame = self.camera_controller.get_frame()
            if frame is not None:
                # Encode frame as JPEG and convert to base64
                try:
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    frame_base64 = base64.b64encode(buffer).decode('utf-8')
                    
                    data["camera"] = {
                        "frame": frame_base64,
                        "width": frame.shape[1],
                        "height": frame.shape[0],
                        "format": "jpeg"
                    }
                except Exception as e:
                    print(f"[DataPublisher] Error encoding camera frame: {e}")
        
        # Collect RTK data (pose and IMU)
        if self.rtk_controller is not None and self.rtk_controller.is_available():
            pose = self.rtk_controller.get_latest_pose()
            imu = self.rtk_controller.get_latest_imu()
            data["rtk"] = {"pose": pose, "imu": imu}
            
            # Debug: log if pose is None but IMU is available (only once)
            if pose is None and imu is not None:
                if not hasattr(self, '_pose_none_logged'):
                    print("[DataPublisher] Warning: RTK IMU available but pose is None")
                    print("[DataPublisher] This may indicate that POSE messages are not being received")
                    self._pose_none_logged = True
        
        # Return data if we have at least lidar, camera, or RTK (with pose or imu)
        has_rtk = data["rtk"] is not None and (
            data["rtk"].get("pose") is not None or data["rtk"].get("imu") is not None
        )
        if data["lidar"] is not None or data["camera"] is not None or has_rtk:
            return data
        
        return None
    
    def _get_status(self):
        """
        Get current system status.
        
        Returns:
            dict: Status information.
        """
        timestamp = time.time()
        
        status = {
            "timestamp": timestamp,
            "lidar_connected": False,
            "camera_connected": False,
            "rtk_connected": False,
            "clients_connected": 0
        }
        
        # Check lidar status
        if self.lidar_controller is not None:
            status["lidar_connected"] = self.lidar_controller.is_available()
        
        # Check camera status
        if self.camera_controller is not None:
            status["camera_connected"] = self.camera_controller.is_available()
        
        # Check RTK status
        if self.rtk_controller is not None:
            status["rtk_connected"] = self.rtk_controller.is_available()
        
        # Get client count
        if self.socket_server is not None:
            status["clients_connected"] = self.socket_server.get_client_count()
        
        return status
