"""
Data publisher for collecting and broadcasting sensor data.
Uses H264 encoding (PyAV) or JPEG fallback (OpenCV) with chunked UDP packets.
"""

import time
import threading
from fractions import Fraction

import cv2

from .config import (
    PUBLISH_RATE,
    CAM_WIDTH,
    CAM_HEIGHT,
    CAM_FPS,
    H264_BITRATE,
)

try:
    import av
    HAS_AV = True
except ImportError:
    HAS_AV = False


class DataPublisher:
    """
    Publisher that collects data from CameraController,
    formats it according to the protocol, and publishes via SocketServer.
    """
    
    def __init__(self, lidar_controller, camera_controller, socket_server,
                 debug_camera=False, debug_lidar=False, publish_rate=None):
        """
        Initialize the data publisher.
        
        Args:
            lidar_controller: LidarController instance (can be None).
            camera_controller: CameraController instance (can be None).
            socket_server: SocketServer instance.
            debug_camera: If True, print camera data debug messages.
            debug_lidar: If True, print lidar data debug messages.
            publish_rate: FPS de publication (Hz). Si None, utilise PUBLISH_RATE du config.
        """
        self.lidar_controller = lidar_controller
        self.debug_camera = debug_camera
        self.debug_lidar = debug_lidar
        self.camera_controller = camera_controller
        self.socket_server = socket_server
        
        self.publish_thread = None
        self.running = False
        self.publish_rate = publish_rate if publish_rate is not None else PUBLISH_RATE
        self.sleep_time = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.1
        self._frame_number = 0
        self._h264_codec = None
    
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
        if not HAS_AV:
            print("[DataPublisher] PyAV non installé. Utilisation du fallback JPEG (OpenCV).")
        
        while self.running:
            try:
                # Publish camera data as H264 chunked UDP packets
                self._publish_camera_chunked()
                
                # Publish status periodically
                status = self._get_status()
                if status is not None:
                    self.socket_server.emit_status(status)
                
                # Sleep to control publish rate
                time.sleep(self.sleep_time)
            
            except Exception as e:
                error_msg = str(e)
                # Check for communication errors that might indicate device issues
                if "X_LINK_ERROR" in error_msg or "Communication" in error_msg or "rgb" in error_msg.lower():
                    print(f"[DataPublisher] Error in publish loop: Communication exception - possible device error/misconfiguration. Original message: '{error_msg}'")
                else:
                    print(f"[DataPublisher] Error in publish loop: {e}")
                time.sleep(self.sleep_time)
    
    def _ensure_h264_codec(self, width: int, height: int):
        """Create H264 codec context if not already created."""
        if self._h264_codec is not None:
            return
        if not HAS_AV:
            return
        try:
            self._h264_codec = av.CodecContext.create('h264', 'w')
            self._h264_codec.width = width
            self._h264_codec.height = height
            self._h264_codec.bit_rate = H264_BITRATE
            self._h264_codec.pix_fmt = 'yuv420p'
            self._h264_codec.time_base = Fraction(1, CAM_FPS)
            self._h264_codec.framerate = CAM_FPS
            self._h264_codec.options['g'] = '1'
        except Exception as e:
            if self.debug_camera:
                print(f"[DataPublisher] Failed to create H264 codec: {e}")
    
    def _publish_camera_chunked(self):
        """
        Get frame from camera, encode (H264 or JPEG), send as chunked UDP packets.
        """
        if self.camera_controller is None or not self.camera_controller.is_available():
            return
        
        frame = self.camera_controller.get_frame()
        if frame is None:
            return
        
        try:
            if HAS_AV:
                height, width = frame.shape[:2]
                self._ensure_h264_codec(width, height)
                if self._h264_codec is not None:
                    av_frame = av.VideoFrame.from_ndarray(frame, format='bgr24')
                    packet_bytes_list = []
                    for packet in self._h264_codec.encode(av_frame):
                        packet_bytes_list.append(bytes(packet))
                    if packet_bytes_list:
                        frame_bytes = b''.join(packet_bytes_list)
                        self.socket_server.send_raw_chunked(frame_bytes, self._frame_number)
                        self._frame_number += 1
            else:
                # Fallback JPEG (OpenCV) - no PyAV required
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if buffer is not None:
                    frame_bytes = buffer.tobytes()
                    self.socket_server.send_raw_chunked(frame_bytes, self._frame_number)
                    self._frame_number += 1
        except Exception as e:
            if self.debug_camera:
                print(f"[DataPublisher] Error encoding/sending camera frame: {e}")
    
    def _get_status(self):
        """
        Get current system status.
        
        Returns:
            dict: Status information.
        """
        timestamp = time.time()
        
        status = {
            "timestamp": timestamp,
            "camera_connected": False,
            "clients_connected": 0
        }
        
        # Check camera status
        if self.camera_controller is not None:
            status["camera_connected"] = self.camera_controller.is_available()
        
        # Client count not tracked anymore (UDP broadcast only)
        if self.socket_server is not None:
            status["clients_connected"] = 0
        
        return status
