"""
Camera controller for DepthAI camera capture (no AI processing).
Supports DepthAI 2.x (ColorCamera + XLinkOut + Device) and 3.3+ (Camera node + pipeline.start).
"""

import threading
import time
import depthai as dai
from .config import CAM_WIDTH, CAM_HEIGHT, CAM_FPS


def _is_depthai_v3(dai_module):
    """DepthAI 3.3+: no XLinkOut, use Camera node + output.createOutputQueue() + pipeline.start()."""
    if not hasattr(dai_module, "node"):
        return False
    has_camera = getattr(dai_module.node, "Camera", None) is not None
    has_xlinkout = getattr(dai_module.node, "XLinkOut", None) is not None
    return has_camera and not has_xlinkout


class CameraController:
    """
    Controller for DepthAI camera - captures raw frames only.
    """
    
    def __init__(self, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS, debug=False):
        """
        Initialize the camera controller.
        
        Args:
            width: Camera width in pixels.
            height: Camera height in pixels.
            fps: Frames per second.
            debug: If True, print debug messages.
        """
        self.width = width
        self.debug = debug
        self.height = height
        self.fps = fps
        self.device = None
        self._pipeline = None  # v3: pipeline; v2: unused
        self.q_rgb = None
        self._initialized = False
        self._use_v3 = _is_depthai_v3(dai)
        self._frame_lock = threading.Lock()  # Lock for thread-safe frame access
        self._can_read_frames = False  # Track if camera can actually read frames
    
    def initialize(self):
        """
        Initialize the DepthAI camera pipeline.
        Uses the first available device (same as --list-devices) when present.
        
        Raises:
            RuntimeError: If camera initialization fails.
        """
        try:
            pipeline = dai.Pipeline()
            if self._use_v3:
                # DepthAI 3.3+: Camera node, no XLinkOut, queue from output, pipeline.start()
                cam = pipeline.create(dai.node.Camera).build()
                rgb_type = getattr(getattr(dai, "ImgFrame", None), "Type", None)
                rgb_type = getattr(rgb_type, "RGB888p", None) if rgb_type else None
                try:
                    if rgb_type is not None:
                        camera_output = cam.requestOutput((self.width, self.height), type=rgb_type)
                    else:
                        camera_output = cam.requestOutput((self.width, self.height))
                except TypeError:
                    camera_output = cam.requestOutput(size=(self.width, self.height), type=rgb_type) if rgb_type else cam.requestOutput(size=(self.width, self.height))
                self.q_rgb = camera_output.createOutputQueue()
                pipeline.start()
                self._pipeline = pipeline
                self.device = True  # placeholder so is_available() works
            else:
                # DepthAI 2.x: ColorCamera + XLinkOut + Device
                if hasattr(pipeline, "create") and hasattr(dai, "node") and getattr(dai.node, "ColorCamera", None) is not None:
                    cam = pipeline.create(dai.node.ColorCamera)
                    xout = pipeline.create(dai.node.XLinkOut)
                else:
                    cam = pipeline.createColorCamera()
                    xout = pipeline.createXLinkOut()
                cam.setPreviewSize(self.width, self.height)
                cam.setInterleaved(False)
                cam.setFps(self.fps)
                xout.setStreamName("rgb")
                cam.preview.link(xout.input)
                # Always use first available device (simplified - no CAMERA_DEVICE_ID)
                device_info = None
                if hasattr(dai, "Device") and hasattr(dai.Device, "getAllAvailableDevices"):
                    available = dai.Device.getAllAvailableDevices()
                    if available:
                        # Use first available device
                        device_info = available[0]
                try:
                    self.device = dai.Device(pipeline, device_info) if device_info else dai.Device(pipeline)
                except TypeError:
                    self.device = dai.Device(pipeline)
                self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
            self._initialized = True
            if self.debug:
                print(f"[Camera] Initialized: {self.width}x{self.height} @ {self.fps}fps")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize camera: {e}")
    
    def get_frame(self):
        """
        Get the latest frame from the camera.
        Thread-safe: uses a lock to prevent concurrent access to the DepthAI queue.
        
        Returns:
            numpy.ndarray or None: OpenCV frame (BGR format) or None if no frame available.
        """
        if not self._initialized or self.q_rgb is None:
            return None
        
        # Use lock to ensure thread-safe access to DepthAI queue
        with self._frame_lock:
            try:
                # Try to get frame from queue (non-blocking if tryGet available)
                if hasattr(self.q_rgb, "tryGet"):
                    frame_in = self.q_rgb.tryGet()
                else:
                    # Fallback to blocking get() - may raise exceptions on Jetson
                    frame_in = self.q_rgb.get()
                
                if frame_in is None:
                    return None
                
                # Convert to OpenCV frame - may raise exceptions on communication errors
                frame = frame_in.getCvFrame()
                # If we successfully got a frame, mark that we can read frames
                if frame is not None:
                    self._can_read_frames = True
                return frame
            except (RuntimeError, Exception) as e:
                # Handle X_LINK_ERROR and other DepthAI communication errors
                # On Jetson, USB communication issues can cause exceptions
                # Return None to allow the system to continue functioning
                if self.debug:
                    error_msg = str(e)
                    # Check if it's an X_LINK_ERROR specifically
                    if "X_LINK_ERROR" in error_msg or "rgb" in error_msg.lower():
                        print(f"[Camera] Communication error reading frame: {e}")
                    else:
                        print(f"[Camera] Error reading frame: {e}")
                return None
    
    def is_available(self):
        """
        Check if camera is available and initialized.
        
        Returns:
            bool: True if camera is ready, False otherwise.
        """
        return self._initialized and (self.device is not None or self._pipeline is not None)
    
    def verify_ready(self, timeout=2.0, max_attempts=10):
        """
        Verify that the camera can actually produce frames.
        This helps ensure the camera pipeline is stable before starting DataPublisher.
        
        Args:
            timeout: Maximum time to wait for a frame (seconds).
            max_attempts: Maximum number of frame read attempts.
        
        Returns:
            bool: True if camera can produce frames, False otherwise.
        """
        if not self.is_available():
            return False
        
        # Try to read at least one frame to verify the pipeline is working
        start_time = time.time()
        attempts = 0
        
        while attempts < max_attempts and (time.time() - start_time) < timeout:
            frame = self.get_frame()
            if frame is not None:
                self._can_read_frames = True
                if self.debug:
                    print(f"[Camera] Verified ready: successfully read frame after {attempts + 1} attempt(s)")
                return True
            attempts += 1
            time.sleep(0.1)  # Small delay between attempts
        
        self._can_read_frames = False
        if self.debug:
            print(f"[Camera] Warning: Could not verify camera readiness after {attempts} attempts")
        return False
    
    def can_read_frames(self):
        """
        Check if camera can actually read frames (not just initialized).
        
        Returns:
            bool: True if camera has successfully read at least one frame, False otherwise.
        """
        return self._can_read_frames
    
    def stop(self):
        """Stop the camera and clean up resources."""
        try:
            if self._pipeline is not None and hasattr(self._pipeline, "stop"):
                self._pipeline.stop()
            elif self.device is not None and self.device is not True and hasattr(self.device, "close"):
                self.device.close()
        except Exception as e:
            if self.debug:
                print(f"[Camera] Error during stop: {e}")
        finally:
            self._initialized = False
            self.device = None
            self._pipeline = None
            self.q_rgb = None
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
