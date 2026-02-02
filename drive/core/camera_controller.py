"""
Camera controller for DepthAI camera capture (no AI processing).
"""

import depthai as dai
from .config import CAM_WIDTH, CAM_HEIGHT, CAM_FPS


class CameraController:
    """
    Controller for DepthAI camera - captures raw frames only.
    """
    
    def __init__(self, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS):
        """
        Initialize the camera controller.
        
        Args:
            width: Camera width in pixels.
            height: Camera height in pixels.
            fps: Frames per second.
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.device = None
        self.q_rgb = None
        self._initialized = False
    
    def initialize(self):
        """
        Initialize the DepthAI camera pipeline.
        
        Raises:
            RuntimeError: If camera initialization fails.
        """
        try:
            pipeline = dai.Pipeline()
            cam = pipeline.createColorCamera()
            cam.setPreviewSize(self.width, self.height)
            cam.setInterleaved(False)
            cam.setFps(self.fps)
            
            xout = pipeline.createXLinkOut()
            xout.setStreamName("rgb")
            cam.preview.link(xout.input)
            
            self.device = dai.Device(pipeline)
            self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
            self._initialized = True
            print(f"[Camera] Initialized: {self.width}x{self.height} @ {self.fps}fps")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize camera: {e}")
    
    def get_frame(self):
        """
        Get the latest frame from the camera.
        
        Returns:
            numpy.ndarray or None: OpenCV frame (BGR format) or None if no frame available.
        """
        if not self._initialized or self.q_rgb is None:
            return None
        
        frame_in = self.q_rgb.tryGet()
        if frame_in is None:
            return None
        
        return frame_in.getCvFrame()
    
    def is_available(self):
        """
        Check if camera is available and initialized.
        
        Returns:
            bool: True if camera is ready, False otherwise.
        """
        return self._initialized and self.device is not None
    
    def stop(self):
        """Stop the camera and clean up resources."""
        if self.device is not None:
            try:
                self.device.close()
            except Exception as e:
                print(f"[Camera] Error during stop: {e}")
            finally:
                self._initialized = False
                self.device = None
                self.q_rgb = None
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
