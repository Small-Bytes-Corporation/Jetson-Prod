"""
Camera controller for DepthAI camera capture (no AI processing).
Supports DepthAI 2.x (ColorCamera + XLinkOut + Device) and 3.3+ (Camera node + pipeline.start).
"""

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
        self._pipeline = None  # v3: pipeline; v2: unused
        self.q_rgb = None
        self._initialized = False
        self._use_v3 = _is_depthai_v3(dai)
    
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
                device_info = None
                if hasattr(dai, "Device") and hasattr(dai.Device, "getAllAvailableDevices"):
                    available = dai.Device.getAllAvailableDevices()
                    if available:
                        device_info = available[0]
                try:
                    self.device = dai.Device(pipeline, device_info) if device_info else dai.Device(pipeline)
                except TypeError:
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
        frame_in = self.q_rgb.tryGet() if hasattr(self.q_rgb, "tryGet") else self.q_rgb.get()
        if frame_in is None:
            return None
        return frame_in.getCvFrame()
    
    def is_available(self):
        """
        Check if camera is available and initialized.
        
        Returns:
            bool: True if camera is ready, False otherwise.
        """
        return self._initialized and (self.device is not None or self._pipeline is not None)
    
    def stop(self):
        """Stop the camera and clean up resources."""
        try:
            if self._pipeline is not None and hasattr(self._pipeline, "stop"):
                self._pipeline.stop()
            elif self.device is not None and self.device is not True and hasattr(self.device, "close"):
                self.device.close()
        except Exception as e:
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
