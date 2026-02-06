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
        self.width = width
        self.debug = debug
        self.height = height
        self.fps = fps
        self.device = None
        self._pipeline = None
        self.q_rgb = None
        self._initialized = False
        self._use_v3 = _is_depthai_v3(dai)
        
        # Thread safety
        self._frame_lock = threading.Lock()
    
    def initialize(self):
        """
        Initialize the DepthAI camera pipeline.
        """
        try:
            pipeline = dai.Pipeline()
            
            if self._use_v3:
                # DepthAI 3.3+ logic
                cam = pipeline.create(dai.node.Camera).build()
                rgb_type = getattr(getattr(dai, "ImgFrame", None), "Type", None)
                rgb_type = getattr(rgb_type, "RGB888p", None) if rgb_type else None
                try:
                    if rgb_type is not None:
                        camera_output = cam.requestOutput((self.width, self.height), type=rgb_type)
                    else:
                        camera_output = cam.requestOutput((self.width, self.height))
                except TypeError:
                    camera_output = cam.requestOutput(size=(self.width, self.height))

                self.q_rgb = camera_output.createOutputQueue()
                pipeline.start()
                self._pipeline = pipeline
                self.device = True
            else:
                # DepthAI 2.x logic
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
                
                # Device selection
                device_info = None
                if hasattr(dai, "Device") and hasattr(dai.Device, "getAllAvailableDevices"):
                    available = dai.Device.getAllAvailableDevices()
                    if available:
                        device_info = available[0]
                
                try:
                    self.device = dai.Device(pipeline, device_info) if device_info else dai.Device(pipeline)
                except TypeError:
                    self.device = dai.Device(pipeline)
                
                # Output queue (NON-BLOCKING IS CRITICAL)
                self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
            
            self._initialized = True
            if self.debug:
                print(f"[Camera] Initialized: {self.width}x{self.height} @ {self.fps}fps")
                
            # Warmup sleep to prevent X_LINK_ERROR on immediate access
            time.sleep(1.0)
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize camera: {e}")
    
    def get_frame(self):
        """
        Get the latest frame from the camera. Safe against X_LINK_ERROR.
        """
        if not self._initialized or self.q_rgb is None:
            return None
        
        with self._frame_lock:
            try:
                # tryGet is non-blocking and safer
                if hasattr(self.q_rgb, "tryGet"):
                    frame_in = self.q_rgb.tryGet()
                else:
                    frame_in = self.q_rgb.get()
                
                if frame_in is None:
                    return None
                
                return frame_in.getCvFrame()
            
            except Exception as e:
                # Catch X_LINK_ERROR or RuntimeErrors without crashing the app
                err_msg = str(e)
                if self.debug and "X_LINK_ERROR" not in err_msg:
                    # Only print non-X_LINK errors to avoid spamming console
                    print(f"[Camera] Error getting frame: {e}")
                return None
    
    def is_available(self):
        return self._initialized and (self.device is not None or self._pipeline is not None)
    
    def stop(self):
        try:
            if self._pipeline is not None and hasattr(self._pipeline, "stop"):
                self._pipeline.stop()
            elif self.device is not None and self.device is not True and hasattr(self.device, "close"):
                self.device.close()
        except Exception:
            pass
        finally:
            self._initialized = False
            self.device = None
            self._pipeline = None
            self.q_rgb = None
