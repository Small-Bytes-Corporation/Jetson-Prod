#!/usr/bin/env python3
"""
Test DepthAI / OAK-D camera seul : pas de socket, affichage dans une fenêtre OpenCV.
Usage: python3 scripts/test_camera.py
       ou depuis la racine: python3 -m scripts.test_camera
Quitter: touche 'q' ou Ctrl+C.
"""

import sys
import argparse

try:
    import cv2
except ImportError:
    print("Install opencv-python: pip install opencv-python")
    sys.exit(1)

try:
    import depthai as dai
except ImportError:
    print("Install depthai: pip install depthai")
    sys.exit(1)


def _is_depthai_v3(dai):
    """DepthAI 3.3+: no XLinkOut, use Camera node + output.createOutputQueue() + pipeline.start()."""
    if not hasattr(dai, "node"):
        return False
    has_camera = getattr(dai.node, "Camera", None) is not None
    has_xlinkout = getattr(dai.node, "XLinkOut", None) is not None
    return has_camera and not has_xlinkout


def build_pipeline_and_queue(dai, width, height, fps):
    """
    Build pipeline and return (pipeline, q_rgb, use_v3).
    - v3: pipeline.start(), q from camera_output.createOutputQueue(), stop with pipeline.stop()
    - v2: device = dai.Device(pipeline), q from device.getOutputQueue("rgb"), stop with device.close()
    """
    pipeline = dai.Pipeline()
    use_v3 = _is_depthai_v3(dai)

    if use_v3:
        # DepthAI 3.3+: Camera node, no XLinkOut, queue from output
        cam = pipeline.create(dai.node.Camera).build()
        ImgFrame = getattr(dai, "ImgFrame", None)
        rgb_type = getattr(ImgFrame, "Type", None) and getattr(ImgFrame.Type, "RGB888p", None)
        if rgb_type is None:
            rgb_type = getattr(dai, "ImgFrame", None) and getattr(dai.ImgFrame.Type, "RGB888p", None)
        # requestOutput(size=(w,h), type=...) or requestOutput((w,h), type=...)
        try:
            if rgb_type is not None:
                camera_output = cam.requestOutput((width, height), type=rgb_type)
            else:
                camera_output = cam.requestOutput((width, height))
        except TypeError:
            try:
                camera_output = cam.requestOutput(size=(width, height), type=rgb_type) if rgb_type else cam.requestOutput(size=(width, height))
            except TypeError:
                camera_output = cam.requestOutput((width, height))
        q_rgb = camera_output.createOutputQueue()
        return pipeline, q_rgb, True
    else:
        # DepthAI 2.x: ColorCamera + XLinkOut + Device
        if hasattr(pipeline, "create") and hasattr(dai, "node") and getattr(dai.node, "ColorCamera", None) is not None:
            cam = pipeline.create(dai.node.ColorCamera)
            xout = pipeline.create(dai.node.XLinkOut)
        else:
            cam = pipeline.createColorCamera()
            xout = pipeline.createXLinkOut()
        cam.setPreviewSize(width, height)
        cam.setInterleaved(False)
        cam.setFps(fps)
        xout.setStreamName("rgb")
        cam.preview.link(xout.input)
        return pipeline, None, False


def main():
    parser = argparse.ArgumentParser(description="Test DepthAI camera avec affichage OpenCV")
    parser.add_argument("--width", type=int, default=640, help="Largeur preview (défaut 640)")
    parser.add_argument("--height", type=int, default=360, help="Hauteur preview (défaut 360)")
    parser.add_argument("--fps", type=int, default=30, help="FPS (défaut 30)")
    args = parser.parse_args()

    width = args.width
    height = args.height
    fps = args.fps

    print("[TestCamera] DepthAI version:", getattr(dai, "__version__", "?"))
    print(f"[TestCamera] Résolution: {width}x{height} @ {fps} fps")
    print("[TestCamera] Affichage: fenêtre OpenCV. Quitter avec 'q' ou Ctrl+C.")

    try:
        pipeline, q_rgb, use_v3 = build_pipeline_and_queue(dai, width, height, fps)
    except Exception as e:
        print(f"[TestCamera] Erreur construction pipeline: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    if use_v3:
        print("[TestCamera] API DepthAI v3 (Camera node, pipeline.start)")
        try:
            pipeline.start()
        except Exception as e:
            print(f"[TestCamera] Erreur pipeline.start(): {e}")
            sys.exit(1)
        device = None
    else:
        device_info = None
        if hasattr(dai, "Device") and hasattr(dai.Device, "getAllAvailableDevices"):
            available = dai.Device.getAllAvailableDevices()
            if available:
                device_info = available[0]
                print(f"[TestCamera] Appareil trouvé: {getattr(device_info, 'mxid', device_info)}")
        try:
            if device_info is not None:
                try:
                    device = dai.Device(pipeline, device_info)
                except TypeError:
                    device = dai.Device(pipeline)
            else:
                device = dai.Device(pipeline)
            q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        except Exception as e:
            print(f"[TestCamera] Erreur ouverture appareil: {e}")
            print("[TestCamera] Vérifiez câble USB, udev, et 'python3 main.py --list-devices'")
            sys.exit(1)
        pipeline = None

    print("[TestCamera] Caméra démarrée. Affichage...")
    window_name = "OAK-D Test (q pour quitter)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while True:
            frame_in = q_rgb.tryGet() if hasattr(q_rgb, "tryGet") else q_rgb.get()
            if frame_in is not None:
                frame = frame_in.getCvFrame()
                if frame is not None:
                    cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if device is not None:
            device.close()
        elif pipeline is not None and hasattr(pipeline, "stop"):
            pipeline.stop()
        print("[TestCamera] Arrêt.")


if __name__ == "__main__":
    main()
