
`end-to-end/client_robocar.py`:
```py
import socket
import struct
import threading
import time
import argparse
import sys
import math
import logging
from fractions import Fraction

try:
    import depthai as dai
    import serial
    from pyvesc import VESC
    HARDWARE_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    HARDWARE_AVAILABLE = False

try:
    import cv2
    import av
    FAKE_VIDEO_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    FAKE_VIDEO_AVAILABLE = False


SERVER_IP = "127.0.0.1"
SERVER_PORT = 9999
CLIENT_PORT_RECEIVE = 10000
SERIAL_PORT = "/dev/ttyACM0"
VIDEO_WIDTH = 320
VIDEO_HEIGHT = 192
DEFAULT_FPS = 10
DEFAULT_BITRATE = 500000
COMMAND_TIMEOUT_S = 1.0

CHUNK_HEADER_FORMAT = "!IHHff"
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FORMAT)
MAX_PAYLOAD_SIZE = 1400

latest_commands = {'throttle': 0.0, 'steer': 0.0, 'timestamp': 0.0}
last_applied_commands = {'throttle': 0.0, 'steer': 0.0}
sent_frames = {}
state_lock = threading.Lock()

def remap_value(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    normalized_val = (value - in_min) / (in_max - in_min)
    return normalized_val * (out_max - out_min) + out_min

def clamp_value(value: float, clamp_min: float, clamp_max: float) -> float:
    return max(clamp_min, min(clamp_max, value))

def init_vesc(serial_port, attempts=5, delay=1.0):
    for i in range(attempts):
        try:
            motor = VESC(serial_port=serial_port)
            if motor.get_firmware_version() is not None:
                logging.info("VESC connected successfully (firmware OK).")
                return motor
            else:
                logging.warning(f"VESC responded with invalid data (try {i+1}/{attempts})")
        except Exception as e:
            logging.warning(f"Error connecting to VESC (try {i+1}/{attempts}): {e}")
        time.sleep(delay)
    raise RuntimeError("Failed to initialize VESC after multiple attempts.")

def apply_motor_commands(motor, throttle: float, steer: float):
    if motor is None:
        return 

    try:
        servo_value = remap_value(steer, -1.0, 1.0, 0.0, 1.0)
        servo_value = clamp_value(servo_value, 0.0, 1.0)
        throttle_value = clamp_value(throttle, -1.0, 1.0)
        
        motor.set_duty_cycle(throttle_value)
        motor.set_servo(servo_value)
    except serial.serialutil.SerialException as e:
        raise RuntimeError(f"VESC communication failed: {e}")

def receive_commands(sock: socket.socket):
    command_struct = struct.Struct('!Iff')
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) == command_struct.size:
                frame_num, throttle, steer = command_struct.unpack(data)
                with state_lock:
                    latest_commands['throttle'] = throttle
                    latest_commands['steer'] = steer
                    latest_commands['timestamp'] = time.time()
                    if frame_num in sent_frames:
                        del sent_frames[frame_num]
        except socket.timeout:
            continue
        except Exception as e:
            logging.error(f"Error in receive_commands thread: {e}")
            break

def get_h264_packets_from_file(video_path, args):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video file: {video_path}")

    codec = av.CodecContext.create('h264', 'w')
    codec.width = VIDEO_WIDTH
    codec.height = VIDEO_HEIGHT
    codec.bit_rate = args.bitrate
    codec.pix_fmt = 'yuv420p'
    codec.time_base = Fraction(1, args.fps)
    codec.framerate = args.fps
    codec.options['g'] = '1'

    logging.info(f"Streaming from {video_path} at {args.fps} FPS, looping.")
    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        frame_resized = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT))
        av_frame = av.VideoFrame.from_ndarray(frame_resized, format='bgr24')
        
        for packet in codec.encode(av_frame):
            yield bytes(packet)
        
        time.sleep(1.0 / args.fps)

def main(args):
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    if args.fake_video and not FAKE_VIDEO_AVAILABLE:
        logging.critical("Error: --fake-video requires 'opencv-python' and 'av'. Please run 'pip install opencv-python av'. Exiting.")
        sys.exit(1)
    if not args.fake_video and not HARDWARE_AVAILABLE:
        logging.critical("Error: Hardware mode requires 'depthai', 'pyserial', 'pyvesc' and its dependencies (e.g., 'pycrc'). Please install them. Exiting.")
        sys.exit(1)

    packet_generator = None
    motor = None
    device = None
    send_socket = None
    receive_socket = None

    try:
        if args.fake_video:
            logging.info(f"Using fake video source: {args.fake_video}")
            packet_generator = get_h264_packets_from_file(args.fake_video, args)
        else:
            logging.info("Using real OAK-D camera and VESC.")
            pipeline = dai.Pipeline()
            cam = pipeline.create(dai.node.ColorCamera)
            cam.setBoardSocket(dai.CameraBoardSocket.RGB)
            cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam.setInterleaved(False)
            cam.setFps(args.fps)
            manip = pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setResize(VIDEO_WIDTH, VIDEO_HEIGHT)
            manip.setWaitForConfigInput(False)
            cam.video.link(manip.inputImage)
            encoder = pipeline.create(dai.node.VideoEncoder)
            encoder.setDefaultProfilePreset(args.fps, dai.VideoEncoderProperties.Profile.H264_MAIN)
            encoder.setBitrate(args.bitrate)
            encoder.setKeyframeFrequency(1)
            manip.out.link(encoder.input)
            xout = pipeline.create(dai.node.XLinkOut)
            xout.setStreamName("h264")
            encoder.bitstream.link(xout.input)
            
            device = dai.Device(pipeline)
            logging.info("OAK-D camera initialized and pipeline started.")
            motor = init_vesc(args.serial_port)
            h264_queue = device.getOutputQueue(name="h264", maxSize=30, blocking=True)
            packet_generator = (p.getData().tobytes() for p in iter(h264_queue.get, None))

        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = (args.host, args.port)
        receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receive_socket.bind(('', CLIENT_PORT_RECEIVE))
        receive_socket.settimeout(1.0)
        
        receiver_thread = threading.Thread(target=receive_commands, args=(receive_socket,), daemon=True)
        receiver_thread.start()
        
        logging.info(f"Streaming video to {server_address[0]}:{server_address[1]}")
        logging.info(f"Listening for commands on UDP port {CLIENT_PORT_RECEIVE}")

        frame_number = 0
        chunk_header_struct = struct.Struct(CHUNK_HEADER_FORMAT)

        for frame_data in packet_generator:
            with state_lock:
                state_throttle = last_applied_commands['throttle']
                state_steer = last_applied_commands['steer']
            
            num_chunks = math.ceil(len(frame_data) / MAX_PAYLOAD_SIZE) if len(frame_data) > 0 else 1
            for i in range(num_chunks):
                start = i * MAX_PAYLOAD_SIZE
                end = start + MAX_PAYLOAD_SIZE
                payload = frame_data[start:end]
                header = chunk_header_struct.pack(frame_number, num_chunks, i, state_throttle, state_steer)
                send_socket.sendto(header + payload, server_address)

            with state_lock:
                now = time.time()
                if (now - latest_commands['timestamp']) > COMMAND_TIMEOUT_S:
                    if frame_number > args.fps:
                        logging.warning(f"Command timeout. Applying safe state.")
                    raw_throttle, raw_steer = 0.0, 0.0
                else:
                    raw_throttle = latest_commands['throttle']
                    raw_steer = latest_commands['steer']
                
                proc_throttle = clamp_value(raw_throttle, args.min_throttle, args.max_throttle)
                proc_steer = clamp_value(raw_steer, args.min_steer, args.max_steer)

                apply_motor_commands(motor, proc_throttle, proc_steer)
                last_applied_commands['throttle'] = proc_throttle
                last_applied_commands['steer'] = proc_steer
                
                sent_frames[frame_number] = time.time()
                timed_out_frames = [fn for fn, t in sent_frames.items() if (now - t) > COMMAND_TIMEOUT_S]
                if timed_out_frames:
                    for fn in timed_out_frames:
                        if fn in sent_frames: del sent_frames[fn]
            
            status_line = (f"Frame: {frame_number} | Sent State: T={state_throttle:.2f}, S={state_steer:.2f} | Rcvd Cmd: T={raw_throttle:.2f}, S={raw_steer:.2f} -> Applied: T={proc_throttle:.2f}, S={proc_steer:.2f} | Pending: {len(sent_frames)}")
            print(status_line, end='\r')
            frame_number += 1

    except RuntimeError as e:
        logging.critical(f"A critical runtime error occurred: {e}. Exiting.")
    except KeyboardInterrupt:
        logging.info("Shutdown requested by user.")
    finally:
        logging.info("Cleaning up resources...")
        if motor:
            try:
                motor.set_duty_cycle(0)
                motor.set_servo(0.5)
                motor.stop_heartbeat()
                logging.info("VESC communication stopped.")
            except Exception as e:
                logging.warning(f"Could not stop VESC cleanly: {e}")
        if device: device.close()
        if send_socket: send_socket.close()
        if receive_socket: receive_socket.close()
        logging.info("Cleanup complete. Exiting.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Robocar client for streaming video and state to an AI server.", formatter_class=argparse.RawTextHelpFormatter)
    group_mode = parser.add_argument_group('Operating Mode')
    group_mode.add_argument("--fake-video", type=str, default=None, metavar="PATH", help="Use a video file instead of the camera. Disables hardware.")
    
    group_network = parser.add_argument_group('Network and Hardware')
    group_network.add_argument("--host", type=str, default=SERVER_IP, help="IP address of the AI server to connect to.")
    group_network.add_argument("--port", type=int, default=SERVER_PORT, help="Port of the AI server to connect to.")
    group_network.add_argument("--serial-port", type=str, default=SERIAL_PORT, help="Serial port for the VESC (hardware mode only).")
    group_network.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Camera/encoder frames per second.")
    group_network.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE, help="H.264 encoder bitrate in bps.")
    
    group_control = parser.add_argument_group('Control Tuning (Hardware Mode)')
    group_control.add_argument('--min-throttle', type=float, default=-1.0, help='Hard minimum limit for throttle.')
    group_control.add_argument('--max-throttle', type=float, default=1.0, help='Hard maximum limit for throttle.')
    group_control.add_argument('--min-steer', type=float, default=-1.0, help='Hard minimum limit for steering.')
    group_control.add_argument('--max-steer', type=float, default=1.0, help='Hard maximum limit for steering.')

    parsed_args = parser.parse_args()
    main(parsed_args)
```

`end-to-end/client_robocar_dual.py`:
```py
import socket
import struct
import threading
import time
import argparse
import sys
import math
import logging
from fractions import Fraction

try:
    import depthai as dai
    import serial
    from pyvesc import VESC
    HARDWARE_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    HARDWARE_AVAILABLE = False

try:
    import cv2
    import av
    FAKE_VIDEO_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    FAKE_VIDEO_AVAILABLE = False


SERVER_IP = "127.0.0.1"
SERVER_PORT = 9999
CLIENT_PORT_RECEIVE = 10000
SERIAL_PORT = "/dev/ttyACM0"
VIDEO_WIDTH = 320
VIDEO_HEIGHT = 192
DEFAULT_FPS = 10
DEFAULT_BITRATE = 500000
COMMAND_TIMEOUT_S = 1.0

CHUNK_HEADER_FORMAT = "!IHH"
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FORMAT)
MAX_PAYLOAD_SIZE = 1400

latest_commands = {'throttle': 0.0, 'steer': 0.0, 'timestamp': 0.0}
sent_frames = {}
state_lock = threading.Lock()

def remap_value(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    normalized_val = (value - in_min) / (in_max - in_min)
    return normalized_val * (out_max - out_min) + out_min

def clamp_value(value: float, clamp_min: float, clamp_max: float) -> float:
    return max(clamp_min, min(clamp_max, value))

def init_vesc(serial_port, attempts=5, delay=1.0):
    for i in range(attempts):
        try:
            motor = VESC(serial_port=serial_port)
            if motor.get_firmware_version() is not None:
                logging.info("VESC connected successfully (firmware OK).")
                return motor
            else:
                logging.warning(f"VESC responded with invalid data (try {i+1}/{attempts})")
        except Exception as e:
            logging.warning(f"Error connecting to VESC (try {i+1}/{attempts}): {e}")
        time.sleep(delay)
    raise RuntimeError("Failed to initialize VESC after multiple attempts.")

def apply_motor_commands(motor, throttle: float, steer: float):
    if motor is None:
        return 

    try:
        servo_value = remap_value(steer, -1.0, 1.0, 0.0, 1.0)
        servo_value = clamp_value(servo_value, 0.0, 1.0)
        throttle_value = clamp_value(throttle, -1.0, 1.0)
        
        motor.set_duty_cycle(throttle_value)
        motor.set_servo(servo_value)
    except serial.serialutil.SerialException as e:
        raise RuntimeError(f"VESC communication failed: {e}")

def receive_commands(sock: socket.socket):
    command_struct = struct.Struct('!Iff')
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) == command_struct.size:
                frame_num, throttle, steer = command_struct.unpack(data)
                with state_lock:
                    latest_commands['throttle'] = throttle
                    latest_commands['steer'] = steer
                    latest_commands['timestamp'] = time.time()
                    if frame_num in sent_frames:
                        del sent_frames[frame_num]
        except socket.timeout:
            continue
        except Exception as e:
            logging.error(f"Error in receive_commands thread: {e}")
            break

def get_h264_packets_from_file(video_path, args):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video file: {video_path}")

    codec = av.CodecContext.create('h264', 'w')
    codec.width = VIDEO_WIDTH
    codec.height = VIDEO_HEIGHT
    codec.bit_rate = args.bitrate
    codec.pix_fmt = 'yuv420p'
    codec.time_base = Fraction(1, args.fps)
    codec.framerate = args.fps
    codec.options['g'] = '1'

    logging.info(f"Streaming from {video_path} at {args.fps} FPS, looping.")
    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        frame_resized = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT))
        av_frame = av.VideoFrame.from_ndarray(frame_resized, format='bgr24')
        
        for packet in codec.encode(av_frame):
            yield bytes(packet)
        
        time.sleep(1.0 / args.fps)

def main(args):
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    if args.fake_video and not FAKE_VIDEO_AVAILABLE:
        logging.critical("Error: --fake-video requires 'opencv-python' and 'av'. Please run 'pip install opencv-python av'. Exiting.")
        sys.exit(1)
    if not args.fake_video and not HARDWARE_AVAILABLE:
        logging.critical("Error: Hardware mode requires 'depthai', 'pyserial', 'pyvesc' and its dependencies. Please install them. Exiting.")
        sys.exit(1)

    packet_generator = None
    motor = None
    device = None
    send_socket = None
    receive_socket = None

    try:
        if args.fake_video:
            logging.info(f"Using fake video source: {args.fake_video}")
            packet_generator = get_h264_packets_from_file(args.fake_video, args)
        else:
            logging.info("Using real OAK-D camera and VESC.")
            pipeline = dai.Pipeline()
            cam = pipeline.create(dai.node.ColorCamera)
            cam.setBoardSocket(dai.CameraBoardSocket.RGB)
            cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam.setInterleaved(False)
            cam.setFps(args.fps)
            manip = pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setResize(VIDEO_WIDTH, VIDEO_HEIGHT)
            manip.setWaitForConfigInput(False)
            cam.video.link(manip.inputImage)
            encoder = pipeline.create(dai.node.VideoEncoder)
            encoder.setDefaultProfilePreset(args.fps, dai.VideoEncoderProperties.Profile.H264_MAIN)
            encoder.setBitrate(args.bitrate)
            encoder.setKeyframeFrequency(1)
            manip.out.link(encoder.input)
            xout = pipeline.create(dai.node.XLinkOut)
            xout.setStreamName("h264")
            encoder.bitstream.link(xout.input)
            
            device = dai.Device(pipeline)
            logging.info("OAK-D camera initialized and pipeline started.")
            motor = init_vesc(args.serial_port)
            h264_queue = device.getOutputQueue(name="h264", maxSize=30, blocking=True)
            packet_generator = (p.getData().tobytes() for p in iter(h264_queue.get, None))

        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = (args.host, args.port)
        receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receive_socket.bind(('', CLIENT_PORT_RECEIVE))
        receive_socket.settimeout(1.0)
        
        receiver_thread = threading.Thread(target=receive_commands, args=(receive_socket,), daemon=True)
        receiver_thread.start()
        
        logging.info(f"Streaming video to {server_address[0]}:{server_address[1]}")
        logging.info(f"Listening for commands on UDP port {CLIENT_PORT_RECEIVE}")

        frame_number = 0
        chunk_header_struct = struct.Struct(CHUNK_HEADER_FORMAT)

        for frame_data in packet_generator:
            num_chunks = math.ceil(len(frame_data) / MAX_PAYLOAD_SIZE) if len(frame_data) > 0 else 1
            for i in range(num_chunks):
                start = i * MAX_PAYLOAD_SIZE
                end = start + MAX_PAYLOAD_SIZE
                payload = frame_data[start:end]
                header = chunk_header_struct.pack(frame_number, num_chunks, i)
                send_socket.sendto(header + payload, server_address)

            with state_lock:
                now = time.time()
                if (now - latest_commands['timestamp']) > COMMAND_TIMEOUT_S:
                    if frame_number > args.fps:
                        logging.warning(f"Command timeout. Applying safe state.")
                    raw_throttle, raw_steer = 0.0, 0.0
                else:
                    raw_throttle = latest_commands['throttle']
                    raw_steer = latest_commands['steer']
                
                proc_throttle = clamp_value(raw_throttle, args.min_throttle, args.max_throttle)
                proc_steer = clamp_value(raw_steer, args.min_steer, args.max_steer)

                apply_motor_commands(motor, proc_throttle, proc_steer)
                
                sent_frames[frame_number] = time.time()
                timed_out_frames = [fn for fn, t in sent_frames.items() if (now - t) > COMMAND_TIMEOUT_S]
                if timed_out_frames:
                    for fn in timed_out_frames:
                        if fn in sent_frames: del sent_frames[fn]
            
            status_line = (f"Frame: {frame_number} | Rcvd Cmd: T={raw_throttle:.2f}, S={raw_steer:.2f} -> Applied: T={proc_throttle:.2f}, S={proc_steer:.2f} | Pending: {len(sent_frames)}")
            print(status_line, end='\r')
            frame_number += 1

    except RuntimeError as e:
        logging.critical(f"A critical runtime error occurred: {e}. Exiting.")
    except KeyboardInterrupt:
        logging.info("Shutdown requested by user.")
    finally:
        logging.info("Cleaning up resources...")
        if motor:
            try:
                motor.set_duty_cycle(0)
                motor.set_servo(0.5)
                motor.stop_heartbeat()
                logging.info("VESC communication stopped.")
            except Exception as e:
                logging.warning(f"Could not stop VESC cleanly: {e}")
        if device: device.close()
        if send_socket: send_socket.close()
        if receive_socket: receive_socket.close()
        logging.info("Cleanup complete. Exiting.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Robocar client for streaming video to an AI server.", formatter_class=argparse.RawTextHelpFormatter)
    group_mode = parser.add_argument_group('Operating Mode')
    group_mode.add_argument("--fake-video", type=str, default=None, metavar="PATH", help="Use a video file instead of the camera. Disables hardware.")
    
    group_network = parser.add_argument_group('Network and Hardware')
    group_network.add_argument("--host", type=str, default=SERVER_IP, help="IP address of the AI server to connect to.")
    group_network.add_argument("--port", type=int, default=SERVER_PORT, help="Port of the AI server to connect to.")
    group_network.add_argument("--serial-port", type=str, default=SERIAL_PORT, help="Serial port for the VESC (hardware mode only).")
    group_network.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Camera/encoder frames per second.")
    group_network.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE, help="H.264 encoder bitrate in bps.")
    
    group_control = parser.add_argument_group('Control Tuning (Hardware Mode)')
    group_control.add_argument('--min-throttle', type=float, default=-1.0, help='Hard minimum limit for throttle.')
    group_control.add_argument('--max-throttle', type=float, default=1.0, help='Hard maximum limit for throttle.')
    group_control.add_argument('--min-steer', type=float, default=-1.0, help='Hard minimum limit for steering.')
    group_control.add_argument('--max-steer', type=float, default=1.0, help='Hard maximum limit for steering.')

    parsed_args = parser.parse_args()
    main(parsed_args)
```

`mask-workbench/real/client_robocar.py`:
```py
import socket
import struct
import threading
import time
import argparse
import sys
import depthai as dai
from pyvesc import VESC

SERVER_IP = "127.0.0.1"
SERVER_PORT = 9999
CLIENT_PORT_RECEIVE = 10000
SERIAL_PORT = "/dev/ttyACM0"
VIDEO_WIDTH = 320
VIDEO_HEIGHT = 180
VIDEO_FPS = 10
COMMAND_TIMEOUT_S = 1.0

latest_commands = {'throttle': 0.0, 'steer': 0.0}
sent_frames = {}
state_lock = threading.Lock()

def init_vesc(serial_port, attempts=5, delay=1.0):
    for i in range(attempts):
        try:
            motor = VESC(serial_port=serial_port)
            if motor.get_firmware_version() is not None:
                print("VESC connected successfully (firmware OK).")
                return motor
            else:
                print(f"VESC responded with invalid data (try {i+1}/{attempts})")
        except Exception as e:
            print(f"Error connecting to VESC (try {i+1}/{attempts}): {e}")
        time.sleep(delay)
    raise RuntimeError("Failed to initialize VESC after multiple attempts.")

def apply_motor_commands(motor: VESC, throttle: float, steer: float):
    servo_value = (steer + 1.0) / 2.0
    servo_value = max(0.0, min(1.0, servo_value))
    throttle_value = max(-1.0, min(1.0, throttle))
    
    motor.set_duty_cycle(throttle_value)
    motor.set_servo(servo_value)

def receive_commands(sock: socket.socket):
    command_struct = struct.Struct('!Iff')

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) == command_struct.size:
                frame_num, throttle, steer = command_struct.unpack(data)

                with state_lock:
                    latest_commands['throttle'] = throttle
                    latest_commands['steer'] = steer
                    if frame_num in sent_frames:
                        del sent_frames[frame_num]
            else:
                print(f"Warning: Received malformed packet of size {len(data)}")

        except socket.timeout:
            continue
        except Exception as e:
            print(f"Error in command receiver thread: {e}")
            break

def main(args):
    motor = None
    try:
        motor = init_vesc(args.serial_port)
    except RuntimeError as e:
        print(f"Fatal: {e}")
        sys.exit(1)

    send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (args.server_ip, args.server_port)

    receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receive_socket.bind(('', CLIENT_PORT_RECEIVE))
    receive_socket.settimeout(1.0)

    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setPreviewSize(VIDEO_WIDTH, VIDEO_HEIGHT)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    cam.setFps(VIDEO_FPS)

    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(VIDEO_FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
    cam.video.link(encoder.input)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("h264")
    encoder.bitstream.link(xout.input)

    frame_number = 0
    header_struct = struct.Struct('!I')

    try:
        with dai.Device(pipeline) as device:
            print("OAK-D camera initialized and pipeline started.")
            h264_queue = device.getOutputQueue(name="h264", maxSize=30, blocking=True)

            receiver_thread = threading.Thread(target=receive_commands, args=(receive_socket,), daemon=True)
            receiver_thread.start()
            print(f"Streaming to {server_address[0]}:{server_address[1]}")
            print(f"Listening for commands on UDP port {CLIENT_PORT_RECEIVE}")
            print("Starting video stream. Press Ctrl+C to stop.")

            while True:
                h264_packet = h264_queue.get()
                data = h264_packet.getData()

                header = header_struct.pack(frame_number)
                packet = header + data
                send_socket.sendto(packet, server_address)

                with state_lock:
                    sent_frames[frame_number] = time.time()
                    
                    apply_motor_commands(motor, latest_commands['throttle'], latest_commands['steer'])

                    now = time.time()
                    timed_out_frames = []
                    for fn, send_time in sent_frames.items():
                        if fn > 0 and (now - send_time) > COMMAND_TIMEOUT_S:
                            timed_out_frames.append(fn)

                    if timed_out_frames:
                        print(f"\nTimeout: No response for frames {timed_out_frames}. Applying safe state.")
                        latest_commands['throttle'] = 0.0
                        latest_commands['steer'] = 0.0
                        apply_motor_commands(motor, latest_commands['throttle'], latest_commands['steer'])
                        for fn in timed_out_frames:
                            del sent_frames[fn]

                status_line = (
                    f"Frame: {frame_number} | "
                    f"Throttle: {latest_commands['throttle']:.2f}, Steer: {latest_commands['steer']:.2f} | "
                    f"Pending Responses: {len(sent_frames)}"
                )
                print(status_line, end='\r')

                frame_number += 1

    except KeyboardInterrupt:
        print("\nShutdown requested by user.")
    except Exception as e:
        print(f"\nA critical error occurred: {e}")
    finally:
        print("\nStopping motors and cleaning up...")
        if motor:
            motor.set_duty_cycle(0)
            motor.stop_heartbeat()
            print("VESC communication stopped.")
        send_socket.close()
        receive_socket.close()
        print("Cleanup complete. Exiting.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Robocar client for streaming video to an AI server.")
    parser.add_argument("--server-ip", type=str, default=SERVER_IP, help="IP address of the AI server.")
    parser.add_argument("--server-port", type=int, default=SERVER_PORT, help="Port of the AI server.")
    parser.add_argument("--serial-port", type=str, default=SERIAL_PORT, help="Serial port for the VESC motor controller.")
    parsed_args = parser.parse_args()
    main(parsed_args)
```

`mask-workbench/real/client_videotester.py`:
```py
#!/usr/bin/env python
import argparse
import socket
import struct
import sys
import threading
import time
from pathlib import Path

try:
    import av
    AV_AVAILABLE = True
except ImportError:
    AV_AVAILABLE = False

try:
    import ffmpeg
    FFMPEG_AVAILABLE = True
except ImportError:
    FFMPEG_AVAILABLE = False

import cv2
from PIL import Image, ImageOps
from tqdm import tqdm

TARGET_W, TARGET_H = 320, 180
DEFAULT_FPS = 10
DEFAULT_SERVER_IP = "127.0.0.1"
DEFAULT_SERVER_PORT = 9999

def get_video_rotation(video_path: Path) -> int:
    if not FFMPEG_AVAILABLE:
        print("Warning: `ffmpeg-python` not found. Cannot auto-detect rotation.")
        print("Install with: pip install ffmpeg-python")
        return 0
    try:
        probe = ffmpeg.probe(str(video_path))
        video_stream = next((s for s in probe['streams'] if s['codec_type'] == 'video'), None)
        if video_stream and 'tags' in video_stream and 'rotate' in video_stream['tags']:
            rotation = int(video_stream['tags']['rotate'])
            print(f"Auto-detected {rotation}-degree rotation.")
            return rotation
    except Exception as e:
        print(f"Warning: Could not get rotation with ffmpeg: {e}")
    print("No rotation metadata found or rotation is 0.")
    return 0

def zoom_to_fit(pil_img: Image.Image, target_w: int, target_h: int) -> Image.Image:
    w_in, h_in = pil_img.size
    ar_in = w_in / h_in
    ar_out = target_w / target_h

    if ar_in > ar_out:
        w_crop = h_in * ar_out
        left = (w_in - w_crop) / 2
        top = 0
        right = (w_in + w_crop) / 2
        bottom = h_in
    else:
        h_crop = w_in / ar_out
        left = 0
        top = (h_in - h_crop) / 2
        right = w_in
        bottom = (h_in + h_crop) / 2

    return pil_img.crop((left, top, right, bottom)).resize((target_w, target_h), Image.Resampling.LANCZOS)

def listen_for_commands(sock: socket.socket, stop_event: threading.Event):
    print("[Listener] Started. Waiting for commands from server...")
    packet_format = struct.Struct('>Iff')

    while not stop_event.is_set():
        try:
            data, server_addr = sock.recvfrom(1024)
            if data and len(data) == packet_format.size:
                frame_num, throttle, steer = packet_format.unpack(data)
                print(f"<-- Recv Frame {frame_num}: Throttle={throttle:.3f}, Steer={steer:.3f}")
            elif data:
                print(f"[Listener] Warning: Received malformed packet of size {len(data)}.")
        except socket.timeout:
            continue
        except Exception as e:
            if not stop_event.is_set():
                print(f"[Listener] Error: {e}")
            break
    print("[Listener] Stopped.")

def main():
    parser = argparse.ArgumentParser(
        description="Video test client for AI driving server. Reads a video file, streams it via UDP, and prints server responses.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("input_video", type=Path, help="Path to the input video file.")
    parser.add_argument("--ip", type=str, default=DEFAULT_SERVER_IP, help="Server IP address.")
    parser.add_argument("--port", type=int, default=DEFAULT_SERVER_PORT, help="Server port.")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Frames per second to stream.")
    args = parser.parse_args()

    if not AV_AVAILABLE:
        sys.exit("Error: PyAV is not installed. Please run: pip install av")
    if not args.input_video.is_file():
        sys.exit(f"Error: Input video not found at '{args.input_video}'")

    server_address = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    stop_event = threading.Event()
    listener = threading.Thread(target=listen_for_commands, args=(sock, stop_event), daemon=True)
    listener.start()

    print(f"Streaming '{args.input_video.name}' to {args.ip}:{args.port} at {args.fps} FPS...")

    cap = cv2.VideoCapture(str(args.input_video))
    if not cap.isOpened():
        sys.exit(f"Error: Could not open video file '{args.input_video}'")

    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    rotation_angle = get_video_rotation(args.input_video)
    rotation_map = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    rotation_code = rotation_map.get(rotation_angle)

    codec = av.CodecContext.create('libx264', 'w')
    codec.width = TARGET_W
    codec.height = TARGET_H
    codec.time_base = av.Rational(1, args.fps)
    codec.framerate = args.fps
    codec.pix_fmt = 'yuv420p'
    codec.options = {'preset': 'ultrafast', 'tune': 'zerolatency'}

    frame_num = 0
    frame_duration = 1.0 / args.fps

    try:
        with tqdm(total=frame_count, desc="Streaming Frames") as pbar:
            while cap.isOpened():
                loop_start_time = time.monotonic()
                ret, frame = cap.read()
                if not ret:
                    break

                if rotation_code is not None:
                    frame = cv2.rotate(frame, rotation_code)

                pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                processed_pil = zoom_to_fit(pil_img, TARGET_W, TARGET_H)

                av_frame = av.VideoFrame.from_image(processed_pil)
                for packet in codec.encode(av_frame):
                    header = struct.pack('>I', frame_num)
                    sock.sendto(header + bytes(packet), server_address)

                pbar.update(1)
                frame_num += 1

                elapsed_time = time.monotonic() - loop_start_time
                sleep_time = frame_duration - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        print("\nFlushing encoder...")
        for packet in codec.encode(None):
            header = struct.pack('>I', frame_num)
            sock.sendto(header + bytes(packet), server_address)
            print(f"--> Sent final flushed packet for frame {frame_num}")

    except KeyboardInterrupt:
        print("\nStreaming interrupted by user.")
    except Exception as e:
        print(f"\nAn error occurred during streaming: {e}")
    finally:
        print("Cleaning up resources...")
        stop_event.set()
        cap.release()
        listener.join(timeout=2.0)
        sock.close()
        print("Client shut down gracefully.")

if __name__ == "__main__":
    main()
```

`end-to-end/server.py`:
```py
import socket
import struct
import threading
import time
import argparse
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms.v2 as transforms
from PIL import Image
import numpy as np
import logging
from collections import deque

try:
    import av
    PYAV_AVAILABLE = True
except ImportError:
    PYAV_AVAILABLE = False

CLIENT_PORT_RECEIVE = 10000
CHUNK_HEADER_FORMAT = "!IHHff"
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FORMAT)
MAX_PACKET_SIZE = 2048
CLIENT_TIMEOUT_S = 10.0
HISTORY_LENGTH = 5
MODEL_INPUT_WIDTH = 320
MODEL_INPUT_HEIGHT = 180

class DirectPredictor(nn.Module):
    def __init__(self, history_length, cnn_out=256, state_features=2, rnn_hidden=256, drop_prob=0.3):
        super().__init__()
        self.cnn_backbone = models.mobilenet_v3_small(weights=None)
        num_cnn_features = self.cnn_backbone.classifier[0].in_features
        self.cnn_backbone.classifier = nn.Identity()
        self.rnn = nn.GRU(input_size=num_cnn_features, hidden_size=rnn_hidden, num_layers=2, batch_first=True, dropout=drop_prob)
        self.head = nn.Sequential(
            nn.Linear(rnn_hidden + state_features, 128), nn.ReLU(), nn.Dropout(p=drop_prob),
            nn.Linear(128, 64), nn.ReLU(), nn.Linear(64, 2)
        )
    def forward(self, image_seq, car_state):
        batch_size, seq_len = image_seq.shape[0], image_seq.shape[1]
        img_reshaped = image_seq.view(batch_size * seq_len, *image_seq.shape[2:])
        cnn_features = self.cnn_backbone(img_reshaped)
        cnn_features = cnn_features.view(batch_size, seq_len, -1)
        rnn_out, _ = self.rnn(cnn_features)
        motion_summary = rnn_out[:, -1, :]
        fused_head_input = torch.cat([motion_summary, car_state], dim=1)
        return self.head(fused_head_input)

client_data = {}
state_lock = threading.Lock()
model = None
image_transform = None
device = None
args = None
sock = None

def process_frame_and_predict(full_frame_data: bytes, frame_num: int, car_state: tuple, client_address):
    client_id = f"{client_address[0]}:{client_address[1]}"
    
    with state_lock:
        client_session = client_data.get(client_address)
        if not client_session: return
        decoder = client_session['decoder']
        decoder_lock = client_session['decoder_lock']
        image_history = client_session['image_history']

    video_frame = None
    with decoder_lock:
        try:
            packet = av.Packet(full_frame_data)
            decoded_frames = decoder.decode(packet)
            if decoded_frames:
                video_frame = decoded_frames[0]
        except Exception as e:
            logging.error(f"PyAV decoder error for {client_id} on frame {frame_num}: {e}")
            return
    
    if not video_frame: return

    try:
        img_320x192 = video_frame.to_image()
        crop_box = (0, 6, 320, 186)
        img_cropped = img_320x192.crop(crop_box)
        
        image_history.append(img_cropped)
        
        pred_throttle, pred_steer = 0.0, 0.0
        can_run_inference = len(image_history) >= HISTORY_LENGTH

        if can_run_inference:
            image_tensors = [image_transform(img) for img in image_history]
            image_seq_tensor = torch.stack(image_tensors, dim=0).unsqueeze(0).to(device)
            car_state_tensor = torch.tensor([list(car_state)], dtype=torch.float32).to(device)

            with torch.no_grad():
                prediction = model(image_seq_tensor, car_state_tensor)
                pred_throttle, pred_steer = prediction[0].cpu().numpy()
            
            logging.debug(f"Predicted throttle={pred_throttle:.2f}, steer={pred_steer:.2f} for frame {frame_num}.")

            if args.debug_every > 0 and frame_num % args.debug_every == 0:
                filename = f"client_{client_address[0]}_{client_address[1]}_frame_{frame_num}.png"
                debug_path = Path(args.debug_dir) / filename
                img_cropped.save(debug_path)
                logging.info(f"Saved debug image to {debug_path} for frame {frame_num}.")
        else:
            logging.debug(f"History not full ({len(image_history)}/{HISTORY_LENGTH}). Sending neutral command for frame {frame_num}.")

        client_ip, _ = client_address
        response_address = (client_ip, CLIENT_PORT_RECEIVE)
        response_packet = struct.pack('!Iff', frame_num, pred_throttle, pred_steer)
        sock.sendto(response_packet, response_address)

    except Exception as e:
        logging.error(f"Error in processing pipeline for {client_id} on frame {frame_num}: {e}", exc_info=True)

def cleanup_clients():
    while True:
        time.sleep(CLIENT_TIMEOUT_S)
        with state_lock:
            now = time.time()
            timed_out_clients = [addr for addr, data in client_data.items() if now - data['last_seen'] > CLIENT_TIMEOUT_S]
            for addr in timed_out_clients:
                del client_data[addr]
                logging.warning(f"Client {addr} timed out. Cleaned up resources.")

def main(cli_args):
    global model, image_transform, device, sock, args
    args = cli_args

    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    if not PYAV_AVAILABLE:
        logging.critical("PyAV library not found. Please run 'pip install av'. Exiting.")
        sys.exit(1)
    if not args.model.exists():
        logging.critical(f"Model file not found: {args.model}. Exiting.")
        sys.exit(1)
    if args.debug_every > 0:
        Path(args.debug_dir).mkdir(parents=True, exist_ok=True)
        logging.info(f"Debug mode enabled. Saving an image every {args.debug_every} frames to '{args.debug_dir}/'.")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logging.info(f"Using device: {device}")

    try:
        model = DirectPredictor(history_length=HISTORY_LENGTH).to(device)
        try:
            checkpoint = torch.load(args.model, map_location=device)
            model.load_state_dict(checkpoint['model_state_dict'])
            logging.info(f"Model '{args.model}' loaded from a training checkpoint.")
        except (KeyError, TypeError):
            model.load_state_dict(torch.load(args.model, map_location=device))
            logging.info(f"Model '{args.model}' loaded directly from a state dictionary.")
        model.eval()
    except Exception as e:
        logging.critical(f"Fatal Error: Could not load model. Error: {e}")
        sys.exit(1)

    image_transform = transforms.Compose([
        transforms.ToImage(),
        transforms.ToDtype(torch.float32, scale=True),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    logging.info("Model and transforms loaded successfully.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    logging.info(f"Listening on {args.host}:{args.port}")

    threading.Thread(target=cleanup_clients, daemon=True).start()

    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        while True:
            try:
                data, addr = sock.recvfrom(MAX_PACKET_SIZE)
                
                with state_lock:
                    if addr not in client_data:
                        logging.info(f"New client connected: {addr}")
                        client_data[addr] = {
                            'packet_buffer': {},
                            'image_history': deque(maxlen=HISTORY_LENGTH),
                            'decoder': av.CodecContext.create('h264', 'r'),
                            'decoder_lock': threading.Lock(),
                            'last_seen': time.time()
                        }
                    client_data[addr]['last_seen'] = time.time()

                if len(data) <= CHUNK_HEADER_SIZE: continue

                header = data[:CHUNK_HEADER_SIZE]
                payload = data[CHUNK_HEADER_SIZE:]
                frame_num, total_chunks, chunk_idx, throttle, steer = struct.unpack(CHUNK_HEADER_FORMAT, header)

                with state_lock:
                    buffer = client_data[addr]['packet_buffer']
                    if buffer.get('frame_num') != frame_num:
                        buffer.clear()
                        buffer['frame_num'] = frame_num
                        buffer['total_chunks'] = total_chunks
                        buffer['chunks'] = {}
                        buffer['car_state'] = (throttle, steer)
                    
                    if buffer['frame_num'] == frame_num:
                        buffer['chunks'][chunk_idx] = payload

                    if len(buffer.get('chunks', {})) == buffer.get('total_chunks'):
                        try:
                            full_frame_data = b''.join(buffer['chunks'][i] for i in range(buffer['total_chunks']))
                            car_state = buffer['car_state']
                            executor.submit(process_frame_and_predict, full_frame_data, frame_num, car_state, addr)
                        except KeyError:
                            logging.error(f"Missing chunk for frame {frame_num} from {addr}. Discarding.")
                        finally:
                            buffer.clear()

            except Exception as e:
                logging.error(f"Server loop error: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AI Driving Command Server for CNN+GRU models.")
    parser.add_argument("--model", type=Path, required=True, help="Path to the trained DirectPredictor model (.pth).")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host IP address to listen on.")
    parser.add_argument("--port", type=int, default=9999, help="Port to listen on.")
    parser.add_argument("--workers", type=int, default=4, help="Number of worker threads for processing frames.")
    parser.add_argument("--debug-every", type=int, default=0, help="Save a debug image every N frames. 0 to disable (default).")
    parser.add_argument("--debug-dir", type=str, default="debug_outputs", help="Directory to save debug images.")
    parsed_args = parser.parse_args()
    main(parsed_args)
```

`end-to-end/server_dual.py`:
```py
import socket
import struct
import threading
import time
import argparse
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import torch
import torch.nn as nn
import torchvision.transforms.v2 as transforms
from PIL import Image
import numpy as np
import logging

try:
    import av
    PYAV_AVAILABLE = True
except ImportError:
    PYAV_AVAILABLE = False

CLIENT_PORT_RECEIVE = 10000
CHUNK_HEADER_FORMAT = "!IHH"
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FORMAT)
MAX_PACKET_SIZE = 2048
CLIENT_TIMEOUT_S = 10.0
MODEL_INPUT_WIDTH = 320
MODEL_INPUT_HEIGHT = 180

class DoubleConv(nn.Module):
    def __init__(self, in_channels, out_channels, mid_channels=None):
        super().__init__()
        if not mid_channels: mid_channels = out_channels
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, mid_channels, 3, padding=1),
            nn.BatchNorm2d(mid_channels), nn.ReLU(inplace=True),
            nn.Conv2d(mid_channels, out_channels, 3, padding=1),
            nn.BatchNorm2d(out_channels), nn.ReLU(inplace=True)
        )
    def forward(self, x): return self.conv(x)

class Down(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.pool_conv = nn.Sequential(nn.MaxPool2d(2), DoubleConv(in_channels, out_channels))
    def forward(self, x): return self.pool_conv(x)

class Up(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.conv = DoubleConv(in_channels, out_channels, in_channels // 2)
    def forward(self, x1, x2):
        x1 = self.up(x1)
        dy = x2.size(2) - x1.size(2)
        dx = x2.size(3) - x1.size(3)
        x1 = nn.functional.pad(x1, [dx // 2, dx - dx // 2, dy // 2, dy - dy // 2])
        return self.conv(torch.cat([x2, x1], dim=1))

class OutConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.c = nn.Conv2d(in_channels, out_channels, 1)
    def forward(self, x): return self.c(x)

class SimpleUNet(nn.Module):
    def __init__(self, n_channels=3, n_classes=1):
        super().__init__()
        self.inc = DoubleConv(n_channels, 64)
        self.d1 = Down(64, 128)
        self.d2 = Down(128, 256)
        self.d3 = Down(256, 512)
        self.d4 = Down(512, 512)
        self.u1 = Up(1024, 256)
        self.u2 = Up(512, 128)
        self.u3 = Up(256, 64)
        self.u4 = Up(128, 64)
        self.outc = OutConv(64, n_classes)
    def forward(self, x):
        x1 = self.inc(x); x2 = self.d1(x1); x3 = self.d2(x2); x4 = self.d3(x3); x5 = self.d4(x4)
        x = self.u1(x5, x4); x = self.u2(x, x3); x = self.u3(x, x2); x = self.u4(x, x1)
        return self.outc(x)

class DrivingModel(nn.Module):
    def __init__(self, input_height=180, input_width=320):
        super().__init__()
        self.backbone = nn.Sequential(
            nn.Conv2d(1, 24, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Flatten(),
        )
        conv_output_size = self._get_conv_output_shape((input_height, input_width))
        self.head = nn.Sequential(
            nn.Linear(conv_output_size, 100), nn.ReLU(),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 10), nn.ReLU(),
            nn.Linear(10, 2), nn.Tanh()
        )
    def _get_conv_output_shape(self, shape):
        with torch.no_grad():
            dummy_input = torch.zeros(1, 1, *shape)
            return self.backbone(dummy_input).shape[1]
    def forward(self, x):
        features = self.backbone(x)
        return self.head(features)

client_data = {}
state_lock = threading.Lock()
model_mask = None
model_driver = None
mask_input_transform = None
device = None
args = None
sock = None

def process_frame_and_predict(full_frame_data: bytes, frame_num: int, client_address):
    client_id = f"{client_address[0]}:{client_address[1]}"
    
    with state_lock:
        client_session = client_data.get(client_address)
        if not client_session: return
        decoder = client_session['decoder']
        decoder_lock = client_session['decoder_lock']

    video_frame = None
    with decoder_lock:
        try:
            packet = av.Packet(full_frame_data)
            decoded_frames = decoder.decode(packet)
            if decoded_frames:
                video_frame = decoded_frames[0]
        except Exception as e:
            logging.error(f"PyAV decoder error for {client_id} on frame {frame_num}: {e}")
            return
    
    if not video_frame: return

    try:
        img_320x192 = video_frame.to_image()
        crop_box = (0, 6, 320, 186)
        img_cropped = img_320x192.crop(crop_box)
        
        with torch.no_grad():
            mask_input_tensor = mask_input_transform(img_cropped).unsqueeze(0).to(device)
            mask_logits = model_mask(mask_input_tensor)
            binary_mask = (torch.sigmoid(mask_logits) > 0.5).float()
            driver_input = (binary_mask * 255.0 - 128.0) / 128.0
            prediction = model_driver(driver_input)
            pred_throttle, pred_steer = prediction[0].cpu().numpy()

        logging.debug(f"Predicted throttle={pred_throttle:.2f}, steer={pred_steer:.2f} for frame {frame_num}.")

        if args.debug_every > 0 and frame_num % args.debug_every == 0:
            filename = f"client_{client_address[0]}_{client_address[1]}_frame_{frame_num}.png"
            debug_path = Path(args.debug_dir) / filename
            img_cropped.save(debug_path)
            logging.info(f"Saved debug image to {debug_path} for frame {frame_num}.")

        client_ip, _ = client_address
        response_address = (client_ip, CLIENT_PORT_RECEIVE)
        response_packet = struct.pack('!Iff', frame_num, pred_throttle, pred_steer)
        sock.sendto(response_packet, response_address)

    except Exception as e:
        logging.error(f"Error in processing pipeline for {client_id} on frame {frame_num}: {e}", exc_info=True)

def cleanup_clients():
    while True:
        time.sleep(CLIENT_TIMEOUT_S)
        with state_lock:
            now = time.time()
            timed_out_clients = [addr for addr, data in client_data.items() if now - data['last_seen'] > CLIENT_TIMEOUT_S]
            for addr in timed_out_clients:
                del client_data[addr]
                logging.warning(f"Client {addr} timed out. Cleaned up resources.")

def main(cli_args):
    global model_mask, model_driver, mask_input_transform, device, sock, args
    args = cli_args

    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    if not PYAV_AVAILABLE:
        logging.critical("PyAV library not found. Please run 'pip install av'. Exiting.")
        sys.exit(1)
    if not args.mask_model.exists(): sys.exit(f"Error: Mask model not found at '{args.mask_model}'")
    if not args.driver_model.exists(): sys.exit(f"Error: Driver model not found at '{args.driver_model}'")

    if args.debug_every > 0:
        Path(args.debug_dir).mkdir(parents=True, exist_ok=True)
        logging.info(f"Debug mode enabled. Saving an image every {args.debug_every} frames to '{args.debug_dir}/'.")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logging.info(f"Using device: {device}")

    try:
        model_mask = SimpleUNet(n_channels=3, n_classes=1).to(device)
        model_driver = DrivingModel(input_height=MODEL_INPUT_HEIGHT, input_width=MODEL_INPUT_WIDTH).to(device)

        model_mask.load_state_dict(torch.load(args.mask_model, map_location=device))
        
        driver_checkpoint = torch.load(args.driver_model, map_location=device)
        driver_state_dict = driver_checkpoint.get('model_state_dict', driver_checkpoint)
        model_driver.load_state_dict(driver_state_dict)

        model_mask.eval()
        model_driver.eval()
        logging.info("Models loaded successfully.")
    except Exception as e:
        logging.critical(f"Fatal Error: Could not load models. Error: {e}")
        sys.exit(1)

    mask_input_transform = transforms.Compose([
        transforms.ToImage(),
        transforms.ToDtype(torch.float32, scale=True),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    logging.info("Transforms initialized.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    logging.info(f"Listening on {args.host}:{args.port}")

    threading.Thread(target=cleanup_clients, daemon=True).start()

    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        while True:
            try:
                data, addr = sock.recvfrom(MAX_PACKET_SIZE)
                
                with state_lock:
                    if addr not in client_data:
                        logging.info(f"New client connected: {addr}")
                        client_data[addr] = {
                            'packet_buffer': {},
                            'decoder': av.CodecContext.create('h264', 'r'),
                            'decoder_lock': threading.Lock(),
                            'last_seen': time.time()
                        }
                    client_data[addr]['last_seen'] = time.time()

                if len(data) <= CHUNK_HEADER_SIZE: continue

                header = data[:CHUNK_HEADER_SIZE]
                payload = data[CHUNK_HEADER_SIZE:]
                frame_num, total_chunks, chunk_idx = struct.unpack(CHUNK_HEADER_FORMAT, header)

                with state_lock:
                    buffer = client_data[addr]['packet_buffer']
                    if buffer.get('frame_num') != frame_num:
                        buffer.clear()
                        buffer['frame_num'] = frame_num
                        buffer['total_chunks'] = total_chunks
                        buffer['chunks'] = {}
                    
                    if buffer['frame_num'] == frame_num:
                        buffer['chunks'][chunk_idx] = payload

                    if len(buffer.get('chunks', {})) == buffer.get('total_chunks'):
                        try:
                            full_frame_data = b''.join(buffer['chunks'][i] for i in range(buffer['total_chunks']))
                            executor.submit(process_frame_and_predict, full_frame_data, frame_num, addr)
                        except KeyError:
                            logging.error(f"Missing chunk for frame {frame_num} from {addr}. Discarding.")
                        finally:
                            buffer.clear()

            except Exception as e:
                logging.error(f"Server loop error: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AI Driving Command Server for two-stage models.")
    parser.add_argument("--mask-model", type=Path, required=True, help="Path to the trained U-Net mask model (.pth).")
    parser.add_argument("--driver-model", type=Path, required=True, help="Path to the trained Driving model (.pth).")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host IP address to listen on.")
    parser.add_argument("--port", type=int, default=9999, help="Port to listen on.")
    parser.add_argument("--workers", type=int, default=4, help="Number of worker threads for processing frames.")
    parser.add_argument("--debug-every", type=int, default=0, help="Save a debug image every N frames. 0 to disable.")
    parser.add_argument("--debug-dir", type=str, default="debug_outputs", help="Directory to save debug images.")
    parsed_args = parser.parse_args()
    main(parsed_args)
```

`mask-workbench/real/server.py`:
```py
import argparse
import os
import socket
import struct
import sys
import threading
import time
from queue import Queue

import av
import torch
import torch.nn as nn
import torchvision.transforms.v2 as transforms
from PIL import Image, ImageDraw, ImageFont

# ======================================================================================
# --- MODEL DEFINITIONS (FROM video2video.py) ---
# ======================================================================================

class DoubleConv(nn.Module):
    def __init__(self, in_channels, out_channels, mid_channels=None):
        super().__init__()
        if not mid_channels: mid_channels = out_channels
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, mid_channels, 3, padding=1),
            nn.BatchNorm2d(mid_channels), nn.ReLU(inplace=True),
            nn.Conv2d(mid_channels, out_channels, 3, padding=1),
            nn.BatchNorm2d(out_channels), nn.ReLU(inplace=True)
        )
    def forward(self, x): return self.conv(x)

class Down(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.pool_conv = nn.Sequential(nn.MaxPool2d(2), DoubleConv(in_channels, out_channels))
    def forward(self, x): return self.pool_conv(x)

class Up(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.conv = DoubleConv(in_channels, out_channels, in_channels // 2)
    def forward(self, x1, x2):
        x1 = self.up(x1)
        dy = x2.size(2) - x1.size(2)
        dx = x2.size(3) - x1.size(3)
        x1 = nn.functional.pad(x1, [dx // 2, dx - dx // 2, dy // 2, dy - dy // 2])
        return self.conv(torch.cat([x2, x1], dim=1))

class OutConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.c = nn.Conv2d(in_channels, out_channels, 1)
    def forward(self, x): return self.c(x)

class SimpleUNet(nn.Module):
    def __init__(self, n_channels=3, n_classes=1):
        super().__init__()
        self.inc = DoubleConv(n_channels, 64)
        self.d1 = Down(64, 128)
        self.d2 = Down(128, 256)
        self.d3 = Down(256, 512)
        self.d4 = Down(512, 512)
        self.u1 = Up(1024, 256)
        self.u2 = Up(512, 128)
        self.u3 = Up(256, 64)
        self.u4 = Up(128, 64)
        self.outc = OutConv(64, n_classes)
    def forward(self, x):
        x1 = self.inc(x); x2 = self.d1(x1); x3 = self.d2(x2); x4 = self.d3(x3); x5 = self.d4(x4)
        x = self.u1(x5, x4); x = self.u2(x, x3); x = self.u3(x, x2); x = self.u4(x, x1)
        return self.outc(x)

class DrivingModel(nn.Module):
    def __init__(self, input_height=180, input_width=320):
        super().__init__()
        self.backbone = nn.Sequential(
            nn.Conv2d(1, 24, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Flatten(),
        )
        conv_output_size = self._get_conv_output_shape((input_height, input_width))
        self.head = nn.Sequential(
            nn.Linear(conv_output_size, 100), nn.ReLU(),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 10), nn.ReLU(),
            nn.Linear(10, 2), nn.Tanh()
        )
    def _get_conv_output_shape(self, shape):
        with torch.no_grad():
            dummy_input = torch.zeros(1, 1, *shape)
            return self.backbone(dummy_input).shape[1]
    def forward(self, x):
        features = self.backbone(x)
        return self.head(features)

# ======================================================================================
# --- HELPER FUNCTIONS (FOR DEBUGGING) ---
# ======================================================================================

def draw_trajectory(draw, w, h, throttle, steer):
    print(f"Called draw_trajectory({draw}, {w}, {h}, {throttle}, {steer})")
    HORIZON_Y_FACTOR, MAX_DEVIATION_FACTOR, STEERING_DAMPING_FACTOR, CONTROL_POINT_Y_FACTOR = 0.2, 0.3, 0.7, 0.4
    LINE_WIDTH, FORWARD_COLOR = 3, '#00FFFF'
    if throttle <= 0: return
    horizon_y = h * HORIZON_Y_FACTOR; start_x, start_y = w / 2, h
    path_length = (start_y - horizon_y) * abs(throttle); end_y = start_y - path_length
    max_deviation = w * MAX_DEVIATION_FACTOR; steering_reduction = 1 - (abs(throttle) * STEERING_DAMPING_FACTOR)
    end_x = start_x + steer * max_deviation * steering_reduction
    control_x, control_y = start_x, start_y - (start_y - end_y) * CONTROL_POINT_Y_FACTOR
    points = [(start_x, start_y)]
    for t_norm in [i / 100.0 for i in range(1, 101)]:
        x = (1 - t_norm)**2 * start_x + 2 * (1 - t_norm) * t_norm * control_x + t_norm**2 * end_x
        y = (1 - t_norm)**2 * start_y + 2 * (1 - t_norm) * t_norm * control_y + t_norm**2 * end_y
        points.append((x, y))
    draw.line(points, fill=FORWARD_COLOR, width=LINE_WIDTH)

def get_font(size: int) -> ImageFont.ImageFont:
    try: return ImageFont.truetype("arial.ttf", size)
    except IOError: return ImageFont.load_default()

# ======================================================================================
# --- CLIENT HANDLER THREAD ---
# ======================================================================================

class ClientHandler(threading.Thread):
    def __init__(self, address, data_queue, main_socket, models, device, debug_every=0):
        super().__init__()
        self.address = address
        self.data_queue = data_queue
        self.main_socket = main_socket
        self.model_mask, self.model_driver = models
        self.device = device
        self.debug_every = debug_every
        self.font = get_font(14) if self.debug_every > 0 else None
        self.stop_event = threading.Event()

        self.codec = av.CodecContext.create('h264', 'r')
        self.mask_input_transform = transforms.Compose([
            transforms.ToImage(), transforms.ToDtype(torch.float32, scale=True),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def stop(self):
        self.stop_event.set()

    def run(self):
        print(f"[+] New client connected: {self.address}")
        with torch.no_grad():
            while not self.stop_event.is_set():
                try:
                    data = self.data_queue.get(timeout=1.0)
                    frame_num = struct.unpack('!L', data[:4])[0]
                    packets = self.codec.parse(data[4:])
                    if not packets:
                        continue

                    for packet in packets:
                        frames = self.codec.decode(packet)
                        for frame in frames:
                            self.process_frame(frame, frame_num)
                except Exception as e:
                    if not self.data_queue.empty():
                        print(f"[-] Error processing frame for {self.address}: {e}")
                    continue
        print(f"[-] Client disconnected: {self.address}")

    def process_frame(self, frame, frame_num):
        pil_img = frame.to_image()
        mask_input = self.mask_input_transform(pil_img).unsqueeze(0).to(self.device)

        mask_logits = self.model_mask(mask_input)
        binary_mask = (torch.sigmoid(mask_logits) > 0.5).float()
        driver_input = (binary_mask * 255.0 - 128.0) / 128.0

        predictions = self.model_driver(driver_input).squeeze()
        
        # --- FIX: Correctly assign model outputs based on video2video.py ground truth ---
        # The model output order is [steer, throttle].
        steer_val = predictions[0].item()
        throttle_val = predictions[1].item()
        
        if self.debug_every > 0 and frame_num % self.debug_every == 0:
            print(f"[DEBUG] Frame {frame_num}: Raw model output [steer, throttle]: {predictions.cpu().numpy()}. Assigned steer={steer_val:.2f}, throttle={throttle_val:.2f}")

        # Packet format is (frame_num, throttle, steer)
        response_packet = struct.pack('!Lff', frame_num, throttle_val, steer_val)
        self.main_socket.sendto(response_packet, self.address)

        if self.debug_every > 0 and frame_num % self.debug_every == 0:
            self.save_debug_image(pil_img, binary_mask, throttle_val, steer_val, frame_num)

    def save_debug_image(self, base_pil, binary_mask_tensor, throttle, steer, frame_num):
        try:
            final_img = base_pil.convert("RGBA")
            track_mask_pil = transforms.ToPILImage()((binary_mask_tensor.squeeze().cpu() * 255).byte())
            green_overlay = Image.new("RGBA", final_img.size, (0, 255, 0, 100))
            final_img = Image.composite(green_overlay, final_img, track_mask_pil)
            final_img = final_img.convert("RGB")
            draw = ImageDraw.Draw(final_img)
            
            draw_trajectory(draw, final_img.width, final_img.height, throttle, steer)
            
            text = f"Steer: {steer:.2f} | Throttle: {throttle:.2f}"
            draw.text((6, 6), text, font=self.font, fill="black")
            draw.text((5, 5), text, font=self.font, fill="white")
            
            filename = f"debug_outputs/client_{self.address[0]}_{self.address[1]}_frame_{frame_num}.png"
            final_img.save(filename)
        except Exception as e:
            print(f"[ERROR] Could not save debug image: {e}")

# ======================================================================================
# --- MAIN SERVER SCRIPT ---
# ======================================================================================

def main():
    parser = argparse.ArgumentParser(description="AI Driving Control Server")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host IP to bind to.")
    parser.add_argument("--port", type=int, default=3993, help="Port to listen on.")
    parser.add_argument("--mask", type=str, required=True, help="Path to U-Net mask model (.pth).")
    parser.add_argument("--driver", type=str, required=True, help="Path to driving command model (.pth).")
    parser.add_argument("--timeout", type=int, default=5, help="Seconds of inactivity before dropping a client.")
    parser.add_argument("--debug-every", type=int, default=0, help="Save a debug image every N frames. 0 to disable.")
    args = parser.parse_args()

    if not torch.cuda.is_available():
        sys.exit("Error: This server requires a CUDA-enabled GPU.")
    device = torch.device("cuda")
    print(f"Using device: {device}")

    if args.debug_every > 0:
        os.makedirs("debug_outputs", exist_ok=True)
        print(f"[INFO] Debug mode enabled. Saving an image every {args.debug_every} frames to 'debug_outputs/'.")

    print("Loading models...")
    model_mask = SimpleUNet(n_channels=3, n_classes=1)
    model_driver = DrivingModel(input_height=180, input_width=320)
    try:
        model_mask.load_state_dict(torch.load(args.mask, map_location=device))
        driver_checkpoint = torch.load(args.driver, map_location=device)
        driver_state_dict = driver_checkpoint.get('model_state_dict', driver_checkpoint)
        model_driver.load_state_dict(driver_state_dict)
    except Exception as e:
        sys.exit(f"Error loading model state_dict: {e}")
    model_mask.to(device).eval()
    model_driver.to(device).eval()
    models = (model_mask, model_driver)
    print("Models loaded successfully.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    print(f"[*] Listening on {args.host}:{args.port}")

    clients = {}
    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
                if addr not in clients:
                    q = Queue()
                    client_thread = ClientHandler(addr, q, sock, models, device, args.debug_every)
                    client_thread.start()
                    clients[addr] = {'queue': q, 'thread': client_thread, 'last_seen': time.time()}

                if addr in clients:
                    clients[addr]['queue'].put(data)
                    clients[addr]['last_seen'] = time.time()

                dead_clients = []
                for client_addr, client_info in clients.items():
                    if time.time() - client_info['last_seen'] > args.timeout:
                        dead_clients.append(client_addr)

                for client_addr in dead_clients:
                    print(f"[*] Client {client_addr} timed out. Cleaning up.")
                    clients[client_addr]['thread'].stop()
                    clients[client_addr]['thread'].join()
                    del clients[client_addr]

            except Exception as e:
                print(f"[!] Server loop error: {e}")

    except KeyboardInterrupt:
        print("\n[*] Shutting down server...")
        for client_info in clients.values():
            client_info['thread'].stop()
            client_info['thread'].join()
    finally:
        sock.close()
        print("[*] Server shut down.")

if __name__ == "__main__":
    main()
```

