"""
Manual driving application using joystick control.
"""

import time
import signal
from drive.core import (
    MotorController, JoystickController, ThrottleController, CameraController,
    LidarController, SocketServer, DataPublisher
)
from drive.core.joystick_controller import Input
from drive.core.config import (
    LOOP_SLEEP_TIME, DEFAULT_SERIAL_PORT, MAX_SPEED,
    DEFAULT_LIDAR_PORT, SOCKETIO_PORT
)


class ManualDriveApp:
    """
    Application for manual driving with joystick control.
    """
    
    def __init__(self, max_speed=MAX_SPEED, serial_port=DEFAULT_SERIAL_PORT, 
                 use_camera=False, enable_socket=False, lidar_port=None, socket_port=SOCKETIO_PORT,
                 use_motor=True):
        """
        Initialize the manual drive application.
        
        Args:
            max_speed: Maximum speed for forward/backward movement.
            serial_port: Serial port for VESC motor.
            use_camera: Whether to use camera (optional, for display/recording).
            enable_socket: Whether to enable socket.io server for data streaming.
            lidar_port: Serial port for lidar (e.g., '/dev/ttyUSB0'). If None, lidar is disabled.
            socket_port: Port for socket.io server.
            use_motor: Whether to enable motor/VESC. If False, motor is disabled (mock mode).
        """
        self.max_speed = max_speed
        self.serial_port = serial_port
        self.use_camera = use_camera
        self.enable_socket = enable_socket
        self.lidar_port = lidar_port
        self.use_motor = use_motor
        
        self.motor = MotorController(serial_port=serial_port, enabled=use_motor)
        self.joystick = JoystickController()
        self.throttle = ThrottleController(max_speed=max_speed)
        self.camera = CameraController() if use_camera else None
        self.lidar = LidarController(serial_port=lidar_port) if lidar_port else None
        
        # Socket.io components
        self.socket_server = None
        self.data_publisher = None
        
        if enable_socket:
            self.socket_server = SocketServer(port=socket_port)
            self.data_publisher = DataPublisher(
                lidar_controller=self.lidar,
                camera_controller=self.camera,
                socket_server=self.socket_server
            )
        
        self.running = True
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, sig, frame):
        """Handle shutdown signals."""
        print("\n[Signal] Exiting...")
        self.running = False
    
    def _handle_exit(self):
        """
        Check if exit is requested (BACK + START buttons).
        
        Returns:
            bool: True if exit requested, False otherwise.
        """
        return (self.joystick.get_button(Input.BACK) and 
                self.joystick.get_button(Input.START))
    
    def _process_camera(self):
        """
        Process camera frame if camera is enabled.
        
        Returns:
            numpy.ndarray or None: Frame if available, None otherwise.
        """
        if self.camera is not None and self.camera.is_available():
            return self.camera.get_frame()
        return None
    
    def run(self):
        """
        Run the main control loop.
        """
        try:
            # Initialize all controllers
            print("[ManualDrive] Initializing controllers...")
            if self.use_motor:
                self.motor.initialize()
            else:
                print("[ManualDrive] Motor disabled (mock mode)")
            self.joystick.initialize()
            
            if self.camera is not None:
                self.camera.initialize()
            
            if self.lidar is not None:
                try:
                    self.lidar.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize lidar: {e}")
                    self.lidar = None
            
            # Start socket server and data publisher if enabled
            if self.enable_socket and self.socket_server is not None:
                self.socket_server.start()
                if self.data_publisher is not None:
                    self.data_publisher.start()
                print(f"[ManualDrive] Socket.io server running on port {self.socket_server.port}")
            
            print("[ManualDrive] Ready! Use BACK+START to exit.")
            
            # Main control loop
            while self.running:
                # Update joystick inputs
                self.joystick.update()
                
                # Check for exit request
                if self._handle_exit():
                    print("[ManualDrive] Exit requested (BACK+START).")
                    break
                
                # Compute target throttle from triggers
                rt_value = self.joystick.get_rt()
                lt_value = self.joystick.get_lt()
                target = self.throttle.compute_target(rt_value, lt_value)
                
                # Update throttle with smooth interpolation
                self.throttle.update(target)
                acceleration = self.throttle.get_acceleration()
                
                # Get steering input
                steering = self.joystick.get_steering()
                
                # Apply motor commands (only if motor is enabled)
                if self.use_motor:
                    self.motor.set_commands(acceleration, steering)
                
                # Process camera if enabled
                if self.use_camera:
                    frame = self._process_camera()
                    if frame is not None:
                        # Frame available for display/processing
                        pass
                
                # Print status
                print(f"Duty: {acceleration:.3f} | Steer: {(steering + 1) / 2:.3f}")
                
                # Control loop frequency
                time.sleep(LOOP_SLEEP_TIME)
        
        except KeyboardInterrupt:
            print("\n[ManualDrive] Keyboard interrupt received.")
        except Exception as e:
            print(f"[ManualDrive] Error: {e}")
            raise
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        print("[ManualDrive] Cleaning up...")
        
        # Stop data publisher and socket server
        if self.data_publisher is not None:
            self.data_publisher.stop()
        
        if self.socket_server is not None:
            self.socket_server.stop()
        
        # Stop motor (only if enabled)
        if self.use_motor:
            self.motor.stop()
        
        # Stop camera
        if self.camera is not None:
            self.camera.stop()
        
        # Stop lidar
        if self.lidar is not None:
            self.lidar.stop()
        
        print("[ManualDrive] Shutdown complete.")
