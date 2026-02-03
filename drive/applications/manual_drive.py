"""
Manual driving application using joystick control.
"""

import os
# Set dummy video driver for headless systems (must be BEFORE pygame import)
os.environ["SDL_VIDEODRIVER"] = "dummy"

import sys
import time
import signal
import pygame
from drive.core import (
    MotorController, JoystickController, ThrottleController, CameraController,
    LidarController, PanTiltController, RTKController, SocketServer, DataPublisher
)
from drive.core.joystick_controller import Input, Axis
from drive.core.config import (
    LOOP_SLEEP_TIME, DEFAULT_SERIAL_PORT, MAX_SPEED,
    DEFAULT_LIDAR_PORT, SOCKETIO_PORT, PAN_TILT_SERIAL_PORT, DEFAULT_RTK_SERIAL_PORT
)


class ManualDriveApp:
    """
    Application for manual driving with joystick control.
    """
    
    def __init__(self, max_speed=MAX_SPEED, serial_port=DEFAULT_SERIAL_PORT, 
                 use_camera=False, enable_socket=False, lidar_port=None, socket_port=SOCKETIO_PORT,
                 use_motor=True, pan_tilt_port=None, use_pan_tilt=True,
                 use_joystick=True, use_throttle=True, use_lidar=None,
                 rtk_port=None, use_rtk=True):
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
            pan_tilt_port: Serial port for pan/tilt controller. If None, uses default from config.
            use_pan_tilt: Whether to enable pan/tilt control. If False, pan/tilt is disabled.
            use_joystick: Whether to enable joystick controller. If False, joystick is disabled.
            use_throttle: Whether to enable throttle controller. If False, throttle is disabled.
            use_lidar: Whether to enable lidar. If None, auto-enabled if lidar_port is provided.
            rtk_port: Serial port for RTK GNSS. If None, uses default from config.
            use_rtk: Whether to enable RTK GNSS (pose/IMU). If False, RTK is disabled.
        """
        self.max_speed = max_speed
        self.serial_port = serial_port
        
        # Module flags (stored as attributes for documentation and introspection)
        self.use_motor = use_motor
        self.use_joystick = use_joystick
        self.use_throttle = use_throttle
        self.use_camera = use_camera
        self.use_lidar = use_lidar if use_lidar is not None else (lidar_port is not None)
        self.use_pan_tilt = use_pan_tilt
        self.use_rtk = use_rtk
        self.enable_socket = enable_socket
        
        # Store lidar_port for reference
        self.lidar_port = lidar_port
        
        # Initialize controllers based on flags
        self.motor = MotorController(serial_port=serial_port, enabled=use_motor) if use_motor else None
        self.joystick = JoystickController() if use_joystick else None
        self.throttle = ThrottleController(max_speed=max_speed) if use_throttle else None
        self.camera = CameraController() if use_camera else None
        self.lidar = LidarController(serial_port=lidar_port) if self.use_lidar and lidar_port else None
        self.pantilt = PanTiltController(
            serial_port=pan_tilt_port or PAN_TILT_SERIAL_PORT,
            enabled=use_pan_tilt
        ) if use_pan_tilt else None
        self.rtk = RTKController(
            serial_port=rtk_port or DEFAULT_RTK_SERIAL_PORT,
            enabled=use_rtk
        ) if use_rtk else None
        
        # Socket.io components
        self.socket_server = None
        self.data_publisher = None
        
        if enable_socket:
            self.socket_server = SocketServer(port=socket_port)
            self.data_publisher = DataPublisher(
                lidar_controller=self.lidar,
                camera_controller=self.camera,
                rtk_controller=self.rtk,
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
        if not self.use_joystick or self.joystick is None:
            return False
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
            
            if self.use_motor and self.motor is not None:
                try:
                    self.motor.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize motor: {e}")
                    print("[ManualDrive] Motor will be disabled")
                    self.motor = None
                    self.use_motor = False
            elif not self.use_motor:
                print("[ManualDrive] Motor disabled")
            
            if self.use_joystick and self.joystick is not None:
                try:
                    self.joystick.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize joystick: {e}")
                    print("[ManualDrive] Joystick will be disabled")
                    self.joystick = None
                    self.use_joystick = False
            elif not self.use_joystick:
                print("[ManualDrive] Joystick disabled")
            
            if self.use_camera and self.camera is not None:
                try:
                    self.camera.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize camera: {e}")
                    self.camera = None
                    self.use_camera = False
            
            if self.use_lidar and self.lidar is not None:
                try:
                    self.lidar.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize lidar: {e}")
                    self.lidar = None
            
            if self.use_pan_tilt and self.pantilt is not None:
                try:
                    self.pantilt.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize pan/tilt: {e}")
                    self.pantilt = None
            
            if self.use_rtk and self.rtk is not None:
                try:
                    self.rtk.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize RTK: {e}")
                    self.rtk = None
                    self.use_rtk = False
            elif not self.use_rtk:
                print("[ManualDrive] RTK disabled")
            
            # Start socket server and data publisher if enabled
            if self.enable_socket and self.socket_server is not None:
                try:
                    self.socket_server.start()
                    if self.data_publisher is not None:
                        self.data_publisher.start()
                    print(f"[ManualDrive] Socket.io server running on port {self.socket_server.port}")
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to start socket server: {e}")
            
            print("[ManualDrive] Ready! Use BACK+START to exit.")
            
            # Main control loop
            while self.running:
                # Process pygame events (needed for joystick input)
                pygame.event.pump()
                
                # Update joystick inputs (if enabled)
                if self.use_joystick and self.joystick is not None:
                    self.joystick.update()
                
                # Check for exit request
                if self._handle_exit():
                    print("[ManualDrive] Exit requested (BACK+START).")
                    break
                
                # Compute target throttle from triggers (if enabled)
                acceleration = 0.0
                steering = 0.0
                
                if self.use_throttle and self.throttle is not None and self.use_joystick and self.joystick is not None:
                    rt_value = self.joystick.get_rt()
                    lt_value = self.joystick.get_lt()
                    target = self.throttle.compute_target(rt_value, lt_value)
                    
                    # Update throttle with smooth interpolation
                    self.throttle.update(target)
                    acceleration = self.throttle.get_acceleration()
                
                # Get steering input (if joystick enabled)
                if self.use_joystick and self.joystick is not None:
                    steering = self.joystick.get_steering()
                
                # Apply motor commands (only if motor is enabled)
                if self.use_motor and self.motor is not None:
                    self.motor.set_commands(acceleration, steering)
                
                # Process pan/tilt control with D-pad (flèches du contrôleur)
                if self.use_pan_tilt and self.pantilt is not None and self.use_joystick and self.joystick is not None:
                    # Read D-pad/hat values (flèches du contrôleur)
                    hat_x = self.joystick.get_axis(Input.HAT_X)  # Left (-1) / Right (1)
                    hat_y = self.joystick.get_axis(Input.HAT_Y)  # Down (-1) / Up (1)
                    
                    # Convertir les valeurs du D-pad en deltas pour update() (accumulation continue)
                    pan_delta = hat_x  # Left = -1, Right = +1
                    tilt_delta = hat_y  # Down = -1, Up = +1
                    
                    # Update pan/tilt avec accumulation continue (ne se réinitialise pas)
                    self.pantilt.update(pan_delta, tilt_delta)
                    
                    # Debug: déterminer quelle flèche est appuyée
                    arrow = "NONE"
                    if hat_x == -1:
                        arrow = "LEFT"
                    elif hat_x == 1:
                        arrow = "RIGHT"
                    elif hat_y == 1:
                        arrow = "UP"
                    elif hat_y == -1:
                        arrow = "DOWN"
                    
                    # Récupérer l'état actuel de x et y
                    pan_pos, tilt_pos = self.pantilt.get_position()
                    
                    # Print debug avec état des flèches et positions
                    sys.stdout.write(f"\rD-pad: {arrow:4s} | hat_x: {hat_x:+2.0f} | hat_y: {hat_y:+2.0f} | Pan(X): {pan_pos:+.2f} | Tilt(Y): {tilt_pos:+.2f}   ")
                    sys.stdout.flush()
                
                # Process camera if enabled
                if self.use_camera and self.camera is not None:
                    frame = self._process_camera()
                    if frame is not None:
                        # Frame available for display/processing
                        pass
                
                # Update RTK (pose/IMU) if enabled
                if self.use_rtk and self.rtk is not None:
                    self.rtk.update()
                
                # Print status
                if self.use_throttle or self.use_joystick:
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
        if self.use_motor and self.motor is not None:
            self.motor.stop()
        
        # Stop camera
        if self.use_camera and self.camera is not None:
            self.camera.stop()
        
        # Stop lidar
        if self.use_lidar and self.lidar is not None:
            self.lidar.stop()
        
        # Stop pan/tilt
        if self.use_pan_tilt and self.pantilt is not None:
            self.pantilt.stop()
        
        # Stop RTK
        if self.rtk is not None:
            self.rtk.stop()
        
        print("[ManualDrive] Shutdown complete.")
