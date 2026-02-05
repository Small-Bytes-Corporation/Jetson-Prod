"""
Manual driving application using joystick control.
"""

import os
# Set dummy video driver for headless systems (must be BEFORE pygame import)
os.environ["SDL_VIDEODRIVER"] = "dummy"

import sys
import time
import pygame
from drive.core import (
    MotorController, JoystickController, ThrottleController, CameraController,
    LidarController, LidarNavigator, PanTiltController, UDPServer, DataPublisher
)
from drive.core.dashboard import Dashboard
from drive.core.joystick_controller import Input, Axis
from drive.core.config import (
    LOOP_SLEEP_TIME, DEFAULT_SERIAL_PORT, MAX_SPEED,
    DEFAULT_LIDAR_PORT, UDP_PORT, PAN_TILT_SERIAL_PORT,
    CAMERA_STARTUP_DELAY,
)


class ManualDriveApp:
    """
    Application for manual driving with joystick control.
    """
    
    def __init__(self, max_speed=MAX_SPEED, serial_port=DEFAULT_SERIAL_PORT, 
                 use_camera=False, enable_socket=False, lidar_port=None, socket_port=UDP_PORT,
                 socket_debug=False,
                 use_motor=True, pan_tilt_port=None, use_pan_tilt=True,
                 use_joystick=True, use_throttle=True, use_lidar=None,
                 debug_pan_tilt=False, debug_joystick=False, debug_lidar=False, debug_camera=False,
                 dashboard=False):
        """
        Initialize the manual drive application.
        
        Args:
            max_speed: Maximum speed for forward/backward movement.
            serial_port: Serial port for VESC motor.
            use_camera: Whether to use camera (optional, for display/recording).
            enable_socket: Whether to enable UDP server for data streaming.
            lidar_port: Serial port for lidar (e.g., '/dev/ttyUSB0'). If None, lidar is disabled.
            socket_port: Port for UDP server.
            socket_debug: If True, print full socket payloads (sensor_data, status) to console.
            use_motor: Whether to enable motor/VESC. If False, motor is disabled (mock mode).
            pan_tilt_port: Serial port for pan/tilt controller. If None, uses default from config.
            use_pan_tilt: Whether to enable pan/tilt control. If False, pan/tilt is disabled.
            use_joystick: Whether to enable joystick controller. If False, joystick is disabled.
            use_throttle: Whether to enable throttle controller. If False, throttle is disabled.
            use_lidar: Whether to enable lidar. If None, auto-enabled if lidar_port is provided.
            debug_pan_tilt: If True, print pan/tilt debug messages.
            debug_joystick: If True, print joystick input and status (Duty/Steer).
            debug_lidar: If True, print lidar debug messages.
            debug_camera: If True, print camera debug messages.
            dashboard: If True, enable terminal dashboard with all debug info.
        """
        # Si dashboard est activé, désactiver tous les prints (affichés dans le dashboard)
        if dashboard:
            debug_pan_tilt = False  # Désactivé pour éviter les prints (affiché dans dashboard)
            debug_joystick = False  # Désactivé pour éviter les prints (affiché dans dashboard)
            debug_lidar = False  # Désactivé pour éviter les prints verbeux (affiché dans dashboard)
            debug_camera = False  # Désactivé pour éviter les prints (affiché dans dashboard)
            socket_debug = True  # Gardé pour les données socket (mais les prints sont supprimés dans UDPServer)
        
        self.max_speed = max_speed
        self.debug_pan_tilt = debug_pan_tilt
        self.debug_joystick = debug_joystick
        self.debug_lidar = debug_lidar
        self.debug_camera = debug_camera
        self.dashboard_enabled = dashboard
        self.serial_port = serial_port
        
        # Module flags (stored as attributes for documentation and introspection)
        self.use_motor = use_motor
        self.use_joystick = use_joystick
        self.use_throttle = use_throttle
        self.use_camera = use_camera
        self.use_lidar = use_lidar if use_lidar is not None else (lidar_port is not None)
        self.use_pan_tilt = use_pan_tilt
        self.enable_socket = enable_socket
        
        # Store lidar_port for reference
        self.lidar_port = lidar_port
        
        # Initialize controllers based on flags
        # Note: error_callback will be set after dashboard creation
        self.motor = MotorController(serial_port=serial_port, enabled=use_motor) if use_motor else None
        self.joystick = JoystickController(debug=debug_joystick) if use_joystick else None
        self.throttle = ThrottleController(max_speed=max_speed) if use_throttle else None
        self.pantilt = PanTiltController(
            serial_port=pan_tilt_port or PAN_TILT_SERIAL_PORT,
            enabled=use_pan_tilt,
            debug=debug_pan_tilt
        ) if use_pan_tilt else None
        
        # Initialize sensors and socket components
        self.camera = CameraController(debug=debug_camera) if use_camera else None
        self.lidar = LidarController(serial_port=lidar_port, debug=debug_lidar) if self.use_lidar and lidar_port else None
        
        # UDP network components
        self.socket_server = None
        self.data_publisher = None
        
        if enable_socket:
            # Suppress debug prints if dashboard is enabled (info displayed in dashboard instead)
            self.socket_server = UDPServer(port=socket_port, debug_payload=socket_debug, suppress_debug_prints=dashboard)
            self.data_publisher = DataPublisher(
                lidar_controller=self.lidar,
                camera_controller=self.camera,
                socket_server=self.socket_server,
                debug_camera=debug_camera,
                debug_lidar=debug_lidar
            )
        
        self.running = True
        
        # Store socket config
        self.socket_port = socket_port
        self.socket_debug = socket_debug
        
        # Autonomous mode
        self.autonomous_mode = False
        self.b_button_pressed = False
        self.lidar_navigator = None
        self._init_errors = {}  # module -> error message for Failed status

        # Initialize lidar navigator if lidar is available
        if self.use_lidar and (self.lidar is not None or lidar_port is not None):
            from drive.core.config import AUTONOMOUS_MAX_SPEED
            self.lidar_navigator = LidarNavigator(max_speed=AUTONOMOUS_MAX_SPEED)
        
        # Initialize dashboard if enabled
        self.dashboard = Dashboard(self) if self.dashboard_enabled else None
        
        # Set error callbacks for controllers if dashboard is enabled
        if self.dashboard is not None:
            def error_callback(msg: str):
                self.dashboard.add_error(msg)
            
            if self.motor is not None:
                self.motor.error_callback = error_callback
            if self.pantilt is not None:
                self.pantilt.error_callback = error_callback
    
    def _print_init_status(self):
        """Print init status for each module (only when dashboard is disabled)."""
        if self.dashboard_enabled:
            return  # Skip prints when dashboard is enabled - status shown in dashboard
        
        # Motor
        if not self.use_motor or self.motor is None:
            status = "Disabled" if not self.use_motor else "Failed ({})".format(
                self._init_errors.get("motor", "init error"))
            print("[ManualDrive] Motor: {}".format(status))
        else:
            print("[ManualDrive] Motor: OK")
        # Joystick
        if not self.use_joystick or self.joystick is None:
            status = "Disabled" if not self.use_joystick else "Failed ({})".format(
                self._init_errors.get("joystick", "init error"))
            print("[ManualDrive] Joystick: {}".format(status))
        else:
            print("[ManualDrive] Joystick: OK ({})".format(self.joystick.get_name()))
        # Pan/Tilt
        if not self.use_pan_tilt or self.pantilt is None:
            status = "Disabled" if not self.use_pan_tilt else "Failed ({})".format(
                self._init_errors.get("pan_tilt", "init error"))
            print("[ManualDrive] Pan/Tilt: {}".format(status))
        else:
            print("[ManualDrive] Pan/Tilt: OK")
        # Lidar
        if not self.use_lidar:
            print("[ManualDrive] Lidar: Disabled")
        elif self.lidar is None:
            print("[ManualDrive] Lidar: Failed ({})".format(
                self._init_errors.get("lidar", "port {} unavailable".format(self.lidar_port or "?"))))
        else:
            print("[ManualDrive] Lidar: OK ({})".format(self.lidar_port))
        # Camera
        if not self.use_camera:
            print("[ManualDrive] Camera: Disabled")
        elif self.camera is None:
            print("[ManualDrive] Camera: Failed ({})".format(
                self._init_errors.get("camera", "init error")))
        elif self.camera.is_available():
            print("[ManualDrive] Camera: OK")
        else:
            print("[ManualDrive] Camera: Failed (not available)")

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
    
    def _handle_mode_toggle(self):
        """
        Handle mode toggle with button B (front montant detection).
        Toggles between manual and autonomous mode.
        """
        if not self.use_joystick or self.joystick is None:
            return
        
        b_pressed = self.joystick.get_button(Input.B)
        
        # Detect front montant (button just pressed)
        if b_pressed and not self.b_button_pressed:
            # Toggle mode only if lidar is available
            if self.use_lidar and self.lidar is not None and self.lidar_navigator is not None:
                self.autonomous_mode = not self.autonomous_mode
            elif self.autonomous_mode:
                # If trying to enable autonomous but lidar not available, switch back to manual
                self.autonomous_mode = False
                if self.dashboard is not None:
                    self.dashboard.add_error("[ManualDrive] Cannot enable autonomous mode: lidar not available. Switching to MANUAL.")
                elif not self.dashboard_enabled:
                    print("\n[ManualDrive] Cannot enable autonomous mode: lidar not available. Switching to MANUAL.")
        
        self.b_button_pressed = b_pressed
    
    def _process_camera(self):
        """
        Process camera frame if camera is enabled.
        
        Returns:
            numpy.ndarray or None: Frame if available, None otherwise.
        """
        if self.camera is not None and self.camera.is_available():
            return self.camera.get_frame()
        return None
    
    def _initialize_sensors(self):
        """
        Initialize camera and lidar in the main process.
        """
        def log_error(message: str):
            if self.dashboard is not None:
                self.dashboard.add_error(message)
            elif not self.dashboard_enabled:
                print(message)
        
        if self.use_camera and self.camera is not None:
            try:
                self.camera.initialize()
            except Exception as e:
                error_msg = f"[Camera] Init failed: {e}"
                self._init_errors["camera"] = str(e)
                log_error(error_msg)
                self.camera = None
                if self.data_publisher is not None:
                    self.data_publisher.camera_controller = None
        if self.use_lidar and self.lidar is not None:
            try:
                if not self.lidar.initialize():
                    error_msg = "[Lidar] Init returned False"
                    self._init_errors["lidar"] = "init returned False"
                    log_error(error_msg)
                    self.lidar = None
                    if self.data_publisher is not None:
                        self.data_publisher.lidar_controller = None
            except Exception as e:
                error_msg = f"[Lidar] Init failed: {e}"
                self._init_errors["lidar"] = str(e)
                log_error(error_msg)
                self.lidar = None
                if self.data_publisher is not None:
                    self.data_publisher.lidar_controller = None
    
    def run(self):
        try:
            # Start dashboard BEFORE initialization if enabled
            if self.dashboard is not None:
                self.dashboard.start()
                # Small delay to let dashboard initialize
                time.sleep(0.2)
                self.dashboard.set_initializing(True)
            
            # Calculate total initialization steps for progress tracking
            total_steps = 0
            if self.use_camera or self.use_lidar:
                total_steps += 1  # Sensors initialization
            if self.enable_socket:
                total_steps += 1  # Socket server
            if self.use_motor:
                total_steps += 1  # Motor
            if self.use_joystick:
                total_steps += 1  # Joystick
            if self.use_pan_tilt:
                total_steps += 1  # Pan/Tilt
            if self.use_lidar and (self.lidar is None or not self.lidar.is_available()):
                total_steps += 1  # Lidar (if not already initialized)
            total_steps += 1  # Finalization
            
            current_step = 0
            
            def update_progress(step_name: str):
                nonlocal current_step
                current_step += 1
                if self.dashboard is not None:
                    progress = current_step / total_steps if total_steps > 0 else 1.0
                    self.dashboard.update_progress(step_name, progress)
            
            def log_error(message: str):
                if self.dashboard is not None:
                    self.dashboard.add_error(message)
                elif not self.dashboard_enabled:
                    print(message)
            
            # Initialize sensors (camera and lidar) if enabled
            # Initialize even if socket is not enabled (e.g., for dashboard display)
            if self.use_camera or self.use_lidar:
                update_progress("Initialisation des capteurs...")
                self._initialize_sensors()
            
            # If camera is enabled and socket is enabled, verify camera is ready before starting DataPublisher
            # This prevents the DataPublisher thread from accessing the camera before it's fully initialized
            # which can cause crashes on Jetson due to resource contention
            camera_ready = False
            if self.enable_socket and self.use_camera and self.camera is not None:
                if self.camera.is_available():
                    # Add delay to allow camera pipeline to stabilize (especially important on Jetson)
                    time.sleep(CAMERA_STARTUP_DELAY)
                    
                    # Verify camera can actually produce frames before starting DataPublisher
                    camera_ready = self.camera.verify_ready()
                    if not camera_ready:
                        log_error("[Camera] Warning: Camera initialization succeeded but cannot read frames. DataPublisher will start but may not receive camera data.")
                        # Don't prevent DataPublisher from starting, but it may not get camera data
                        camera_ready = True  # Allow DataPublisher to start anyway
                else:
                    log_error("[ManualDrive] Warning: Camera initialization failed. DataPublisher will start without camera.")
            
            # Start socket server if enabled
            if self.enable_socket and self.socket_server is not None:
                update_progress("Démarrage du serveur socket...")
                try:
                    self.socket_server.start()
                    # Only start DataPublisher if camera is ready (or if camera is not enabled)
                    if self.data_publisher is not None:
                        # If camera failed to initialize, remove it from DataPublisher
                        if self.use_camera and (self.camera is None or not camera_ready):
                            if self.camera is None:
                                self.data_publisher.camera_controller = None
                                log_error("[ManualDrive] Camera not available, DataPublisher will run without camera data")
                        self.data_publisher.start()
                except Exception as e:
                    log_error(f"[ManualDrive] Warning: Failed to start socket server: {e}")
            
            # Initialiser uniquement les contrôleurs du processus parent (VESC et pan/tilt)
            if self.use_motor and self.motor is not None:
                update_progress("Initialisation du moteur...")
                try:
                    self.motor.initialize()
                except Exception as e:
                    error_msg = f"[Motor] Init failed: {e}"
                    self._init_errors["motor"] = str(e)
                    log_error(error_msg)
                    self.motor = None
                    self.use_motor = False
            else:
                self.motor = None
                self.use_motor = False

            if self.use_joystick and self.joystick is not None:
                update_progress("Initialisation du joystick...")
                try:
                    self.joystick.initialize()
                except Exception as e:
                    error_msg = f"[Joystick] Init failed: {e}"
                    self._init_errors["joystick"] = str(e)
                    log_error(error_msg)
                    self.joystick = None
                    self.use_joystick = False
            else:
                self.joystick = None
                self.use_joystick = False

            if self.use_pan_tilt and self.pantilt is not None:
                update_progress("Initialisation du pan/tilt...")
                try:
                    self.pantilt.initialize()
                except Exception as e:
                    error_msg = f"[PanTilt] Init failed: {e}"
                    self._init_errors["pan_tilt"] = str(e)
                    log_error(error_msg)
                    self.pantilt = None
                    self.use_pan_tilt = False
            else:
                self.pantilt = None
                self.use_pan_tilt = False
            
            # Initialize pygame video/event system if not already done (e.g. when joystick disabled)
            if not self.use_joystick or self.joystick is None:
                pygame.init()
            
            # Initialize lidar: either in _initialize_sensors (when enable_socket) or here
            if self.use_lidar and self.lidar is not None and not self.lidar.is_available():
                update_progress("Initialisation du lidar...")
                try:
                    if not self.lidar.initialize():
                        error_msg = "[Lidar] Init returned False"
                        self._init_errors["lidar"] = "init returned False"
                        log_error(error_msg)
                        self.lidar = None
                        if self.data_publisher is not None:
                            self.data_publisher.lidar_controller = None
                except Exception as e:
                    error_msg = f"[Lidar] Init failed: {e}"
                    self._init_errors["lidar"] = str(e)
                    log_error(error_msg)
                    self.lidar = None
                    if self.data_publisher is not None:
                        self.data_publisher.lidar_controller = None
            elif self.use_lidar and self.lidar is None and self.lidar_port is not None:
                update_progress("Initialisation du lidar...")
                self.lidar = LidarController(serial_port=self.lidar_port, debug=self.debug_lidar)
                try:
                    if not self.lidar.initialize():
                        error_msg = "[Lidar] Init returned False"
                        self._init_errors["lidar"] = "init returned False"
                        log_error(error_msg)
                        self.lidar = None
                except Exception as e:
                    error_msg = f"[Lidar] Init failed: {e}"
                    self._init_errors["lidar"] = str(e)
                    log_error(error_msg)
                    self.lidar = None

            # Finalize initialization
            update_progress("Finalisation...")
            
            # Print init status only if dashboard is disabled
            self._print_init_status()
            
            # Mark initialization as complete
            if self.dashboard is not None:
                self.dashboard.set_initializing(False)

            last_status_print = 0.0

            while self.running:
                # Process pygame events (needed for joystick input when enabled)
                pygame.event.pump()

                if self.use_joystick and self.joystick is not None:
                    self.joystick.update()

                    if self._handle_exit():
                        break
                    
                    # Handle mode toggle with button B
                    self._handle_mode_toggle()

                acceleration = 0.0
                steering = 0.0
                lidar_scan = None
                should_send_motor_commands = True

                # Get lidar scan for dashboard (even in manual mode)
                if self.lidar is not None and self.lidar.is_available():
                    lidar_scan = self.lidar.get_scan()

                # Autonomous mode: use lidar navigation
                if self.autonomous_mode and self.lidar is not None and self.lidar_navigator is not None:
                    if lidar_scan is not None and len(lidar_scan) > 0:
                        acceleration, steering = self.lidar_navigator.compute_commands(lidar_scan)
                        should_send_motor_commands = True
                    else:
                        # No lidar data: don't call navigation and don't send commands to motor
                        acceleration = 0.0
                        steering = 0.0
                        should_send_motor_commands = False
                
                # Manual mode: use joystick
                else:
                    if self.use_throttle and self.throttle is not None and self.use_joystick and self.joystick is not None:
                        rt_value = self.joystick.get_rt()
                        lt_value = self.joystick.get_lt()
                        target = self.throttle.compute_target(rt_value, lt_value)
                        self.throttle.update(target)
                        acceleration = self.throttle.get_acceleration()

                    if self.use_joystick and self.joystick is not None:
                        steering = self.joystick.get_steering()

                if self.use_motor and self.motor is not None and should_send_motor_commands:
                    self.motor.set_commands(acceleration, steering)

                if self.use_pan_tilt and self.pantilt is not None and self.use_joystick and self.joystick is not None:
                    hat_x = self.joystick.get_axis(Input.HAT_X)
                    hat_y = self.joystick.get_axis(Input.HAT_Y)

                    if hat_x != 0 or hat_y != 0:
                        self.pantilt.update(hat_x, hat_y)
                    else:
                        pan_axis = self.joystick.get_axis(Axis.RIGHT_JOY_X)
                        tilt_axis = -self.joystick.get_axis(Axis.RIGHT_JOY_Y)
                        self.pantilt.set_analog_position(pan_axis, tilt_axis)

                # Update dashboard with current values
                if self.dashboard is not None:
                    # Collect joystick values
                    rt_value = 0.0
                    lt_value = 0.0
                    joystick_steering = 0.0
                    pan_axis = 0.0
                    tilt_axis = 0.0
                    
                    if self.use_joystick and self.joystick is not None:
                        rt_value = self.joystick.get_rt()
                        lt_value = self.joystick.get_lt()
                        joystick_steering = self.joystick.get_steering()
                        pan_axis = self.joystick.get_axis(Axis.RIGHT_JOY_X)
                        tilt_axis = -self.joystick.get_axis(Axis.RIGHT_JOY_Y)
                    
                    # Collect lidar info (use lidar_scan already retrieved above)
                    lidar_points = 0
                    lidar_min_angle = None
                    lidar_max_angle = None
                    if lidar_scan is not None and len(lidar_scan) > 0:
                        lidar_points = len(lidar_scan)
                        angles = [p.get("angle", 0) for p in lidar_scan if "angle" in p]
                        if angles:
                            lidar_min_angle = min(angles)
                            lidar_max_angle = max(angles)
                    
                    mode_str = "AUTONOMOUS" if self.autonomous_mode else "MANUAL"
                    
                    # Get pan/tilt values even if pan/tilt is disabled
                    pan_speed = 0.0
                    tilt_pos = 0.0
                    if self.pantilt is not None and hasattr(self.pantilt, 'get_position'):
                        pan_speed, tilt_pos = self.pantilt.get_position()
                    
                    self.dashboard.update_data(
                        acceleration=acceleration,
                        steering=steering,
                        mode=mode_str,
                        joystick_rt=rt_value,
                        joystick_lt=lt_value,
                        joystick_steering=joystick_steering,
                        joystick_pan_axis=pan_axis,
                        joystick_tilt_axis=tilt_axis,
                        lidar_points=lidar_points,
                        lidar_min_angle=lidar_min_angle,
                        lidar_max_angle=lidar_max_angle,
                        pan_speed=pan_speed,
                        tilt_position=tilt_pos,
                    )

                # Debug joystick output removed - displayed in dashboard instead

                time.sleep(LOOP_SLEEP_TIME)

        except KeyboardInterrupt:
            pass  # Handled gracefully
        except Exception as e:
            if self.dashboard is not None:
                self.dashboard.add_error(f"[ManualDrive] Error: {e}")
            elif not self.dashboard_enabled:
                print(f"[ManualDrive] Error: {e}")
            import traceback
            if not self.dashboard_enabled:
                traceback.print_exc()
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources."""
        # Stop dashboard if enabled
        if self.dashboard is not None:
            self.dashboard.stop()
        
        # Stop sensors, data publisher, and socket server
        if self.data_publisher is not None:
            self.data_publisher.stop()
        if self.socket_server is not None:
            self.socket_server.stop()
        if self.lidar is not None:
            self.lidar.stop()
        if self.camera is not None:
            self.camera.stop()
            time.sleep(0.5)  # DepthAI cleanup
        
        # Stop motor (only if enabled)
        if self.use_motor and self.motor is not None:
            self.motor.stop()
        
        # Stop pan/tilt
        if self.use_pan_tilt and self.pantilt is not None:
            self.pantilt.stop()
