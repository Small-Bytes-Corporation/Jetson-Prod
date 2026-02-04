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
    DEFAULT_LIDAR_PORT, SOCKETIO_PORT, PAN_TILT_SERIAL_PORT, DEFAULT_RTK_SERIAL_PORT,
    RTK_TCP_DEFAULT_PORT,
)


class ManualDriveApp:
    """
    Application for manual driving with joystick control.
    """
    
    def __init__(self, max_speed=MAX_SPEED, serial_port=DEFAULT_SERIAL_PORT, 
                 use_camera=False, enable_socket=False, lidar_port=None, socket_port=SOCKETIO_PORT,
                 socket_debug=False,
                 use_motor=True, pan_tilt_port=None, use_pan_tilt=True,
                 use_joystick=True, use_throttle=True, use_lidar=None,
                 rtk_port=None, use_rtk=True, rtk_tcp_host=None, rtk_tcp_port=None):
        """
        Initialize the manual drive application.
        
        Args:
            max_speed: Maximum speed for forward/backward movement.
            serial_port: Serial port for VESC motor.
            use_camera: Whether to use camera (optional, for display/recording).
            enable_socket: Whether to enable socket.io server for data streaming.
            lidar_port: Serial port for lidar (e.g., '/dev/ttyUSB0'). If None, lidar is disabled.
            socket_port: Port for socket.io server.
            socket_debug: If True, print full socket payloads (sensor_data, status) to console.
            use_motor: Whether to enable motor/VESC. If False, motor is disabled (mock mode).
            pan_tilt_port: Serial port for pan/tilt controller. If None, uses default from config.
            use_pan_tilt: Whether to enable pan/tilt control. If False, pan/tilt is disabled.
            use_joystick: Whether to enable joystick controller. If False, joystick is disabled.
            use_throttle: Whether to enable throttle controller. If False, throttle is disabled.
            use_lidar: Whether to enable lidar. If None, auto-enabled if lidar_port is provided.
            rtk_port: Serial port for RTK GNSS. If None, uses default from config.
            use_rtk: Whether to enable RTK GNSS (pose/IMU). If False, RTK is disabled.
            rtk_tcp_host: TCP host for RTK (e.g. 'localhost') when using p1-runner --tcp. Enables valid RTK fix.
            rtk_tcp_port: TCP port for RTK when rtk_tcp_host is set (default 30201).
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
        
        # Determine if we'll use fork for sensors.
        # Camera must NOT run in forked process: DepthAI/XLink fails with X_LINK_DEVICE_NOT_FOUND
        # when initialized after fork. So we only fork when we have lidar/RTK but no camera.
        self.will_fork_sensors = enable_socket and (use_lidar or use_rtk) and not use_camera
        
        # Initialize controllers based on flags
        # Note: Sensors (camera, lidar, RTK) and socket will be created in child process if forking
        self.motor = MotorController(serial_port=serial_port, enabled=use_motor) if use_motor else None
        self.joystick = JoystickController() if use_joystick else None
        self.throttle = ThrottleController(max_speed=max_speed) if use_throttle else None
        self.pantilt = PanTiltController(
            serial_port=pan_tilt_port or PAN_TILT_SERIAL_PORT,
            enabled=use_pan_tilt
        ) if use_pan_tilt else None
        
        # Sensors and socket only created if NOT forking (legacy mode)
        if not self.will_fork_sensors:
            self.camera = CameraController() if use_camera else None
            self.lidar = LidarController(serial_port=lidar_port) if self.use_lidar and lidar_port else None
            self.rtk = RTKController(
                serial_port=rtk_port or DEFAULT_RTK_SERIAL_PORT,
                enabled=use_rtk,
                tcp_host=rtk_tcp_host,
                tcp_port=rtk_tcp_port or (RTK_TCP_DEFAULT_PORT if rtk_tcp_host else None)
            ) if use_rtk else None
            
            # Socket.io components
            self.socket_server = None
            self.data_publisher = None
            
            if enable_socket:
                self.socket_server = SocketServer(port=socket_port, debug_payload=socket_debug)
                self.data_publisher = DataPublisher(
                    lidar_controller=self.lidar,
                    camera_controller=self.camera,
                    rtk_controller=self.rtk,
                    socket_server=self.socket_server
                )
        else:
            # Sensors will be created in child process
            self.camera = None
            self.lidar = None
            self.rtk = None
            self.socket_server = None
            self.data_publisher = None
        
        self.running = True
        self.sensor_process_pid = None  # PID du processus enfant pour les capteurs
        
        # Store socket config for sensor process
        self.socket_port = socket_port
        self.socket_debug = socket_debug
        self.rtk_port = rtk_port
        self.rtk_tcp_host = rtk_tcp_host
        self.rtk_tcp_port = rtk_tcp_port or RTK_TCP_DEFAULT_PORT
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, sig, frame):
        """Handle shutdown signals in parent process."""
        print(f"\n[ManualDrive] Signal {sig} received, exiting...")
        self.running = False
        
        # Tuer le processus enfant si il existe encore
        if self.sensor_process_pid is not None:
            try:
                # Vérifier si le processus existe encore
                os.kill(self.sensor_process_pid, 0)  # Ne tue pas, juste vérifie
                print(f"[ManualDrive] Sending SIGTERM to sensor process (PID {self.sensor_process_pid})...")
                os.kill(self.sensor_process_pid, signal.SIGTERM)
            except ProcessLookupError:
                # Le processus n'existe plus, essayer de le récolter pour éviter un zombie
                try:
                    os.waitpid(self.sensor_process_pid, os.WNOHANG)
                except (ChildProcessError, ProcessLookupError):
                    pass
            except OSError:
                # Le processus n'existe plus ou erreur d'accès, essayer de le récolter
                try:
                    os.waitpid(self.sensor_process_pid, os.WNOHANG)
                except (ChildProcessError, ProcessLookupError):
                    pass
    
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
    
    def _initialize_sensors_main_process(self):
        """
        Initialize camera, lidar, RTK in the main process (no fork).
        Used when camera is enabled, since DepthAI fails with X_LINK_DEVICE_NOT_FOUND
        when initialized in a forked child process.
        """
        print("[ManualDrive] Initializing sensors in main process...")
        if self.use_camera and self.camera is not None:
            try:
                self.camera.initialize()
                print("[ManualDrive] Camera initialized successfully")
            except Exception as e:
                print(f"[ManualDrive] Warning: Failed to initialize camera: {e}")
                self.camera = None
                if self.data_publisher is not None:
                    self.data_publisher.camera_controller = None
        if self.use_lidar and self.lidar is not None:
            try:
                if not self.lidar.initialize():
                    print("[ManualDrive] Warning: Lidar initialization failed")
                    self.lidar = None
                    if self.data_publisher is not None:
                        self.data_publisher.lidar_controller = None
                else:
                    print("[ManualDrive] Lidar initialized successfully")
            except Exception as e:
                print(f"[ManualDrive] Warning: Failed to initialize lidar: {e}")
                self.lidar = None
                if self.data_publisher is not None:
                    self.data_publisher.lidar_controller = None
        if self.use_rtk and self.rtk is not None:
            try:
                if not self.rtk.initialize():
                    print("[ManualDrive] Warning: RTK initialization failed")
                    self.rtk = None
                    if self.data_publisher is not None:
                        self.data_publisher.rtk_controller = None
                else:
                    print("[ManualDrive] RTK initialized successfully")
            except Exception as e:
                print(f"[ManualDrive] Warning: Failed to initialize RTK: {e}")
                self.rtk = None
                if self.data_publisher is not None:
                    self.data_publisher.rtk_controller = None
    
    def _run_sensor_process(self):
        """
        Fonction exécutée dans le processus enfant pour gérer uniquement les capteurs.
        Initialise les capteurs (camera, lidar, RTK) et le serveur socket.io,
        puis lance la boucle de publication des données.
        """
        # Réinitialiser les handlers de signaux pour le processus enfant
        running = [True]  # Utiliser une liste pour permettre la modification dans le handler
        
        def sensor_signal_handler(sig, frame):
            """Handle shutdown signals in sensor process."""
            print(f"\n[SensorProcess] Signal {sig} received, exiting...")
            running[0] = False
        
        signal.signal(signal.SIGINT, sensor_signal_handler)
        signal.signal(signal.SIGTERM, sensor_signal_handler)
        
        print("[SensorProcess] Initializing sensors...")
        sys.stdout.flush()  # Forcer l'affichage immédiat
        
        # Initialiser les contrôleurs de capteurs
        print("[SensorProcess] Creating sensor controllers...")
        sys.stdout.flush()
        camera = CameraController() if self.use_camera else None
        lidar = LidarController(serial_port=self.lidar_port) if self.use_lidar and self.lidar_port else None
        rtk = RTKController(
            serial_port=self.rtk_port or DEFAULT_RTK_SERIAL_PORT,
            enabled=self.use_rtk,
            tcp_host=self.rtk_tcp_host,
            tcp_port=self.rtk_tcp_port if self.rtk_tcp_host else None
        ) if self.use_rtk else None
        
        # Initialiser le socket server et data publisher
        socket_server = None
        data_publisher = None
        
        if self.enable_socket:
            print("[SensorProcess] Creating socket server and data publisher...")
            socket_server = SocketServer(port=self.socket_port, debug_payload=self.socket_debug)
            data_publisher = DataPublisher(
                lidar_controller=lidar,
                camera_controller=camera,
                rtk_controller=rtk,
                socket_server=socket_server
            )
        
        # Initialiser les capteurs
        if self.use_camera and camera is not None:
            print("[SensorProcess] Initializing camera...")
            sys.stdout.flush()
            try:
                camera.initialize()
                print("[SensorProcess] Camera initialized successfully")
                sys.stdout.flush()
            except Exception as e:
                print(f"[SensorProcess] Warning: Failed to initialize camera: {e}")
                import traceback
                traceback.print_exc()
                sys.stdout.flush()
                camera = None
        else:
            print("[SensorProcess] Camera not enabled or not available")
            sys.stdout.flush()
        
        if self.use_lidar and lidar is not None:
            print("[SensorProcess] Initializing lidar...")
            try:
                if not lidar.initialize():
                    print("[SensorProcess] Warning: Lidar initialization failed")
                    lidar = None
                else:
                    print("[SensorProcess] Lidar initialized successfully")
            except Exception as e:
                print(f"[SensorProcess] Warning: Failed to initialize lidar: {e}")
                lidar = None
        
        if self.use_rtk and rtk is not None:
            print("[SensorProcess] Initializing RTK...")
            try:
                if not rtk.initialize():
                    print("[SensorProcess] Warning: RTK initialization failed")
                    rtk = None
                else:
                    print("[SensorProcess] RTK initialized successfully")
            except Exception as e:
                print(f"[SensorProcess] Warning: Failed to initialize RTK: {e}")
                rtk = None
        
        # Démarrer le socket server et le data publisher
        if self.enable_socket and socket_server is not None:
            print("[SensorProcess] Starting socket server...")
            sys.stdout.flush()
            try:
                socket_server.start()
                print("[SensorProcess] Socket server started")
                sys.stdout.flush()
                if data_publisher is not None:
                    print("[SensorProcess] Starting data publisher...")
                    sys.stdout.flush()
                    data_publisher.start()
                    print("[SensorProcess] Data publisher started")
                    sys.stdout.flush()
                print(f"[SensorProcess] Socket.io server running on port {socket_server.port}")
                sys.stdout.flush()
            except Exception as e:
                print(f"[SensorProcess] Warning: Failed to start socket server: {e}")
                import traceback
                traceback.print_exc()
                sys.stdout.flush()
        
        print("[SensorProcess] Ready! Collecting and publishing sensor data...")
        sys.stdout.flush()
        
        # Boucle principale du processus capteurs
        try:
            while running[0]:
                # Mettre à jour RTK pour collecter les données
                if rtk is not None:
                    rtk.update()
                
                # Le DataPublisher s'occupe de collecter et publier les données
                # dans son propre thread, donc on attend juste ici
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n[SensorProcess] Keyboard interrupt received.")
        except Exception as e:
            print(f"[SensorProcess] Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Nettoyage des ressources du processus capteurs
            print("[SensorProcess] Cleaning up...")
            
            # Arrêter le data publisher en premier (il peut utiliser les capteurs)
            if data_publisher is not None:
                data_publisher.stop()
            
            # Arrêter le socket server
            if socket_server is not None:
                socket_server.stop()
            
            # Arrêter les capteurs dans l'ordre inverse de leur initialisation
            if rtk is not None:
                rtk.stop()
            
            if lidar is not None:
                lidar.stop()
            
            # Arrêter la caméra en dernier (DepthAI peut avoir des threads à nettoyer)
            if camera is not None:
                camera.stop()
                # Attendre un peu pour que DepthAI nettoie ses threads
                time.sleep(0.5)
            
            print("[SensorProcess] Shutdown complete.")
            # Utiliser os._exit() pour éviter les destructeurs Python qui peuvent causer des problèmes
            # avec les bibliothèques C++ comme DepthAI
            os._exit(0)
    
    def run(self):
        try:
            print("[ManualDrive] Initializing controllers...")
            
            # Créer le processus enfant pour les capteurs si nécessaire
            if self.will_fork_sensors:
                print("[ManualDrive] Forking sensor process...")
                child_pid = os.fork()
                
                if child_pid == 0:
                    # Processus enfant : exécuter la fonction des capteurs
                    self._run_sensor_process()
                    # _run_sensor_process() appelle sys.exit(0), donc on ne devrait jamais arriver ici
                    return
                else:
                    # Processus parent : sauvegarder le PID du processus enfant
                    self.sensor_process_pid = child_pid
                    print(f"[ManualDrive] Sensor process started with PID {child_pid}")
                    # Attendre un peu pour que le processus enfant s'initialise
                    time.sleep(1.0)
            elif self.enable_socket and self.socket_server is not None:
                # Mode sans fork (camera ou pas de lidar/rtk) : init capteurs dans le processus principal
                self._initialize_sensors_main_process()
                try:
                    self.socket_server.start()
                    if self.data_publisher is not None:
                        self.data_publisher.start()
                    print(f"[ManualDrive] Socket.io server running on port {self.socket_server.port}")
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to start socket server: {e}")
            
            # Initialiser uniquement les contrôleurs du processus parent (VESC et pan/tilt)
            if self.use_motor and self.motor is not None:
                try:
                    self.motor.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize motor: {e}")
                    self.motor = None
                    self.use_motor = False
            else:
                self.motor = None
                self.use_motor = False

            if self.use_joystick and self.joystick is not None:
                try:
                    self.joystick.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize joystick: {e}")
                    self.joystick = None
                    self.use_joystick = False
            else:
                self.joystick = None
                self.use_joystick = False

            if self.use_pan_tilt and self.pantilt is not None:
                try:
                    self.pantilt.initialize()
                except Exception as e:
                    print(f"[ManualDrive] Warning: Failed to initialize pan/tilt: {e}")
                    self.pantilt = None
                    self.use_pan_tilt = False
            else:
                self.pantilt = None
                self.use_pan_tilt = False
            
            # Initialize pygame video/event system if not already done (e.g. when joystick disabled)
            if not self.use_joystick or self.joystick is None:
                pygame.init()
            
            print("[ManualDrive] Ready! Use BACK+START to exit.")

            last_status_print = 0.0

            while self.running:
                # Process pygame events (needed for joystick input when enabled)
                pygame.event.pump()

                if self.use_joystick and self.joystick is not None:
                    self.joystick.update()

                    if self._handle_exit():
                        print("[ManualDrive] Exit requested (BACK+START).")
                        break

                acceleration = 0.0
                steering = 0.0

                if self.use_throttle and self.throttle is not None and self.use_joystick and self.joystick is not None:
                    rt_value = self.joystick.get_rt()
                    lt_value = self.joystick.get_lt()
                    target = self.throttle.compute_target(rt_value, lt_value)
                    self.throttle.update(target)
                    acceleration = self.throttle.get_acceleration()

                if self.use_joystick and self.joystick is not None:
                    steering = self.joystick.get_steering()

                if self.use_motor and self.motor is not None:
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

                # Update RTK when running in main process (no fork, e.g. when camera is used)
                if self.rtk is not None:
                    self.rtk.update()

                now = time.monotonic()
                if now - last_status_print > 0.2:
                    last_status_print = now
                    sys.stdout.write(f"\rDuty: {acceleration:.3f} | Steer: {(steering + 1) / 2:.3f}   ")
                    sys.stdout.flush()

                time.sleep(LOOP_SLEEP_TIME)

        except KeyboardInterrupt:
            print("\n[ManualDrive] Keyboard interrupt received.")
        except Exception as e:
            print(f"[ManualDrive] Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
            # S'assurer que le processus parent se termine
            # Réinitialiser les handlers de signaux pour éviter les boucles
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            signal.signal(signal.SIGTERM, signal.SIG_DFL)

    def cleanup(self):
        """Clean up resources."""
        print("[ManualDrive] Cleaning up...")
        
        # Tuer le processus enfant si il existe encore
        if self.sensor_process_pid is not None:
            try:
                # Vérifier si le processus existe encore
                os.kill(self.sensor_process_pid, 0)  # Ne tue pas, juste vérifie
                print(f"[ManualDrive] Terminating sensor process (PID {self.sensor_process_pid})...")
                os.kill(self.sensor_process_pid, signal.SIGTERM)
                
                # Attendre la fin du processus enfant (avec timeout)
                max_wait = 2.0  # Maximum 2 secondes d'attente
                waited = 0.0
                process_terminated = False
                
                while waited < max_wait:
                    try:
                        # Vérifier si le processus existe encore
                        os.kill(self.sensor_process_pid, 0)
                        # Le processus existe encore, essayer de le récolter
                        try:
                            pid, status = os.waitpid(self.sensor_process_pid, os.WNOHANG)
                            if pid != 0:
                                # Le processus s'est terminé
                                print(f"[ManualDrive] Sensor process terminated (status: {status})")
                                process_terminated = True
                                break
                        except ChildProcessError:
                            # Le processus est déjà terminé (récolté par un autre wait)
                            print(f"[ManualDrive] Sensor process already terminated")
                            process_terminated = True
                            break
                    except ProcessLookupError:
                        # Le processus n'existe plus
                        print(f"[ManualDrive] Sensor process not found")
                        process_terminated = True
                        break
                    except OSError:
                        # Erreur d'accès, le processus n'existe probablement plus
                        print(f"[ManualDrive] Sensor process access error, assuming terminated")
                        process_terminated = True
                        break
                    
                    time.sleep(0.1)
                    waited += 0.1
                
                # Si le processus n'est toujours pas terminé, forcer avec SIGKILL
                if not process_terminated:
                    try:
                        os.kill(self.sensor_process_pid, 0)  # Vérifier qu'il existe encore
                        print(f"[ManualDrive] Force killing sensor process...")
                        os.kill(self.sensor_process_pid, signal.SIGKILL)
                        # Attendre la fin forcée (non-bloquant avec timeout)
                        for _ in range(10):  # 1 seconde max
                            try:
                                pid, status = os.waitpid(self.sensor_process_pid, os.WNOHANG)
                                if pid != 0:
                                    break
                            except (ChildProcessError, ProcessLookupError):
                                break
                            time.sleep(0.1)
                    except (ProcessLookupError, OSError):
                        pass  # Déjà terminé
                        
            except ProcessLookupError:
                # Le processus n'existe plus, essayer de le récolter quand même
                try:
                    os.waitpid(self.sensor_process_pid, os.WNOHANG)
                except (ChildProcessError, ProcessLookupError):
                    pass  # Déjà récolté ou n'existe pas
            except OSError as e:
                # Le processus n'existe plus ou erreur d'accès
                # Essayer de le récolter quand même pour éviter un zombie
                try:
                    os.waitpid(self.sensor_process_pid, os.WNOHANG)
                except (ChildProcessError, ProcessLookupError):
                    pass
        
        # When not forking, sensors run in main process: stop data_publisher, socket, sensors
        if not self.will_fork_sensors:
            if self.data_publisher is not None:
                self.data_publisher.stop()
            if self.socket_server is not None:
                self.socket_server.stop()
            if self.rtk is not None:
                self.rtk.stop()
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
        
        print("[ManualDrive] Shutdown complete.")
        
        # S'assurer que le processus parent se termine correctement
        # Réinitialiser les handlers de signaux pour éviter les boucles
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
