"""
Dashboard terminal pour afficher l'état des modules et les valeurs en temps réel.
"""

import time
import threading
from typing import Optional
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.layout import Layout
from rich.text import Text


class Dashboard:
    """
    Dashboard terminal pour afficher l'état des modules et les valeurs en temps réel.
    Utilise rich pour créer une interface moderne dans le terminal.
    """
    
    def __init__(self, app):
        """
        Initialise le dashboard.
        
        Args:
            app: Instance de ManualDriveApp avec références aux contrôleurs.
        """
        self.app = app
        self.console = Console()
        self.running = False
        self.update_thread = None
        self.update_rate = 0.1  # 10 Hz
        
        # Données en temps réel (mises à jour depuis la boucle principale)
        self.current_data = {
            'acceleration': 0.0,
            'steering': 0.0,
            'mode': 'MANUAL',
            'joystick_rt': 0.0,
            'joystick_lt': 0.0,
            'joystick_steering': 0.0,
            'joystick_pan_axis': 0.0,
            'joystick_tilt_axis': 0.0,
            'lidar_points': 0,
            'lidar_min_angle': None,
            'lidar_max_angle': None,
            'socket_clients': 0,
        }
        self._data_lock = threading.Lock()
    
    def update_data(self, **kwargs):
        """
        Met à jour les données du dashboard depuis la boucle principale.
        
        Args:
            **kwargs: Données à mettre à jour (acceleration, steering, mode, etc.)
        """
        with self._data_lock:
            self.current_data.update(kwargs)
    
    def _get_module_status(self, module_name, use_flag, controller, init_error_key=None):
        """
        Obtient le statut d'un module.
        
        Returns:
            tuple: (status_text, status_color) où status_text est "OK", "Disabled", ou "Failed"
        """
        if not use_flag:
            return ("Disabled", "yellow")
        
        if controller is None:
            error_msg = ""
            if init_error_key and hasattr(self.app, '_init_errors'):
                error_msg = self.app._init_errors.get(init_error_key, "")
            if error_msg:
                return ("Failed", "red")
            return ("Failed", "red")
        
        # Vérifier si le contrôleur est initialisé/disponible
        if hasattr(controller, 'is_available'):
            if controller.is_available():
                return ("OK", "green")
            else:
                return ("Failed", "red")
        elif hasattr(controller, '_initialized'):
            if controller._initialized:
                return ("OK", "green")
            else:
                return ("Failed", "red")
        
        return ("OK", "green")
    
    def _create_modules_table(self):
        """Crée le tableau d'état des modules."""
        table = Table(show_header=True, header_style="bold magenta", box=None)
        table.add_column("Module", style="cyan", width=12)
        table.add_column("Status", width=10)
        
        # Motor
        status, color = self._get_module_status("Motor", self.app.use_motor, self.app.motor, "motor")
        table.add_row("Motor", Text(status, style=color))
        
        # Joystick
        status, color = self._get_module_status("Joystick", self.app.use_joystick, self.app.joystick, "joystick")
        joystick_name = ""
        if self.app.joystick and hasattr(self.app.joystick, 'get_name'):
            joystick_name = f" ({self.app.joystick.get_name()})"
        table.add_row("Joystick", Text(status + joystick_name, style=color))
        
        # Throttle
        status, color = self._get_module_status("Throttle", self.app.use_throttle, self.app.throttle)
        table.add_row("Throttle", Text(status, style=color))
        
        # Camera
        status, color = self._get_module_status("Camera", self.app.use_camera, self.app.camera, "camera")
        camera_info = ""
        if self.app.camera and hasattr(self.app.camera, 'is_available') and self.app.camera.is_available():
            camera_info = f" ({self.app.camera.width}x{self.app.camera.height})"
        table.add_row("Camera", Text(status + camera_info, style=color))
        
        # PanTilt
        status, color = self._get_module_status("PanTilt", self.app.use_pan_tilt, self.app.pantilt, "pan_tilt")
        table.add_row("Pan/Tilt", Text(status, style=color))
        
        # Lidar
        status, color = self._get_module_status("Lidar", self.app.use_lidar, self.app.lidar, "lidar")
        lidar_info = ""
        if self.app.lidar_port:
            lidar_info = f" ({self.app.lidar_port})"
        table.add_row("Lidar", Text(status + lidar_info, style=color))
        
        # Socket
        socket_status = "Disabled"
        socket_color = "yellow"
        if self.app.enable_socket:
            if self.app.socket_server and self.app.socket_server.is_running():
                socket_status = "Running"
                socket_color = "green"
            else:
                socket_status = "Failed"
                socket_color = "red"
        socket_info = ""
        if self.app.socket_server:
            socket_info = f" (:{self.app.socket_server.port})"
        table.add_row("Socket", Text(socket_status + socket_info, style=socket_color))
        
        return table
    
    def _create_controls_table(self):
        """Crée le tableau des valeurs de contrôle."""
        table = Table(show_header=True, header_style="bold blue", box=None)
        table.add_column("Control", style="cyan", width=15)
        table.add_column("Value", width=15)
        
        with self._data_lock:
            mode = self.current_data.get('mode', 'MANUAL')
            acceleration = self.current_data.get('acceleration', 0.0)
            steering = self.current_data.get('steering', 0.0)
        
        table.add_row("Mode", Text(mode, style="bold"))
        table.add_row("Acceleration", f"{acceleration:.3f}")
        table.add_row("Steering", f"{steering:.3f}")
        
        return table
    
    def _create_joystick_table(self):
        """Crée le tableau des valeurs du joystick."""
        table = Table(show_header=True, header_style="bold green", box=None)
        table.add_column("Axis", style="cyan", width=15)
        table.add_column("Value", width=15)
        
        with self._data_lock:
            rt = self.current_data.get('joystick_rt', 0.0)
            lt = self.current_data.get('joystick_lt', 0.0)
            steering = self.current_data.get('joystick_steering', 0.0)
            pan_axis = self.current_data.get('joystick_pan_axis', 0.0)
            tilt_axis = self.current_data.get('joystick_tilt_axis', 0.0)
        
        table.add_row("RT (Throttle)", f"{rt:.3f}")
        table.add_row("LT (Brake)", f"{lt:.3f}")
        table.add_row("Steering", f"{steering:.3f}")
        table.add_row("Pan Axis", f"{pan_axis:.3f}")
        table.add_row("Tilt Axis", f"{tilt_axis:.3f}")
        
        return table
    
    def _create_sensors_table(self):
        """Crée le tableau des valeurs des capteurs."""
        table = Table(show_header=True, header_style="bold yellow", box=None)
        table.add_column("Sensor", style="cyan", width=15)
        table.add_column("Value", width=20)
        
        # PanTilt
        pan_speed = "N/A"
        tilt_pos = "N/A"
        if self.app.pantilt and hasattr(self.app.pantilt, 'get_position'):
            pan, tilt = self.app.pantilt.get_position()
            pan_speed = f"{pan:.3f}"
            tilt_pos = f"{tilt:.3f}"
        table.add_row("Pan Speed", pan_speed)
        table.add_row("Tilt Position", tilt_pos)
        
        # Lidar
        with self._data_lock:
            lidar_points = self.current_data.get('lidar_points', 0)
            lidar_min_angle = self.current_data.get('lidar_min_angle')
            lidar_max_angle = self.current_data.get('lidar_max_angle')
        
        lidar_info = f"{lidar_points} points"
        if lidar_min_angle is not None and lidar_max_angle is not None:
            lidar_info += f" ({lidar_min_angle:.1f}°-{lidar_max_angle:.1f}°)"
        table.add_row("Lidar Scan", lidar_info)
        
        # Socket clients
        with self._data_lock:
            socket_clients = self.current_data.get('socket_clients', 0)
        table.add_row("Socket Clients", str(socket_clients))
        
        # Socket emission info
        if self.app.socket_server and self.app.socket_server.is_running():
            emission_info = self.app.socket_server.get_last_emission_info()
            last_sensor_time = emission_info.get('last_sensor_data_time')
            last_status_time = emission_info.get('last_status_time')
            sensor_size = emission_info.get('last_sensor_data_size')
            
            if last_sensor_time:
                import time
                age = time.time() - last_sensor_time
                sensor_info = f"{age:.1f}s ago"
                if sensor_size:
                    sensor_info += f" ({sensor_size} bytes)"
            else:
                sensor_info = "Never"
            table.add_row("Last Sensor Data", sensor_info)
            
            if last_status_time:
                import time
                age = time.time() - last_status_time
                status_info = f"{age:.1f}s ago"
            else:
                status_info = "Never"
            table.add_row("Last Status", status_info)
        
        return table
    
    def _create_layout(self):
        """Crée la mise en page du dashboard."""
        layout = Layout()
        
        # Layout principal: 2 colonnes
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main")
        )
        
        # Header avec titre
        header_text = Text("ROBOCAR DASHBOARD", style="bold white on blue", justify="center")
        layout["header"].update(Panel(header_text, border_style="blue"))
        
        # Main layout: 2 colonnes
        layout["main"].split_row(
            Layout(name="left"),
            Layout(name="right")
        )
        
        # Colonne gauche: Modules et Controls
        layout["left"].split_column(
            Layout(Panel(self._create_modules_table(), title="Modules Status", border_style="magenta"), name="modules"),
            Layout(Panel(self._create_controls_table(), title="Controls", border_style="blue"), name="controls")
        )
        
        # Colonne droite: Joystick et Sensors
        layout["right"].split_column(
            Layout(Panel(self._create_joystick_table(), title="Joystick", border_style="green"), name="joystick"),
            Layout(Panel(self._create_sensors_table(), title="Sensors", border_style="yellow"), name="sensors")
        )
        
        return layout
    
    def _update_loop(self):
        """Boucle de mise à jour du dashboard."""
        try:
            with Live(self._create_layout(), refresh_per_second=10, screen=False) as live:
                while self.running:
                    # Mettre à jour les données du socket
                    if self.app.socket_server:
                        with self._data_lock:
                            self.current_data['socket_clients'] = self.app.socket_server.get_client_count()
                            # Les informations d'émission sont récupérées directement dans _create_sensors_table
                    
                    # Mettre à jour l'affichage
                    live.update(self._create_layout())
                    time.sleep(self.update_rate)
        except Exception as e:
            self.console.print(f"[red]Dashboard error: {e}[/red]")
    
    def start(self):
        """Démarre le dashboard dans un thread séparé."""
        if self.running:
            return
        
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
    
    def stop(self):
        """Arrête le dashboard."""
        if not self.running:
            return
        
        self.running = False
        if self.update_thread is not None:
            self.update_thread.join(timeout=1.0)
