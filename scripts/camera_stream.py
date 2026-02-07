#!/usr/bin/env python3
"""
Script minimal pour envoyer uniquement les données de la caméra via UDP.
Pas de contrôle de voiture, pas de joystick, pas de moteur, rien d'autre.

Usage: python3 scripts/camera_stream.py [options]
       ou depuis la racine: python3 -m scripts.camera_stream [options]
Quitter: Ctrl+C
"""

import sys
import os
import argparse
import signal
import time

# Ajouter le répertoire parent au path pour les imports
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sys.path.insert(0, project_root)

# Importer depuis le package drive.core pour que les imports relatifs fonctionnent
# On évite de charger __init__.py en créant un faux module pour drive.core
import importlib
import importlib.util

# Créer un faux module drive.core pour que les imports relatifs fonctionnent
# sans charger le vrai __init__.py qui importe tous les modules
core_dir = os.path.join(project_root, 'drive', 'core')

if 'drive' not in sys.modules:
    drive_module = type(sys)('drive')
    drive_module.__path__ = [os.path.join(project_root, 'drive')]
    sys.modules['drive'] = drive_module

if 'drive.core' not in sys.modules:
    core_module = type(sys)('drive.core')
    core_module.__path__ = [core_dir]
    core_module.__package__ = 'drive.core'
    sys.modules['drive.core'] = core_module

# Créer un __init__.py factice pour que Python reconnaisse drive.core comme un package
if 'drive.core.__init__' not in sys.modules:
    init_module = type(sys)('drive.core.__init__')
    init_module.__package__ = 'drive.core'
    sys.modules['drive.core.__init__'] = init_module

def _load_module(name, file_path):
    """Charge un module depuis un fichier en tant que module de drive.core."""
    full_name = f'drive.core.{name}'
    # Enregistrer le module dans sys.modules AVANT de l'exécuter
    # pour que les imports relatifs puissent le trouver
    spec = importlib.util.spec_from_file_location(full_name, file_path)
    module = importlib.util.module_from_spec(spec)
    module.__package__ = 'drive.core'
    module.__name__ = full_name
    # Enregistrer AVANT l'exécution pour que les imports relatifs fonctionnent
    sys.modules[full_name] = module
    spec.loader.exec_module(module)
    return module

# Charger config en premier car tous les autres modules en dépendent
# et s'assurer qu'il est bien enregistré dans sys.modules
config = _load_module('config', os.path.join(core_dir, 'config.py'))
# Vérifier que config est bien accessible
if 'drive.core.config' not in sys.modules:
    sys.modules['drive.core.config'] = config

# Charger les autres modules qui dépendent de config
camera_controller = _load_module('camera_controller', os.path.join(core_dir, 'camera_controller.py'))
udp_server = _load_module('udp_server', os.path.join(core_dir, 'udp_server.py'))
data_publisher = _load_module('data_publisher', os.path.join(core_dir, 'data_publisher.py'))

CameraController = camera_controller.CameraController
UDPServer = udp_server.UDPServer
DataPublisher = data_publisher.DataPublisher
CAM_WIDTH = config.CAM_WIDTH
CAM_HEIGHT = config.CAM_HEIGHT
CAM_FPS = config.CAM_FPS
UDP_PORT = config.UDP_PORT


class CameraStreamApp:
    """Application minimale pour streamer uniquement les données de la caméra."""
    
    def __init__(self, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS, 
                 port=UDP_PORT, debug=False):
        """
        Initialiser l'application.
        
        Args:
            width: Largeur de la caméra en pixels.
            height: Hauteur de la caméra en pixels.
            fps: Frames par seconde.
            port: Port UDP pour l'envoi des données.
            debug: Activer les messages de debug.
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.port = port
        self.debug = debug
        self.running = False
        
        # Composants
        self.camera = None
        self.udp_server = None
        self.data_publisher = None
    
    def initialize(self):
        """Initialiser tous les composants."""
        print("[CameraStream] Initialisation...")
        
        # Initialiser la caméra
        try:
            self.camera = CameraController(
                width=self.width,
                height=self.height,
                fps=self.fps,
                debug=self.debug
            )
            self.camera.initialize()
            print(f"[CameraStream] Caméra initialisée: {self.width}x{self.height} @ {self.fps}fps")
        except Exception as e:
            print(f"[CameraStream] Erreur initialisation caméra: {e}")
            return False
        
        # Vérifier que la caméra est disponible
        if not self.camera.is_available():
            print("[CameraStream] Avertissement: La caméra n'est pas disponible")
        
        # Initialiser le serveur UDP
        try:
            self.udp_server = UDPServer(
                port=self.port,
                debug_payload=self.debug
            )
            self.udp_server.start()
            print(f"[CameraStream] Serveur UDP démarré sur le port {self.port}")
            if self.debug:
                print("[CameraStream] [debug] Mode debug activé - affichage des données envoyées")
        except Exception as e:
            print(f"[CameraStream] Erreur initialisation serveur UDP: {e}")
            return False
        
        # Vérifier PyAV (requis pour H264)
        if not getattr(data_publisher, 'HAS_AV', False):
            print("[CameraStream] ATTENTION: PyAV non installé. Les données caméra ne seront pas envoyées.")
            print("[CameraStream] Installez avec: pip install av")
        
        # Initialiser le publisher (seulement caméra, pas de lidar)
        try:
            self.data_publisher = DataPublisher(
                lidar_controller=None,  # Pas de lidar
                camera_controller=self.camera,
                socket_server=self.udp_server,
                debug_camera=self.debug,
                debug_lidar=False
            )
            self.data_publisher.start()
            print("[CameraStream] Publisher démarré")
            if self.debug:
                print(f"[CameraStream] [debug] Fréquence de publication: {self.data_publisher.publish_rate} Hz")
        except Exception as e:
            print(f"[CameraStream] Erreur initialisation publisher: {e}")
            return False
        
        return True
    
    def run(self):
        """Lancer la boucle principale."""
        if not self.initialize():
            print("[CameraStream] Échec de l'initialisation")
            return 1
        
        self.running = True
        print("[CameraStream] Envoi des données de la caméra... (Ctrl+C pour arrêter)")
        
        # Statistiques pour le debug
        last_debug_time = time.time()
        frame_count = 0
        
        try:
            # La boucle de publication est gérée par DataPublisher dans un thread séparé
            # On surveille les statistiques et affiche le debug si activé
            while self.running:
                time.sleep(0.1)
                
                # Afficher les statistiques toutes les secondes si debug activé
                if self.debug:
                    current_time = time.time()
                    if current_time - last_debug_time >= 1.0:
                        # Récupérer les informations du serveur UDP
                        if self.udp_server is not None:
                            emission_info = self.udp_server.get_last_emission_info()
                            last_sensor_time = emission_info.get('last_sensor_data_time')
                            last_status_time = emission_info.get('last_status_time')
                            data_size = emission_info.get('last_sensor_data_size')
                            
                            # Afficher les statistiques des données envoyées
                            if last_sensor_time is not None:
                                time_since_last = current_time - last_sensor_time
                                size_info = f", {data_size} bytes H264" if data_size else ""
                                print(f"[CameraStream] [debug] Dernier envoi caméra (H264 chunké): il y a {time_since_last:.2f}s{size_info}")
                            
                            # Afficher les informations de la caméra
                            if self.camera is not None and self.camera.is_available():
                                frame = self.camera.get_frame()
                                if frame is not None:
                                    frame_count += 1
                                    print(f"[CameraStream] [debug] Frame #{frame_count}: {frame.shape[1]}x{frame.shape[0]}, disponible: Oui")
                                else:
                                    print("[CameraStream] [debug] Frame: Non disponible")
                            else:
                                print("[CameraStream] [debug] Caméra: Non disponible")
                        
                        last_debug_time = current_time
        except KeyboardInterrupt:
            print("\n[CameraStream] Interruption reçue")
        finally:
            self.stop()
        
        return 0
    
    def stop(self):
        """Arrêter proprement tous les composants."""
        if not self.running:
            return
        
        self.running = False
        print("[CameraStream] Arrêt en cours...")
        
        # Arrêter le publisher
        if self.data_publisher is not None:
            self.data_publisher.stop()
        
        # Arrêter le serveur UDP
        if self.udp_server is not None:
            self.udp_server.stop()
        
        # Arrêter la caméra
        if self.camera is not None:
            self.camera.stop()
        
        print("[CameraStream] Arrêt terminé")


def signal_handler(signum, frame):
    """Gestionnaire de signaux pour arrêt propre."""
    print(f"\n[CameraStream] Signal {signum} reçu")
    if hasattr(signal_handler, 'app') and signal_handler.app is not None:
        signal_handler.app.stop()
    sys.exit(0)


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Script minimal pour envoyer uniquement les données de la caméra via UDP"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=UDP_PORT,
        help=f"Port UDP pour l'envoi des données (défaut: {UDP_PORT})"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Activer les messages de debug"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=CAM_WIDTH,
        help=f"Largeur de la caméra en pixels (défaut: {CAM_WIDTH})"
    )
    parser.add_argument(
        "--height",
        type=int,
        default=CAM_HEIGHT,
        help=f"Hauteur de la caméra en pixels (défaut: {CAM_HEIGHT})"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=CAM_FPS,
        help=f"Frames par seconde (défaut: {CAM_FPS})"
    )
    
    args = parser.parse_args()
    
    # Créer l'application
    app = CameraStreamApp(
        width=args.width,
        height=args.height,
        fps=args.fps,
        port=args.port,
        debug=args.debug
    )
    
    # Enregistrer le gestionnaire de signaux
    signal_handler.app = app
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Lancer l'application
    return app.run()


if __name__ == "__main__":
    sys.exit(main())
