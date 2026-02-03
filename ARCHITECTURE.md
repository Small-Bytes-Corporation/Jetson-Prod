# Architecture du système de contrôle Robocar

## Structure modulaire

Le code a été refactorisé en classes et modules pour une meilleure organisation et maintenabilité.

### Structure des dossiers

```
drive/
├── core/                    # Modules de base réutilisables
│   ├── motor_controller.py      # Contrôle du moteur VESC
│   ├── joystick_controller.py   # Gestion du joystick/gamepad
│   ├── throttle_controller.py  # Gestion de l'accélération/freinage
│   ├── camera_controller.py    # Capture de frames caméra (sans IA)
│   ├── lidar_controller.py     # Acquisition de données lidar
│   ├── pantilt_controller.py   # Contrôle pan/tilt de la caméra
│   ├── socket_server.py        # Serveur Socket.io
│   ├── data_publisher.py       # Publication de données via Socket.io
│   └── config.py               # Constantes centralisées
├── applications/            # Applications métier
│   └── manual_drive.py         # Conduite manuelle
└── main.py                  # Point d'entrée principal
```

## Classes principales

### MotorController
Gère l'initialisation et le contrôle du moteur VESC avec retry automatique.

### JoystickController
Gère les inputs du joystick/gamepad via pygame. Fournit des méthodes pour lire les boutons et axes.

### ThrottleController
Calcule l'accélération cible depuis les triggers et applique une interpolation progressive pour des transitions fluides.

### CameraController
Capture des frames brutes depuis la caméra DepthAI. Pas de traitement IA, juste la capture.

### LidarController
Acquiert des données depuis un capteur lidar via communication série.

### PanTiltController
Contrôle le système pan/tilt de la caméra via communication série avec un Arduino. Supporte le mouvement discret avec limites de sécurité.

### SocketServer et DataPublisher
Serveur Socket.io pour le streaming de données en temps réel (lidar, caméra) vers des clients web.

### ManualDriveApp
Application de conduite manuelle qui orchestre tous les contrôleurs dans une boucle propre.

## Flags de modules

Chaque module peut être activé ou désactivé individuellement via des flags. Tous les modules ont un flag correspondant stocké comme attribut dans `ManualDriveApp`:

- `use_motor`: Contrôle du moteur VESC (défaut: `True`)
- `use_joystick`: Gestion du joystick/gamepad (défaut: `True`)
- `use_throttle`: Gestion de l'accélération/freinage (défaut: `True`)
- `use_camera`: Capture de frames caméra (défaut: `False`)
- `use_lidar`: Acquisition de données lidar (défaut: Auto-détecté)
- `use_pan_tilt`: Contrôle pan/tilt de la caméra (défaut: `True`)
- `enable_socket`: Serveur Socket.io pour streaming (défaut: `False`)

Pour une documentation complète des modules, leurs flags, arguments CLI et exemples d'utilisation, voir [MODULES.md](MODULES.md).

## Utilisation

```bash
python3 main.py [--max-speed FLOAT] [--serial-port PATH] [--camera] 
                [--enable-socket] [--lidar-port PATH] [--socket-port INT]
                [--no-motor] [--pan-tilt-port PATH] [--no-pan-tilt]
                [--no-joystick] [--no-throttle] [--no-lidar] [--no-camera]
```

Voir [MODULES.md](MODULES.md) pour la liste complète des arguments et exemples d'utilisation.

## Boucle principale

Chaque application utilise une boucle principale propre qui :
1. Met à jour les inputs (joystick)
2. Calcule les commandes (throttle)
3. Applique les commandes (motor)
4. Récupère les frames caméra si nécessaire
5. Gère les événements (exit, etc.)
6. Contrôle la fréquence avec sleep

## Notes

- Le système n'inclut plus de traitement IA (segmentation, raycasting, etc.).
- La caméra capture uniquement des frames brutes pour affichage/enregistrement.
- Tous les fichiers liés à l'entraînement, l'agent et l'enregistrement ont été supprimés pour simplifier le codebase.
- Tous les modules sont modulaires et peuvent être activés/désactivés individuellement via des flags.
- Les flags permettent une configuration flexible pour différents cas d'usage (tests, développement, production).

## Documentation

- [MODULES.md](MODULES.md): Documentation complète de tous les modules, flags et arguments CLI
- [SOCKET_PROTOCOL.md](SOCKET_PROTOCOL.md): Protocole de communication Socket.io
