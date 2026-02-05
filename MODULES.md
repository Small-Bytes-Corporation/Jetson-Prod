# Documentation des Modules

Ce document décrit tous les modules disponibles dans le système de contrôle Robocar, leurs flags de configuration, et leur utilisation.

## Vue d'ensemble

Le système Robocar est composé de plusieurs modules modulaires qui peuvent être activés ou désactivés individuellement selon les besoins. Chaque module a un flag correspondant qui contrôle son activation.

## Tableau récapitulatif des modules

| Module | Flag | Valeur par défaut | Type | Description |
|--------|------|-------------------|------|-------------|
| Motor | `use_motor` | `True` | Contrôleur | Contrôle du moteur VESC |
| Joystick | `use_joystick` | `True` | Contrôleur | Gestion des entrées joystick/gamepad |
| Throttle | `use_throttle` | `True` | Contrôleur | Gestion de l'accélération/freinage |
| Camera | `use_camera` | `False` | Contrôleur | Capture de frames caméra DepthAI |
| Lidar | `use_lidar` | Auto | Contrôleur | Acquisition de données lidar |
| PanTilt | `use_pan_tilt` | `True` | Contrôleur | Contrôle pan/tilt de la caméra |
| Network | `enable_socket` | `False` | Service | Serveur UDP pour streaming |

## Description détaillée des modules

### MotorController

**Fichier**: `drive/core/motor_controller.py`

**Description**: 
Contrôle le moteur VESC (Vedder Electronic Speed Controller) pour la propulsion du véhicule. Gère l'accélération et la direction via les commandes servo.

**Flag**: `use_motor` (défaut: `True`)

**Arguments CLI**:
- `--no-motor`: Désactive le contrôleur moteur (mode mock)

**Port série**: Configuré via `--serial-port` (défaut: `config.DEFAULT_SERIAL_PORT`, typiquement `/dev/ttyACM1`)

**Dépendances**:
- Nécessite `ThrottleController` pour calculer l'accélération
- Nécessite `JoystickController` pour obtenir la direction

**Exemple d'utilisation**:
```bash
# Activer le moteur (par défaut)
python3 main.py

# Désactiver le moteur pour tests
python3 main.py --no-motor
```

---

### JoystickController

**Fichier**: `drive/core/joystick_controller.py`

**Description**:
Gère les entrées du joystick/gamepad via pygame. Fournit des méthodes pour lire les boutons, axes analogiques, triggers et D-pad.

**Flag**: `use_joystick` (défaut: `True`)

**Arguments CLI**:
- `--no-joystick`: Désactive le contrôleur joystick

**Dépendances**:
- Aucune (module de base)

**Utilisé par**:
- Tous les autres modules pour les entrées utilisateur
- Nécessaire pour le contrôle de base (direction, accélération, pan/tilt)

**Exemple d'utilisation**:
```bash
# Utiliser le joystick (par défaut)
python3 main.py

# Désactiver le joystick (pour tests sans gamepad)
python3 main.py --no-joystick
```

---

### ThrottleController

**Fichier**: `drive/core/throttle_controller.py`

**Description**:
Calcule l'accélération cible depuis les triggers du joystick (RT pour accélérer, LT pour freiner) et applique une interpolation progressive pour des transitions fluides.

**Flag**: `use_throttle` (défaut: `True`)

**Arguments CLI**:
- `--no-throttle`: Désactive le contrôleur throttle

**Dépendances**:
- Nécessite `JoystickController` pour lire les triggers
- Généralement utilisé avec `MotorController`

**Exemple d'utilisation**:
```bash
# Utiliser le throttle (par défaut)
python3 main.py

# Désactiver le throttle (pour tests)
python3 main.py --no-throttle
```

---

### CameraController

**Fichier**: `drive/core/camera_controller.py`

**Description**:
Capture des frames brutes depuis la caméra DepthAI. Pas de traitement IA, juste la capture de frames pour affichage ou enregistrement.

**Flag**: `use_camera` (défaut: `False`)

**Arguments CLI**:
- `--camera`: Active la caméra
- `--no-camera`: Désactive explicitement la caméra (même si `--camera` est présent)

**Configuration**:
- Résolution: 320x180 (configurable dans `config.py`)
- FPS: 30 (configurable dans `config.py`)

**Dépendances**:
- Aucune

**Exemple d'utilisation**:
```bash
# Activer la caméra
python3 main.py --camera

# Désactiver explicitement la caméra
python3 main.py --camera --no-camera
```

---

### LidarController

**Fichier**: `drive/core/lidar_controller.py`

**Description**:
Acquiert des données depuis un capteur lidar via communication série. Les données peuvent être publiées via UDP si activé.

**Flag**: `use_lidar` (défaut: Auto-détecté si `lidar_port` fourni)

**Arguments CLI**:
- `--lidar-port PATH`: Spécifie le port série du lidar (active automatiquement le lidar)
- `--no-lidar`: Désactive explicitement le lidar (même si `--lidar-port` est fourni)

**Port série**: Configuré via `--lidar-port` (défaut: détection automatique)

**Dépendances**:
- Généralement utilisé avec `UDPServer` et `DataPublisher` pour streaming

**Exemple d'utilisation**:
```bash
# Activer le lidar avec port spécifique
python3 main.py --lidar-port /dev/ttyUSB0

# Activer le lidar avec socket (utilise le port par défaut)
python3 main.py --enable-socket

# Désactiver explicitement le lidar
python3 main.py --lidar-port /dev/ttyUSB0 --no-lidar
```

---

### PanTiltController

**Fichier**: `drive/core/pantilt_controller.py`

**Description**:
Contrôle le système pan/tilt de la caméra via communication série avec un Arduino. Supporte le mouvement discret via le D-pad du joystick avec limites de sécurité.

**Flag**: `use_pan_tilt` (défaut: `True`)

**Arguments CLI**:
- `--pan-tilt-port PATH`: Spécifie le port série du contrôleur pan/tilt
- `--no-pan-tilt`: Désactive le contrôle pan/tilt

**Port série**: Configuré via `--pan-tilt-port` (défaut: `config.PAN_TILT_SERIAL_PORT`, typiquement `/dev/ttyACM2`)

**Contrôles**:
- D-pad gauche/droite: Pan (rotation horizontale)
- D-pad haut/bas: Tilt (rotation verticale)
- Mouvement discret: se déplace uniquement quand le D-pad est pressé

**Limites**:
- Pan: -1.0 à +1.0 (configurable)
- Tilt: -1.0 à +1.0 (configurable)
- Step size: 0.05 par incrément (configurable)

**Dépendances**:
- Nécessite `JoystickController` pour lire le D-pad

**Exemple d'utilisation**:
```bash
# Utiliser pan/tilt (par défaut)
python3 main.py

# Spécifier un port série différent
python3 main.py --pan-tilt-port /dev/ttyACM1

# Désactiver pan/tilt
python3 main.py --no-pan-tilt
```

---

---

### UDPServer et DataPublisher

**Fichiers**: 
- `drive/core/udp_server.py`
- `drive/core/data_publisher.py`

**Description**:
Serveur UDP pour le streaming de données en temps réel (lidar, caméra) vers des clients via UDP broadcast. Voir [docs/PROTOCOL.md](docs/PROTOCOL.md) pour le détail du protocole (messages et formats des payloads).

**Flag**: `enable_socket` (défaut: `False`)

**Arguments CLI**:
- `--enable-socket`: Active le serveur UDP
- `--socket-port INT`: Spécifie le port du serveur (défaut: 3000)

**Port**: Configuré via `--socket-port` (défaut: 3000)

**Dépendances**:
- Peut utiliser `LidarController` pour publier des données lidar
- Peut utiliser `CameraController` pour publier des frames caméra

**Exemple d'utilisation**:
```bash
# Activer le serveur socket avec lidar
python3 main.py --enable-socket

# Activer avec port personnalisé
python3 main.py --enable-socket --socket-port 8080

# Activer avec caméra et lidar
python3 main.py --enable-socket --camera --lidar-port /dev/ttyUSB0

# Tests socket sans lidar
python3 main.py --enable-socket --no-lidar
```

---

## Arguments CLI complets

### Arguments de configuration générale

- `--max-speed FLOAT`: Vitesse maximale (défaut: 0.15)
- `--serial-port PATH`: Port série pour le moteur VESC (défaut: `config.DEFAULT_SERIAL_PORT`, typiquement `/dev/ttyACM1`)

### Arguments d'activation de modules

- `--camera`: Active la caméra
- `--enable-socket`: Active le serveur Socket.io

### Arguments de désactivation de modules

- `--no-motor`: Désactive le moteur
- `--no-joystick`: Désactive le joystick
- `--no-throttle`: Désactive le throttle
- `--no-camera`: Désactive la caméra
- `--no-lidar`: Désactive le lidar
- `--no-pan-tilt`: Désactive le pan/tilt

### Arguments de configuration de ports

- `--lidar-port PATH`: Port série pour le lidar (défaut: auto-détection ou `config.DEFAULT_LIDAR_PORT`)
- `--pan-tilt-port PATH`: Port série pour le pan/tilt (défaut: `config.PAN_TILT_SERIAL_PORT`, typiquement `/dev/ttyACM2`)
- `--socket-port INT`: Port pour le serveur UDP (défaut: 3000)

### Arguments Socket

- `--socket-debug`: Afficher les payloads complets (sensor_data, status) sur la console

### Outils

- `--list-devices`: Lister les ports série et caméras DepthAI avec identification (Lidar, VESC, etc.) puis quitter
- `--no-probe`: Avec `--list-devices`, n'afficher que les infos USB sans sondage protocole (plus rapide, pas d'ouverture exclusive de port)

---

## Découverte des périphériques

La commande `python3 main.py --list-devices` affiche les ports série et les caméras DepthAI détectés, avec une identification des périphériques (Lidar, VESC, etc.) lorsque le sondage protocole est activé. Avec `--no-probe`, seules les informations USB sont listées, sans ouvrir les ports en exclusivité. Implémentation : [drive/core/device_discovery.py](drive/core/device_discovery.py).

```bash
# Identifier qui est branché où (Lidar, VESC, caméra DepthAI)
python3 main.py --list-devices

# Liste USB uniquement (rapide, sans sondage)
python3 main.py --list-devices --no-probe
```

---

## Exemples d'utilisation

### Configuration minimale (moteur + joystick)
```bash
python3 main.py
```

### Configuration complète
```bash
python3 main.py --camera --enable-socket --lidar-port /dev/ttyUSB0
```

### Tests sans matériel
```bash
python3 main.py --no-motor --no-pan-tilt --enable-socket
```

### Tests lidar uniquement
```bash
python3 main.py --no-motor --no-joystick --no-throttle --lidar-port /dev/ttyUSB0 --enable-socket
```

### Tests socket sans lidar
```bash
python3 main.py --enable-socket --no-lidar
```

### Configuration personnalisée
```bash
python3 main.py \
  --max-speed 0.2 \
  --serial-port /dev/ttyACM0 \
  --camera \
  --enable-socket \
  --lidar-port /dev/ttyUSB0 \
  --socket-port 8080 \
  --pan-tilt-port /dev/ttyACM1
```

---

## Dépendances entre modules

```
JoystickController (base)
    ├── MotorController (nécessite joystick pour direction)
    ├── ThrottleController (nécessite joystick pour triggers)
    └── PanTiltController (nécessite joystick pour D-pad)

CameraController (indépendant)
    └── DataPublisher (optionnel, si socket activé)

LidarController (indépendant)
    └── DataPublisher (optionnel, si socket activé)

    └── DataPublisher (optionnel, si socket activé)

UDPServer (indépendant)
    └── DataPublisher (nécessite UDP server)
```

---

## Notes importantes

1. **Joystick et Throttle**: Essentiels pour le contrôle de base, mais peuvent être désactivés pour tests
2. **Lidar**: Activé automatiquement si `--lidar-port` fourni, sauf si `--no-lidar` est spécifié
3. **Camera**: Désactivée par défaut, doit être explicitement activée avec `--camera`
4. **Socket**: Nécessite généralement lidar ou caméra pour publier des données utiles
5. **PanTilt**: Activé par défaut, utilise le D-pad du joystick pour le contrôle discret
6. **Motor**: Activé par défaut, nécessite throttle et joystick pour fonctionner correctement

---

## Configuration avancée

Les constantes de configuration peuvent être modifiées dans `drive/core/config.py`:

- `MAX_SPEED`: Vitesse maximale par défaut
- `DEFAULT_SERIAL_PORT`: Port série moteur VESC par défaut
- `PAN_TILT_SERIAL_PORT`: Port série pan/tilt par défaut
- `CAM_WIDTH`, `CAM_HEIGHT`, `CAM_FPS`: Configuration caméra
- `PAN_TILT_STEP_SIZE`: Taille du pas pour mouvement discret pan/tilt
- `PAN_MIN/MAX`, `TILT_MIN/MAX`: Limites pan/tilt
- `LOOP_SLEEP_TIME`: Fréquence de la boucle principale
- `PUBLISH_RATE`: Fréquence de publication UDP (Hz)
- `UDP_PORT`: Port par défaut du serveur UDP

---

Pour le détail du protocole UDP (messages et payloads), voir [docs/PROTOCOL.md](docs/PROTOCOL.md) et [UDP_PROTOCOL.md](UDP_PROTOCOL.md).

Pour plus d'informations sur l'architecture générale, voir [ARCHITECTURE.md](ARCHITECTURE.md).
