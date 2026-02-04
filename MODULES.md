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
| RTK | `use_rtk` | `True` | Contrôleur | Stream RTK GNSS (pose + IMU accel/gyro) |
| Socket | `enable_socket` | `False` | Service | Serveur Socket.io pour streaming |

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
Acquiert des données depuis un capteur lidar via communication série. Les données peuvent être publiées via Socket.io si activé.

**Flag**: `use_lidar` (défaut: Auto-détecté si `lidar_port` fourni)

**Arguments CLI**:
- `--lidar-port PATH`: Spécifie le port série du lidar (active automatiquement le lidar)
- `--no-lidar`: Désactive explicitement le lidar (même si `--lidar-port` est fourni)

**Port série**: Configuré via `--lidar-port` (défaut: détection automatique)

**Dépendances**:
- Généralement utilisé avec `SocketServer` et `DataPublisher` pour streaming

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

### RTKController

**Fichier**: `drive/core/rtk_controller.py`

**Description**:
Lit le flux Fusion Engine sur le port série de la balise Point One (Quectel LG69T) et expose en stream la dernière **pose** (position GNSS, type de solution, vitesse, ypr) et la dernière **IMU** (accéléromètre axes x,y,z en m/s², gyroscope x,y,z en rad/s). L’IMU n’est émise qu’avec le firmware SDK-AP (GNSS+IMU) ; avec SDK-AM (GNSS seul), `get_latest_imu()` restera `None`.

**Flag**: `use_rtk` (défaut: `True`). Le module est intégré au `main.py` ; les données RTK (pose + IMU) sont publiées via Socket.io dans l'événement `sensor_data` lorsque le socket est activé.

**Arguments CLI**:
- `--no-rtk`: Désactive le RTK GNSS (pose/IMU)
- `--rtk-port PATH`: Port série pour le RTK (défaut: `DEFAULT_RTK_SERIAL_PORT` dans `config.py`)

**Port série**: Configuré via `--rtk-port` ou `config.py` (`DEFAULT_RTK_SERIAL_PORT`, `RTK_BAUDRATE`).

**Données exposées**:
- `get_latest_pose()` : dict avec `lla_deg`, `solution_type`, `position_std_enu_m`, `gps_time`, `p1_time`, optionnel `ypr_deg`, `velocity_body_mps`.
- `get_latest_imu()` : dict avec `accel_xyz`, `gyro_xyz` ; `None` si aucun message IMU reçu.

**Dépendances**:
- `fusion-engine-client` (pip), `pyserial`
- Optionnellement `DataPublisher` (si socket activé) pour publier pose et IMU

**Exemple d'utilisation**:
```bash
# RTK activé par défaut avec main.py
python3 main.py

# Désactiver le RTK (tests sans GNSS)
python3 main.py --no-rtk

# Spécifier le port RTK
python3 main.py --rtk-port /dev/ttyUSB0

# Test du module RTK seul (depuis la racine du projet)
python scripts/test_rtk.py

# Test avec port série spécifique
python scripts/test_rtk.py --port /dev/ttyUSB0
```

**Configuration Polaris RTK**:

Le récepteur RTK Point One nécessite une connexion au réseau de correction RTK Polaris pour obtenir une solution valide. La configuration Polaris est disponible dans `drive/core/config.py` :

- `POLARIS_API_KEY` : Clé API Polaris pour l'authentification
- `POLARIS_NTRIP_HOST` : Hostname du serveur NTRIP (truertk.pointonenav.com ou virtualrtk.pointonenav.com)
- `POLARIS_NTRIP_PORT` : Port NTRIP (2102 pour TLS encrypted, 2101 pour plaintext)
- `POLARIS_NTRIP_MOUNT_POINT` : Point de montage NTRIP (POLARIS pour ITRF2014 global datum)

**Méthodes disponibles**:
- `get_polaris_config_info()` : Retourne les informations de configuration Polaris (clé API masquée pour sécurité)

**Scripts de configuration**:

1. **Script bash automatique** (`scripts/setup_rtk_polaris.sh`) - RECOMMANDÉ :

```bash
# Configuration automatique complète (clone p1-host-tools, installe dépendances, configure RTK)
./scripts/setup_rtk_polaris.sh

# Avec options personnalisées
./scripts/setup_rtk_polaris.sh --port /dev/ttyUSB1 --device-id mon-device

# Vérification uniquement (sans configuration)
./scripts/setup_rtk_polaris.sh --check-only --port /dev/ttyUSB0
```

2. **Script Python utilitaire** (`scripts/configure_rtk.py`) :

```bash
# Afficher la configuration Polaris et vérifier la connexion
python scripts/configure_rtk.py

# Afficher uniquement la configuration
python scripts/configure_rtk.py --show-config

# Vérifier la connexion au récepteur RTK
python scripts/configure_rtk.py --check --port /dev/ttyUSB0

# Afficher les instructions de configuration NTRIP
python scripts/configure_rtk.py --instructions
```

**Note importante** : La configuration du récepteur RTK se fait généralement dans le firmware du récepteur lui-même (via l'interface web, les outils Point One, ou les commandes AT). Le script `configure_rtk.py` sert principalement de référence et d'aide à la configuration.

---

### SocketServer et DataPublisher

**Fichiers**: 
- `drive/core/socket_server.py`
- `drive/core/data_publisher.py`

**Description**:
Serveur Socket.io pour le streaming de données en temps réel (lidar, caméra, RTK) vers des clients web ou autres applications. Les données RTK (pose + IMU) sont incluses dans l’événement `sensor_data` ; le statut `status` inclut `rtk_connected`. Voir [docs/PROTOCOL.md](docs/PROTOCOL.md) pour le détail du protocole (événements et formats des payloads).

**Flag**: `enable_socket` (défaut: `False`)

**Arguments CLI**:
- `--enable-socket`: Active le serveur Socket.io
- `--socket-port INT`: Spécifie le port du serveur (défaut: 3000)

**Port**: Configuré via `--socket-port` (défaut: 3000)

**Dépendances**:
- Peut utiliser `LidarController` pour publier des données lidar
- Peut utiliser `CameraController` pour publier des frames caméra
- Peut utiliser `RTKController` pour publier pose et IMU RTK

**Exemple d'utilisation**:
```bash
# Activer le serveur socket avec lidar et RTK
python3 main.py --enable-socket

# Activer avec port personnalisé
python3 main.py --enable-socket --socket-port 8080

# Activer avec caméra, lidar et RTK
python3 main.py --enable-socket --camera --lidar-port /dev/ttyUSB0

# Tests sans RTK
python3 main.py --enable-socket --no-rtk
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
- `--no-rtk`: Désactive le RTK GNSS (pose/IMU)

### Arguments de configuration de ports

- `--lidar-port PATH`: Port série pour le lidar (défaut: auto-détection ou `config.DEFAULT_LIDAR_PORT`)
- `--pan-tilt-port PATH`: Port série pour le pan/tilt (défaut: `config.PAN_TILT_SERIAL_PORT`, typiquement `/dev/ttyACM2`)
- `--rtk-port PATH`: Port série pour le RTK GNSS (défaut: auto-détection ou `config.DEFAULT_RTK_SERIAL_PORT`)
- `--socket-port INT`: Port pour le serveur Socket.io (défaut: 3000)

### Arguments Socket

- `--socket-debug`: Afficher les payloads complets (sensor_data, status) sur la console

### Outils

- `--list-devices`: Lister les ports série et caméras DepthAI avec identification (Lidar, RTK, etc.) puis quitter
- `--no-probe`: Avec `--list-devices`, n'afficher que les infos USB sans sondage protocole (plus rapide, pas d'ouverture exclusive de port)

---

## Découverte des périphériques

La commande `python3 main.py --list-devices` affiche les ports série et les caméras DepthAI détectés, avec une identification des périphériques (Lidar, RTK, etc.) lorsque le sondage protocole est activé. Avec `--no-probe`, seules les informations USB sont listées, sans ouvrir les ports en exclusivité. Implémentation : [drive/core/device_discovery.py](drive/core/device_discovery.py).

```bash
# Identifier qui est branché où (Lidar, RTK, caméra DepthAI)
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

### Tests socket sans RTK
```bash
python3 main.py --enable-socket --no-rtk
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

RTKController (indépendant)
    └── DataPublisher (optionnel, si socket activé)

SocketServer (indépendant)
    └── DataPublisher (nécessite socket server)
```

---

## Notes importantes

1. **Joystick et Throttle**: Essentiels pour le contrôle de base, mais peuvent être désactivés pour tests
2. **Lidar**: Activé automatiquement si `--lidar-port` fourni, sauf si `--no-lidar` est spécifié
3. **Camera**: Désactivée par défaut, doit être explicitement activée avec `--camera`
4. **Socket**: Nécessite généralement lidar, caméra ou RTK pour publier des données utiles
5. **PanTilt**: Activé par défaut, utilise le D-pad du joystick pour le contrôle discret
6. **Motor**: Activé par défaut, nécessite throttle et joystick pour fonctionner correctement
7. **RTK**: Activé par défaut, désactivable avec `--no-rtk` ; données publiées dans `sensor_data` si socket activé

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
- `PUBLISH_RATE`: Fréquence de publication Socket.io (Hz)
- `SOCKETIO_PORT`: Port par défaut du serveur Socket.io
- `DEFAULT_RTK_SERIAL_PORT`, `RTK_BAUDRATE`: Port et débit série RTK

---

Pour le détail du protocole Socket.io (événements et payloads), voir [docs/PROTOCOL.md](docs/PROTOCOL.md).

Pour plus d'informations sur l'architecture générale, voir [ARCHITECTURE.md](ARCHITECTURE.md).
