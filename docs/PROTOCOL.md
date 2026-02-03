# Protocole Socket (Socket.io)

Ce document décrit le protocole Socket.io utilisé par le serveur Robocar pour diffuser les données des capteurs (lidar, caméra, RTK) et le statut des connexions.

## Vue d'ensemble

Lorsque `--enable-socket` est activé, le serveur écoute sur le port configuré (défaut: 3000). Les clients se connectent via Socket.io et reçoivent les événements `sensor_data` et `status` à la fréquence de publication (`PUBLISH_RATE`, défaut 10 Hz).

## Événements émis par le serveur

### `sensor_data`

Payload envoyé périodiquement avec les données des capteurs.

| Champ      | Type   | Description |
|-----------|--------|-------------|
| `timestamp` | number | Timestamp Unix (secondes) au moment de la collecte |
| `lidar`   | object \| null | Données lidar (voir ci-dessous) ou `null` si non disponible |
| `camera`  | object \| null | Données caméra (voir ci-dessous) ou `null` si non disponible |
| `rtk`     | object \| null | Données RTK GNSS (pose + IMU) ou `null` si non disponible |

**Structure de `lidar`** (si présent):

- `points`: tableau des points du scan
- `scan_complete`: booléen

**Structure de `camera`** (si présent):

- `frame`: image encodée en base64 (JPEG)
- `width`, `height`: dimensions en pixels
- `format`: `"jpeg"`

**Structure de `rtk`** (si présent):

- `pose`: objet ou `null` — dernière pose GNSS (champs optionnels selon le message Fusion Engine):
  - `lla_deg`: [lat, lon, alt] en degrés / mètres
  - `solution_type`: type de solution (ex: nom d’énumération)
  - `position_std_enu_m`: écart-type position ENU [e, n, u] en mètres
  - `gps_time`: temps GPS
  - `p1_time`: temps Point One
  - `ypr_deg`: [yaw, pitch, roll] en degrés
  - `velocity_body_mps`: vitesse corps [vx, vy, vz] en m/s
- `imu`: objet ou `null` — dernière mesure IMU (champs optionnels):
  - `accel_xyz`: accélération [ax, ay, az] en m/s²
  - `gyro_xyz`: vitesse angulaire [gx, gy, gz] en rad/s
  - `p1_time`: temps Point One

Les données ne sont émises que si au moins une source (lidar, camera ou rtk avec pose ou imu) est disponible.

### `status`

Payload envoyé périodiquement avec l’état des périphériques et des clients.

| Champ               | Type    | Description |
|---------------------|---------|-------------|
| `timestamp`          | number  | Timestamp Unix (secondes) |
| `lidar_connected`   | boolean | Lidar initialisé et disponible |
| `camera_connected`  | boolean | Caméra initialisée et disponible |
| `rtk_connected`     | boolean | RTK GNSS initialisé et disponible |
| `clients_connected` | number  | Nombre de clients Socket.io connectés |

### À la connexion

Lorsqu’un client se connecte, le serveur envoie immédiatement un événement `status` avec par exemple `message: "Connected to robocar sensor stream"` et `connected: true`.

## Événements reçus par le serveur

| Événement   | Description |
|-------------|-------------|
| `connect`   | Connexion d’un client (réponse: envoi d’un `status` initial) |
| `disconnect`| Déconnexion d’un client |
| `ping`      | Le serveur répond par un événement `pong` avec `timestamp` |

## Référence

- Serveur: [drive/core/socket_server.py](../drive/core/socket_server.py)
- Collecte et format des données: [drive/core/data_publisher.py](../drive/core/data_publisher.py)
- Structure pose/IMU RTK: [drive/core/rtk_controller.py](../drive/core/rtk_controller.py)
