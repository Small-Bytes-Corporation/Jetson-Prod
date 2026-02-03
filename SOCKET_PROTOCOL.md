# Protocole Socket.io - Robocar Sensor Stream

Référence canonique (format JSON exact et champs détaillés) : [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Vue d'ensemble

Le serveur socket.io diffuse les données du lidar, de la caméra et du RTK GNSS (pose + IMU) en temps réel aux clients connectés.

## Connexion

**URL:** `http://<host>:<port>`

Par défaut: `http://localhost:3000`

## Events

### Events émis par le serveur

#### `sensor_data`

Données des capteurs (lidar + caméra + RTK) envoyées périodiquement. Chaque bloc peut être `null` si la source n'est pas disponible.

**Format:**
```json
{
  "timestamp": 1234567890.123,
  "lidar": {
    "points": [
      { "angle": 0.0, "distance": 1.5, "intensity": 100 }
    ],
    "scan_complete": true
  },
  "camera": {
    "frame": "base64_encoded_jpeg_image",
    "width": 320,
    "height": 180,
    "format": "jpeg"
  },
  "rtk": {
    "pose": {
      "lla_deg": [48.85, 2.35, 100.0],
      "solution_type": "RTK_FIX",
      "position_std_enu_m": [0.01, 0.01, 0.02],
      "gps_time": 1234567890.5,
      "p1_time": 1234567890.5,
      "ypr_deg": [0.1, -0.05, 0.0],
      "velocity_body_mps": [1.0, 0.0, 0.0]
    },
    "imu": {
      "accel_xyz": [0.1, 0.0, 9.81],
      "gyro_xyz": [0.001, 0.002, 0.0],
      "p1_time": 1234567890.5
    }
  }
}
```

**Notes:**
- `timestamp`: Timestamp Unix en secondes (float)
- `lidar.points`: Tableau de points lidar avec angle (degrés), distance (mètres), intensity (0-255)
- `lidar.scan_complete`: Booléen indiquant si le scan est complet
- `camera.frame`: Image JPEG encodée en base64
- `camera.width/height`: Dimensions de l'image
- `camera.format`: Format de l'image ("jpeg")
- `rtk`: Objet ou `null`. `pose`: position GNSS (lla_deg, solution_type, etc.). `imu`: accélération (accel_xyz m/s²), gyro (gyro_xyz rad/s). Voir [docs/PROTOCOL.md](docs/PROTOCOL.md) pour les champs détaillés.

#### `status`

Deux formats possibles :

**À la connexion** (envoyé une fois quand un client se connecte) :
```json
{
  "message": "Connected to robocar sensor stream",
  "connected": true
}
```

**Périodique** (envoyé à la fréquence de publication, avec l'état des périphériques) :
```json
{
  "timestamp": 1234567890.123,
  "lidar_connected": true,
  "camera_connected": true,
  "rtk_connected": true,
  "clients_connected": 2
}
```

### Events émis par le client

#### `ping`

Test de connexion. Le serveur répond avec `pong`.

**Exemple:**
```javascript
socket.emit('ping');
```

#### `connect`

Événement automatique lors de la connexion. Le serveur répond avec un message de statut.

### Events reçus par le client

#### `pong`

Réponse au ping.

**Format:**
```json
{
  "timestamp": 1234567890.123
}
```

## Exemple d'utilisation (JavaScript)

```javascript
const io = require('socket.io-client');

// Connexion au serveur
const socket = io('http://localhost:3000');

// Écouter les données des capteurs
socket.on('sensor_data', (data) => {
  console.log('Timestamp:', data.timestamp);
  
  // Traiter les données lidar
  if (data.lidar && data.lidar.points) {
    console.log(`Lidar: ${data.lidar.points.length} points`);
    data.lidar.points.forEach(point => {
      console.log(`  Angle: ${point.angle}°, Distance: ${point.distance}m`);
    });
  }
  
  // Traiter les données caméra
  if (data.camera && data.camera.frame) {
    const imageData = `data:image/${data.camera.format};base64,${data.camera.frame}`;
    document.getElementById('camera').src = imageData;
  }
  // Traiter les données RTK (pose + IMU)
  if (data.rtk && (data.rtk.pose || data.rtk.imu)) {
    console.log('RTK:', data.rtk);
  }
});

// Écouter le statut (à la connexion: message + connected; périodique: timestamp + *_connected + clients_connected)
socket.on('status', (status) => {
  console.log('Status:', status);
  if (status.clients_connected !== undefined) {
    console.log(`Clients connectés: ${status.clients_connected}`);
  }
});

// Gestion de la connexion
socket.on('connect', () => {
  console.log('Connecté au serveur');
});

socket.on('disconnect', () => {
  console.log('Déconnecté du serveur');
});

// Test de connexion
setInterval(() => {
  socket.emit('ping');
}, 5000);
```

## Exemple d'utilisation (Python)

```python
import socketio

# Créer un client socket.io
sio = socketio.Client()

@sio.on('connect')
def on_connect():
    print('Connecté au serveur')

@sio.on('disconnect')
def on_disconnect():
    print('Déconnecté du serveur')

@sio.on('sensor_data')
def on_sensor_data(data):
    print(f'Timestamp: {data["timestamp"]}')
    
    # Traiter les données lidar
    if data.get('lidar') and data['lidar'].get('points'):
        points = data['lidar']['points']
        print(f'Lidar: {len(points)} points')
        for point in points:
            print(f'  Angle: {point["angle"]}°, Distance: {point["distance"]}m')
    
    # Traiter les données caméra
    if data.get('camera') and data['camera'].get('frame'):
        import base64
        frame_data = base64.b64decode(data['camera']['frame'])
        # Sauvegarder ou traiter l'image
        with open('frame.jpg', 'wb') as f:
            f.write(frame_data)

@sio.on('status')
def on_status(status):
    print(f'Status: {status}')

# Connexion
sio.connect('http://localhost:3000')

# Attendre les événements
sio.wait()
```

## Configuration

### Fréquence de publication

Par défaut: 10 Hz (10 fois par seconde)

Modifiable dans `drive/core/config.py`:
```python
PUBLISH_RATE = 10  # Hz
```

### Port du serveur

Par défaut: 3000

Modifiable via l'argument `--socket-port`:
```bash
python3 main.py --enable-socket --socket-port 8080
```

### CORS

Par défaut: `*` (tous les origines autorisés)

Pour la production, restreindre dans `drive/core/config.py`:
```python
SOCKETIO_CORS_ORIGINS = ['http://localhost:3000', 'https://yourdomain.com']
```

## Notes importantes

- Les images caméra sont encodées en JPEG avec une qualité de 85% pour réduire la taille
- Le lidar doit être connecté et initialisé pour recevoir des données lidar
- La caméra doit être activée avec `--camera` pour recevoir des données caméra
- Le protocole D500 exact doit être implémenté dans `LidarController._parse_lidar_data()`
