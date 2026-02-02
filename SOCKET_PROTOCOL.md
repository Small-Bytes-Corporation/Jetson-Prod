# Protocole Socket.io - Robocar Sensor Stream

## Vue d'ensemble

Le serveur socket.io diffuse les données du lidar et de la caméra en temps réel aux clients connectés.

## Connexion

**URL:** `http://<host>:<port>`

Par défaut: `http://localhost:3000`

## Events

### Events émis par le serveur

#### `sensor_data`

Données des capteurs (lidar + caméra) envoyées périodiquement.

**Format:**
```json
{
  "timestamp": 1234567890.123,
  "lidar": {
    "points": [
      {
        "angle": 0.0,
        "distance": 1.5,
        "intensity": 100
      },
      ...
    ],
    "scan_complete": true
  },
  "camera": {
    "frame": "base64_encoded_jpeg_image",
    "width": 320,
    "height": 180,
    "format": "jpeg"
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

#### `status`

Statut du système envoyé périodiquement.

**Format:**
```json
{
  "timestamp": 1234567890.123,
  "lidar_connected": true,
  "camera_connected": true,
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
    // Afficher l'image dans un élément <img>
    document.getElementById('camera').src = imageData;
  }
});

// Écouter le statut
socket.on('status', (status) => {
  console.log('Status:', status);
  console.log(`Clients connectés: ${status.clients_connected}`);
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
