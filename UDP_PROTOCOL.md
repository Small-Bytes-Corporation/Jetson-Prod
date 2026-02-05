# Protocole UDP - Robocar Sensor Stream

Référence canonique (format JSON exact et champs détaillés) : [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Vue d'ensemble

Le serveur UDP diffuse les données de la caméra en temps réel aux clients connectés via UDP broadcast.

## Connexion

**Port UDP:** Par défaut: `3000`

Les clients doivent écouter sur ce port pour recevoir les messages broadcast.

## Messages

### Messages émis par le serveur

#### `sensor_data`

Données de la caméra envoyées périodiquement. Le bloc peut être `null` si la source n'est pas disponible.

**Format:**
```json
{
  "type": "sensor_data",
  "timestamp": 1234567890.123,
  "camera": {
    "frame": "base64_encoded_jpeg_image",
    "width": 320,
    "height": 180,
    "format": "jpeg"
  }
}
```

**Notes:**
- `type`: Toujours `"sensor_data"`
- `timestamp`: Timestamp Unix en secondes (float)
- `camera.frame`: Image JPEG encodée en base64
- `camera.width/height`: Dimensions de l'image
- `camera.format`: Format de l'image ("jpeg")

#### `status`

Message envoyé périodiquement avec l'état des périphériques et des clients.

**Format:**
```json
{
  "type": "status",
  "timestamp": 1234567890.123,
  "camera_connected": true,
  "clients_connected": 2
}
```

**Notes:**
- `type`: Toujours `"status"`
- `timestamp`: Timestamp Unix en secondes
- `camera_connected`: Boolean indiquant si la caméra est disponible
- `clients_connected`: Nombre de clients ayant envoyé un heartbeat récent

### Messages émis par le client

#### `heartbeat`

Les clients doivent envoyer un message heartbeat périodiquement (recommandé: toutes les 1 seconde) pour indiquer leur présence.

**Format:**
```json
{
  "type": "heartbeat",
  "timestamp": 1234567890.123
}
```

Le serveur considère un client comme déconnecté s'il n'a pas reçu de heartbeat depuis 3 secondes.

## Exemple d'utilisation (Python)

```python
import socket
import json
import time
import threading
import base64

# Configuration
UDP_PORT = 3000
BROADCAST_ADDR = '255.255.255.255'

# Créer socket UDP pour recevoir
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
recv_sock.bind(('', UDP_PORT))

# Socket pour envoyer les heartbeats
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def send_heartbeat():
    """Envoyer un heartbeat toutes les secondes"""
    while True:
        heartbeat = {
            "type": "heartbeat",
            "timestamp": time.time()
        }
        message = json.dumps(heartbeat).encode('utf-8')
        send_sock.sendto(message, (BROADCAST_ADDR, UDP_PORT))
        time.sleep(1.0)

# Démarrer le thread heartbeat
heartbeat_thread = threading.Thread(target=send_heartbeat, daemon=True)
heartbeat_thread.start()

# Écouter les messages
print("Écoute des messages UDP sur le port", UDP_PORT)
while True:
    data, addr = recv_sock.recvfrom(65507)  # Taille max UDP
    try:
        message = json.loads(data.decode('utf-8'))
        msg_type = message.get('type')
        
        if msg_type == 'sensor_data':
            print(f"Timestamp: {message.get('timestamp')}")
            
            # Traiter les données caméra
            if message.get('camera') and message['camera'].get('frame'):
                frame_data = base64.b64decode(message['camera']['frame'])
                # Sauvegarder ou traiter l'image
                with open('frame.jpg', 'wb') as f:
                    f.write(frame_data)
                print("Image sauvegardée")
        
        elif msg_type == 'status':
            print(f"Status: camera={message.get('camera_connected')}, clients={message.get('clients_connected')}")
    
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        print(f"Erreur de décodage: {e}")
```

## Exemple d'utilisation (JavaScript/Node.js)

```javascript
const dgram = require('dgram');
const client = dgram.createSocket('udp4');

const UDP_PORT = 3000;
const BROADCAST_ADDR = '255.255.255.255';

// Écouter les messages
client.bind(UDP_PORT, () => {
    console.log(`Écoute des messages UDP sur le port ${UDP_PORT}`);
    client.setBroadcast(true);
});

// Envoyer des heartbeats périodiquement
setInterval(() => {
    const heartbeat = JSON.stringify({
        type: 'heartbeat',
        timestamp: Date.now() / 1000
    });
    client.send(heartbeat, UDP_PORT, BROADCAST_ADDR, (err) => {
        if (err) console.error('Erreur envoi heartbeat:', err);
    });
}, 1000); // Toutes les secondes

// Recevoir les messages
client.on('message', (msg, rinfo) => {
    try {
        const message = JSON.parse(msg.toString());
        const msgType = message.type;
        
        if (msgType === 'sensor_data') {
            console.log('Timestamp:', message.timestamp);
            
            // Traiter les données caméra
            if (message.camera && message.camera.frame) {
                const imageData = `data:image/${message.camera.format};base64,${message.camera.frame}`;
                // Afficher l'image dans le navigateur ou la sauvegarder
                console.log('Image reçue:', message.camera.width, 'x', message.camera.height);
            }
        } else if (msgType === 'status') {
            console.log(`Status: camera=${message.camera_connected}, clients=${message.clients_connected}`);
        }
    } catch (e) {
        console.error('Erreur de décodage:', e);
    }
});
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

### Timeout des heartbeats

Par défaut: 3 secondes

Un client est considéré comme déconnecté s'il n'a pas envoyé de heartbeat depuis ce délai.

Modifiable dans `drive/core/config.py`:
```python
UDP_HEARTBEAT_TIMEOUT = 3.0  # secondes
```

## Notes importantes

- Les images caméra sont encodées en JPEG avec une qualité de 85% pour réduire la taille
- La caméra doit être activée avec `--camera` pour recevoir des données caméra
- Les messages sont envoyés en UDP broadcast (255.255.255.255)
- Les clients doivent envoyer des heartbeats régulièrement pour être comptés comme connectés
- La taille maximale d'un datagramme UDP est de 65507 octets
