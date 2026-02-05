# Protocole UDP

Ce document décrit le protocole UDP utilisé par le serveur Robocar pour diffuser les données de la caméra et le statut des connexions.

## Vue d'ensemble

Lorsque `--enable-socket` est activé, le serveur diffuse les données via UDP broadcast sur le port configuré (défaut: 3000). Les clients écoutent sur ce port et reçoivent les messages `sensor_data` et `status` à la fréquence de publication (`PUBLISH_RATE`, défaut 10 Hz).

Les clients doivent envoyer des messages heartbeat périodiquement pour indiquer leur présence. Le serveur considère un client comme déconnecté s'il n'a pas reçu de heartbeat depuis 3 secondes.

## Messages émis par le serveur

Tous les messages sont envoyés en UDP broadcast (255.255.255.255) et contiennent un champ `type` pour identifier le type de message.

### `sensor_data`

Message envoyé périodiquement avec les données de la caméra.

**Format JSON:**
```json
{
  "type": "sensor_data",
  "timestamp": 1234567890.123,
  "camera": {
    "frame": "/9j/4AAQSkZJRg...",
    "width": 320,
    "height": 180,
    "format": "jpeg"
  }
}
```

| Champ      | Type   | Description |
|-----------|--------|-------------|
| `type` | string | Toujours `"sensor_data"` |
| `timestamp` | number | Timestamp Unix (secondes) au moment de la collecte |
| `camera`  | object \| null | Données caméra (voir ci-dessous) ou `null` si non disponible |

**Structure de `camera`** (si présent):
- `frame`: image encodée en base64 (JPEG)
- `width`, `height`: dimensions en pixels
- `format`: `"jpeg"`

Les données sont émises dès que la caméra est disponible (initialisée). Si la caméra n'a pas encore produit de données, le payload contient `camera: null` avec un `timestamp`.

### `status`

Message envoyé périodiquement avec l'état des périphériques et des clients.

**Format JSON:**
```json
{
  "type": "status",
  "timestamp": 1234567890.123,
  "camera_connected": true,
  "clients_connected": 2
}
```

| Champ               | Type    | Description |
|---------------------|---------|-------------|
| `type` | string | Toujours `"status"` |
| `timestamp`          | number  | Timestamp Unix (secondes) |
| `camera_connected`  | boolean | Caméra initialisée et disponible |
| `clients_connected` | number  | Nombre de clients UDP connectés (ayant envoyé un heartbeat récent) |

## Messages reçus par le serveur

### `heartbeat`

Les clients doivent envoyer un message heartbeat périodiquement (recommandé: toutes les 1 seconde) pour indiquer leur présence.

**Format JSON:**
```json
{
  "type": "heartbeat",
  "timestamp": 1234567890.123
}
```

| Champ      | Type   | Description |
|-----------|--------|-------------|
| `type` | string | Toujours `"heartbeat"` |
| `timestamp` | number | Timestamp Unix (secondes) |

Le serveur répond implicitement en continuant à envoyer les messages `sensor_data` et `status`. Un client est considéré comme déconnecté s'il n'a pas envoyé de heartbeat depuis 3 secondes.

## Encodage et transmission

- Tous les messages sont encodés en JSON puis en UTF-8 avant transmission
- Les messages sont envoyés en UDP broadcast sur le port configuré (défaut: 3000)
- Les clients doivent écouter sur le même port pour recevoir les messages
- Les clients doivent envoyer leurs heartbeats au même port

## Exemple d'implémentation client (Python)

```python
import socket
import json
import time

# Configuration
UDP_PORT = 3000
BROADCAST_ADDR = '255.255.255.255'

# Créer socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', UDP_PORT))

# Thread pour envoyer les heartbeats
def send_heartbeat():
    heartbeat_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    heartbeat_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while True:
        heartbeat = {
            "type": "heartbeat",
            "timestamp": time.time()
        }
        message = json.dumps(heartbeat).encode('utf-8')
        heartbeat_sock.sendto(message, (BROADCAST_ADDR, UDP_PORT))
        time.sleep(1.0)  # Envoyer toutes les secondes

# Démarrer le thread heartbeat
import threading
heartbeat_thread = threading.Thread(target=send_heartbeat, daemon=True)
heartbeat_thread.start()

# Écouter les messages
while True:
    data, addr = sock.recvfrom(65507)  # Taille max UDP
    try:
        message = json.loads(data.decode('utf-8'))
        msg_type = message.get('type')
        
        if msg_type == 'sensor_data':
            print(f"Received sensor_data: timestamp={message.get('timestamp')}")
            # Traiter les données caméra si présentes
            if message.get('camera'):
                frame_base64 = message['camera']['frame']
                # Décoder l'image base64...
        
        elif msg_type == 'status':
            print(f"Status: camera={message.get('camera_connected')}, clients={message.get('clients_connected')}")
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        print(f"Error decoding message: {e}")
```

## Référence

- Serveur: [drive/core/udp_server.py](../drive/core/udp_server.py)
- Collecte et format des données: [drive/core/data_publisher.py](../drive/core/data_publisher.py)
