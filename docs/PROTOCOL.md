# Protocole UDP

Ce document décrit le protocole UDP utilisé par le serveur Robocar pour diffuser les données de la caméra et le statut des connexions.

## Vue d'ensemble

Lorsque `--enable-socket` est activé, le serveur diffuse les données via UDP broadcast sur le port configuré (défaut: 3000). Les clients écoutent sur ce port et reçoivent les messages `sensor_data` et `status` à la fréquence de publication (`PUBLISH_RATE`, défaut 10 Hz).

Les clients doivent envoyer des messages heartbeat périodiquement pour indiquer leur présence. Le serveur considère un client comme déconnecté s'il n'a pas reçu de heartbeat depuis 3 secondes.

## Messages émis par le serveur

### Données caméra : paquets chunkés (binaire)

Les données caméra sont envoyées en **paquets UDP chunkés** (format binaire, MTU-friendly). Chaque frame H264 est découpée en chunks de 1400 octets maximum.

**Format par paquet :**
- **En-tête binaire** : `struct.Struct("!IHH")` = `frame_number` (I, 4 octets), `num_chunks` (H, 2 octets), `chunk_index` (H, 2 octets)
- **Payload** : jusqu'à 1400 octets de données H264

Le client doit :
1. Recevoir tous les paquets UDP
2. Détecter le format (en-tête binaire vs JSON) : si les premiers octets correspondent à un en-tête valide (`!IHH`), traiter en mode chunké
3. Réassembler les chunks par `frame_number` et `chunk_index`
4. Décoder le H264 (ex. avec PyAV ou FFmpeg)

**Constantes :**
- `CHUNK_HEADER_FORMAT = "!IHH"`
- `CHUNK_MAX_PAYLOAD_SIZE = 1400`

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

- **Données caméra** : format binaire chunké (en-tête `!IHH` + payload H264)
- **Status** : encodé en JSON puis UTF-8
- Tous les messages sont envoyés en UDP broadcast sur le port configuré (défaut: 3000)
- Les clients doivent écouter sur le même port pour recevoir les messages
- Les clients doivent envoyer leurs heartbeats au même port

## Exemple d'implémentation client (Python)

```python
import socket
import struct
import json
import time
import threading

# Configuration
UDP_PORT = 3000
BROADCAST_ADDR = '255.255.255.255'
CHUNK_HEADER_FORMAT = "!IHH"
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FORMAT)

# Buffer pour réassembler les chunks caméra
frame_buffers = {}  # frame_number -> {total_chunks, chunks: {chunk_idx: bytes}}

# Créer socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', UDP_PORT))

def send_heartbeat():
    heartbeat_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    heartbeat_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while True:
        heartbeat = {"type": "heartbeat", "timestamp": time.time()}
        heartbeat_sock.sendto(json.dumps(heartbeat).encode('utf-8'), (BROADCAST_ADDR, UDP_PORT))
        time.sleep(1.0)

threading.Thread(target=send_heartbeat, daemon=True).start()

while True:
    data, addr = sock.recvfrom(65507)
    # Détecter format : JSON (status) vs binaire chunké (caméra)
    try:
        if data[:1] == b'{':  # Message JSON (status)
            message = json.loads(data.decode('utf-8'))
            if message.get('type') == 'status':
                print(f"Status: camera={message.get('camera_connected')}, clients={message.get('clients_connected')}")
        elif len(data) >= CHUNK_HEADER_SIZE:  # Paquet chunké caméra
            frame_num, total_chunks, chunk_idx = struct.unpack(CHUNK_HEADER_FORMAT, data[:CHUNK_HEADER_SIZE])
            payload = data[CHUNK_HEADER_SIZE:]
            if frame_num not in frame_buffers:
                frame_buffers[frame_num] = {'total_chunks': total_chunks, 'chunks': {}}
            frame_buffers[frame_num]['chunks'][chunk_idx] = payload
            buf = frame_buffers[frame_num]
            if len(buf['chunks']) == buf['total_chunks']:
                full_frame = b''.join(buf['chunks'][i] for i in range(buf['total_chunks']))
                del frame_buffers[frame_num]
                # Décoder H264 avec PyAV : decoder.decode(av.Packet(full_frame))
                print(f"Frame {frame_num} complète: {len(full_frame)} octets H264")
    except (json.JSONDecodeError, UnicodeDecodeError, struct.error) as e:
        print(f"Error: {e}")
```

## Référence

- Serveur: [drive/core/udp_server.py](../drive/core/udp_server.py)
- Collecte et format des données: [drive/core/data_publisher.py](../drive/core/data_publisher.py)
