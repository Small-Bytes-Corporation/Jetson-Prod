# Protocole UDP - Robocar Sensor Stream

Référence canonique (format JSON exact et champs détaillés) : [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Vue d'ensemble

Le serveur UDP diffuse les données de la caméra en temps réel aux clients connectés via UDP broadcast.

## Connexion

**Port UDP:** Par défaut: `3000`

Les clients doivent écouter sur ce port pour recevoir les messages broadcast.

## Messages

### Messages émis par le serveur

#### Données caméra : paquets chunkés (binaire)

Les données caméra sont envoyées en **paquets UDP chunkés** (format binaire, MTU-friendly). Chaque frame H264 est découpée en chunks de 1400 octets maximum.

**Format par paquet :**
- **En-tête** : `struct.Struct("!IHH")` = `frame_number` (4 octets), `num_chunks` (2 octets), `chunk_index` (2 octets)
- **Payload** : jusqu'à 1400 octets de données H264

Le client doit réassembler les chunks par `frame_number` puis décoder le H264 (ex. PyAV).

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
import struct
import json
import time
import threading

# Configuration
UDP_PORT = 3000
BROADCAST_ADDR = '255.255.255.255'
CHUNK_HEADER_FORMAT = "!IHH"
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FORMAT)
frame_buffers = {}  # frame_number -> {total_chunks, chunks: {}}

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
recv_sock.bind(('', UDP_PORT))

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def send_heartbeat():
    while True:
        send_sock.sendto(json.dumps({"type": "heartbeat", "timestamp": time.time()}).encode('utf-8'),
                         (BROADCAST_ADDR, UDP_PORT))
        time.sleep(1.0)

threading.Thread(target=send_heartbeat, daemon=True).start()

print("Écoute des messages UDP sur le port", UDP_PORT)
while True:
    data, addr = recv_sock.recvfrom(65507)
    try:
        if data[:1] == b'{':  # JSON (status)
            message = json.loads(data.decode('utf-8'))
            if message.get('type') == 'status':
                print(f"Status: camera={message.get('camera_connected')}")
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
                # Décoder H264 avec PyAV, ou sauvegarder
                with open(f'frame_{frame_num}.h264', 'wb') as f:
                    f.write(full_frame)
                print(f"Frame {frame_num} complète: {len(full_frame)} octets")
    except (json.JSONDecodeError, UnicodeDecodeError, struct.error) as e:
        print(f"Erreur: {e}")
```

## Exemple d'utilisation (JavaScript/Node.js)

```javascript
const dgram = require('dgram');
const client = dgram.createSocket('udp4');

const UDP_PORT = 3000;
const BROADCAST_ADDR = '255.255.255.255';
const CHUNK_HEADER_SIZE = 8;  // !IHH = 4 + 2 + 2 octets
const frameBuffers = {};  // frame_number -> {total_chunks, chunks: {}}

client.bind(UDP_PORT, () => {
    console.log(`Écoute des messages UDP sur le port ${UDP_PORT}`);
    client.setBroadcast(true);
});

setInterval(() => {
    client.send(JSON.stringify({ type: 'heartbeat', timestamp: Date.now() / 1000 }),
               UDP_PORT, BROADCAST_ADDR);
}, 1000);

client.on('message', (msg, rinfo) => {
    try {
        if (msg[0] === 0x7B) {  // '{' = JSON (status)
            const message = JSON.parse(msg.toString());
            if (message.type === 'status') {
                console.log(`Status: camera=${message.camera_connected}`);
            }
        } else if (msg.length >= CHUNK_HEADER_SIZE) {  // Paquet chunké caméra
            const frameNum = msg.readUInt32BE(0);
            const totalChunks = msg.readUInt16BE(4);
            const chunkIdx = msg.readUInt16BE(6);
            const payload = msg.slice(CHUNK_HEADER_SIZE);
            if (!frameBuffers[frameNum]) {
                frameBuffers[frameNum] = { totalChunks, chunks: {} };
            }
            frameBuffers[frameNum].chunks[chunkIdx] = payload;
            const buf = frameBuffers[frameNum];
            if (Object.keys(buf.chunks).length === buf.totalChunks) {
                const fullFrame = Buffer.concat(
                    [...Array(buf.totalChunks)].map((_, i) => buf.chunks[i])
                );
                delete frameBuffers[frameNum];
                console.log(`Frame ${frameNum} complète: ${fullFrame.length} octets H264`);
            }
        }
    } catch (e) {
        console.error('Erreur:', e);
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

- Les images caméra sont encodées en H264 (PyAV) et envoyées en paquets chunkés de 1400 octets (MTU-friendly)
- La caméra doit être activée avec `--camera` pour recevoir des données caméra
- Les messages sont envoyés en UDP broadcast (255.255.255.255)
- Les clients doivent envoyer des heartbeats régulièrement pour être comptés comme connectés
- La taille maximale d'un datagramme UDP est de 65507 octets
