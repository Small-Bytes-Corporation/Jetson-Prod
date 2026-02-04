# Protocole Socket (Socket.io)

Ce document décrit le protocole Socket.io utilisé par le serveur Robocar pour diffuser les données de la caméra et le statut des connexions.

## Vue d'ensemble

Lorsque `--enable-socket` est activé, le serveur écoute sur le port configuré (défaut: 3000). Les clients se connectent via Socket.io et reçoivent les événements `sensor_data` et `status` à la fréquence de publication (`PUBLISH_RATE`, défaut 10 Hz).

## Événements émis par le serveur

### `sensor_data`

Payload envoyé périodiquement avec les données de la caméra.

| Champ      | Type   | Description |
|-----------|--------|-------------|
| `timestamp` | number | Timestamp Unix (secondes) au moment de la collecte |
| `camera`  | object \| null | Données caméra (voir ci-dessous) ou `null` si non disponible |

**Structure de `camera`** (si présent):

- `frame`: image encodée en base64 (JPEG)
- `width`, `height`: dimensions en pixels
- `format`: `"jpeg"`

Les données sont émises dès que la caméra est disponible (initialisée). Si la caméra n'a pas encore produit de données, le payload contient `camera: null` avec un `timestamp`.

### `status`

Payload envoyé périodiquement avec l’état des périphériques et des clients.

| Champ               | Type    | Description |
|---------------------|---------|-------------|
| `timestamp`          | number  | Timestamp Unix (secondes) |
| `camera_connected`  | boolean | Caméra initialisée et disponible |
| `clients_connected` | number  | Nombre de clients Socket.io connectés |

### À la connexion

Lorsqu’un client se connecte, le serveur envoie immédiatement un événement `status` avec le format « à la connexion » (voir section Format JSON exact ci-dessous).

## Format JSON exact

Le bloc `camera` peut être `null` si la source n’est pas disponible ou désactivée.

### Exemple `sensor_data`

```json
{
  "timestamp": 1234567890.123,
  "camera": {
    "frame": "/9j/4AAQSkZJRg...",
    "width": 320,
    "height": 180,
    "format": "jpeg"
  }
}
```

### Exemple `status` à la connexion

Envoyé une fois lorsqu’un client se connecte (pas de `timestamp` ni des booléens périphériques).

```json
{
  "message": "Connected to robocar sensor stream",
  "connected": true
}
```

### Exemple `status` périodique

Envoyé à la fréquence de publication (`PUBLISH_RATE`), avec l’état des périphériques et le nombre de clients.

```json
{
  "timestamp": 1234567890.123,
  "camera_connected": true,
  "clients_connected": 2
}
```

## Événements reçus par le serveur

| Événement   | Description |
|-------------|-------------|
| `connect`   | Connexion d’un client (réponse: envoi d’un `status` initial) |
| `disconnect`| Déconnexion d’un client |
| `ping`      | Le serveur répond par un événement `pong` avec `timestamp` |

## Référence

- Serveur: [drive/core/socket_server.py](../drive/core/socket_server.py)
- Collecte et format des données: [drive/core/data_publisher.py](../drive/core/data_publisher.py)
