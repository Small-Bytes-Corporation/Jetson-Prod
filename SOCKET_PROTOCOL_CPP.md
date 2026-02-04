# Documentation C++ - Client Socket.io Robocar

## Vue d'ensemble

Cette documentation décrit comment implémenter un client C++ pour recevoir et parser les données du serveur socket.io du robocar. Le serveur diffuse les données de la caméra en temps réel.

## Bibliothèques recommandées

Pour implémenter le client socket.io en C++, vous pouvez utiliser :

- **socket.io-client-cpp** : https://github.com/socketio/socket.io-client-cpp
- **nlohmann/json** : https://github.com/nlohmann/json (pour le parsing JSON)
- **Base64** : Pour décoder les images caméra (bibliothèque standard ou https://github.com/ReneNyffenegger/cpp-base64)

## Connexion au serveur

**URL:** `http://<host>:<port>`

Par défaut: `http://localhost:3000`

**Transport:** WebSocket (socket.io utilise WebSocket par défaut)

## Structure des messages

### Format général

Tous les messages sont au format JSON et sont envoyés via les events socket.io suivants :

- `sensor_data` : Données caméra
- `status` : Statut du système (deux formats : à la connexion ou périodique)
- `pong` : Réponse au ping

## Event: `sensor_data`

### Structure JSON

Le bloc `camera` peut être `null` si la source n'est pas disponible.

```json
{
  "timestamp": 1234567890.123,
  "camera": {
    "frame": "base64_encoded_jpeg_image",
    "width": 320,
    "height": 180,
    "format": "jpeg"
  }
}
```

### Structure C++ recommandée

```cpp
struct CameraData {
    std::string frame_base64;  // Image JPEG encodée en base64
    int width;
    int height;
    std::string format;        // "jpeg"
};

struct SensorData {
    double timestamp;          // Timestamp Unix (secondes)
    CameraData camera;
};
```

### Exemple de parsing avec nlohmann/json

```cpp
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

using json = nlohmann::json;

SensorData parseSensorData(const std::string& json_string) {
    SensorData data;
    json j = json::parse(json_string);
    
    // Parser timestamp
    data.timestamp = j["timestamp"].get<double>();
    
    // Parser données caméra
    if (j.contains("camera") && !j["camera"].is_null()) {
        json camera_json = j["camera"];
        data.camera.frame_base64 = camera_json.value("frame", "");
        data.camera.width = camera_json.value("width", 0);
        data.camera.height = camera_json.value("height", 0);
        data.camera.format = camera_json.value("format", "jpeg");
    }
    
    return data;
}
```

### Décodage de l'image caméra

```cpp
#include <string>
#include <vector>
#include <base64.h>  // Utilisez votre bibliothèque base64 préférée

std::vector<uint8_t> decodeCameraFrame(const CameraData& camera_data) {
    // Décoder la chaîne base64 en bytes JPEG
    std::string decoded = base64_decode(camera_data.frame_base64);
    return std::vector<uint8_t>(decoded.begin(), decoded.end());
}

// Exemple d'utilisation avec OpenCV pour afficher l'image
#include <opencv2/opencv.hpp>

cv::Mat decodeCameraImage(const CameraData& camera_data) {
    std::vector<uint8_t> jpeg_data = decodeCameraFrame(camera_data);
    cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
    return image;
}
```

## Event: `status`

Deux formats possibles : **à la connexion** (envoyé une fois) ou **périodique** (envoyé à la fréquence de publication). Le parser doit gérer les champs optionnels.

### Structure JSON à la connexion

```json
{
  "message": "Connected to robocar sensor stream",
  "connected": true
}
```

### Structure JSON périodique

```json
{
  "timestamp": 1234567890.123,
  "camera_connected": true,
  "clients_connected": 2
}
```

### Structure C++ recommandée

```cpp
struct SystemStatus {
    std::optional<double> timestamp;       // présent seulement en format périodique
    std::optional<std::string> message;    // présent seulement à la connexion
    std::optional<bool> connected;         // présent seulement à la connexion
    bool camera_connected = false;
    int clients_connected = 0;
};
```

### Exemple de parsing (gère les deux formats)

```cpp
SystemStatus parseStatus(const std::string& json_string) {
    SystemStatus status;
    json j = json::parse(json_string);
    
    if (j.contains("timestamp") && !j["timestamp"].is_null())
        status.timestamp = j["timestamp"].get<double>();
    if (j.contains("message") && j["message"].is_string())
        status.message = j["message"].get<std::string>();
    if (j.contains("connected") && !j["connected"].is_null())
        status.connected = j["connected"].get<bool>();
    
    status.camera_connected = j.value("camera_connected", false);
    status.clients_connected = j.value("clients_connected", 0);
    
    return status;
}
```

## Event: `pong`

### Structure JSON

```json
{
  "timestamp": 1234567890.123
}
```

### Exemple de parsing

```cpp
double parsePong(const std::string& json_string) {
    json j = json::parse(json_string);
    return j["timestamp"].get<double>();
}
```

## Exemple complet avec socket.io-client-cpp

```cpp
#include <sio_client.h>
#include <sio_socket.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <string>

using namespace sio;
using json = nlohmann::json;

class RobocarClient {
private:
    sio::client client;
    sio::socket::ptr socket;
    
public:
    void connect(const std::string& url) {
        // Gestion de la connexion
        client.set_open_listener([this]() {
            std::cout << "Connecté au serveur" << std::endl;
        });
        
        client.set_close_listener([](sio::client::close_reason const& reason) {
            std::cout << "Déconnecté du serveur" << std::endl;
        });
        
        client.set_fail_listener([]() {
            std::cerr << "Échec de connexion" << std::endl;
        });
        
        // Connexion
        client.connect(url);
        socket = client.socket();
        
        // Écouter les events
        setupEventHandlers();
    }
    
    void setupEventHandlers() {
        // Event: sensor_data
        socket->on("sensor_data", [this](sio::event& ev) {
            std::string json_string = ev.get_message()->get_string();
            SensorData data = parseSensorData(json_string);
            onSensorData(data);
        });
        
        // Event: status
        socket->on("status", [this](sio::event& ev) {
            std::string json_string = ev.get_message()->get_string();
            SystemStatus status = parseStatus(json_string);
            onStatus(status);
        });
        
        // Event: pong
        socket->on("pong", [this](sio::event& ev) {
            std::string json_string = ev.get_message()->get_string();
            double timestamp = parsePong(json_string);
            std::cout << "Pong reçu: " << timestamp << std::endl;
        });
    }
    
    void sendPing() {
        socket->emit("ping");
    }
    
    void disconnect() {
        client.disconnect();
    }
    
private:
    void onSensorData(const SensorData& data) {
        std::cout << "=== Sensor Data ===" << std::endl;
        std::cout << "Timestamp: " << data.timestamp << std::endl;
        
        if (!data.camera.frame_base64.empty()) {
            std::cout << "Camera: " << data.camera.width << "x" << data.camera.height 
                     << " (" << data.camera.format << ")" << std::endl;
            cv::Mat image = decodeCameraImage(data.camera);
            if (!image.empty()) std::cout << "Image décodée avec succès" << std::endl;
        }
    }
    
    void onStatus(const SystemStatus& status) {
        std::cout << "=== Status ===" << std::endl;
        if (status.message.has_value()) std::cout << "Message: " << *status.message << std::endl;
        if (status.timestamp.has_value()) std::cout << "Timestamp: " << *status.timestamp << std::endl;
        std::cout << "Camera connectée: " << (status.camera_connected ? "Oui" : "Non") << std::endl;
        std::cout << "Clients connectés: " << status.clients_connected << std::endl;
    }
    
    // Fonctions de parsing (voir exemples ci-dessus)
    SensorData parseSensorData(const std::string& json_string);
    SystemStatus parseStatus(const std::string& json_string);
    double parsePong(const std::string& json_string);
    cv::Mat decodeCameraImage(const CameraData& camera_data);
};

// Exemple d'utilisation
int main() {
    RobocarClient client;
    client.connect("http://localhost:3000");
    
    // Attendre les messages
    std::this_thread::sleep_for(std::chrono::seconds(60));
    
    // Envoyer un ping toutes les 5 secondes
    std::thread ping_thread([&client]() {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            client.sendPing();
        }
    });
    
    ping_thread.join();
    client.disconnect();
    return 0;
}
```

## Gestion des erreurs

### Vérifications recommandées

```cpp
SensorData parseSensorData(const std::string& json_string) {
    SensorData data;
    
    try {
        json j = json::parse(json_string);
        
        // Vérifier que le JSON est valide
        if (!j.is_object()) {
            throw std::runtime_error("JSON invalide: pas un objet");
        }
        
        // Parser timestamp (obligatoire)
        if (!j.contains("timestamp")) {
            throw std::runtime_error("Timestamp manquant");
        }
        data.timestamp = j["timestamp"].get<double>();
        
        // Parser caméra (optionnel)
        if (j.contains("camera") && !j["camera"].is_null()) {
            json camera_json = j["camera"];
            if (camera_json.contains("frame")) data.camera.frame_base64 = camera_json["frame"].get<std::string>();
            data.camera.width = camera_json.value("width", 0);
            data.camera.height = camera_json.value("height", 0);
            data.camera.format = camera_json.value("format", "jpeg");
        }
        
    } catch (const json::parse_error& e) {
        std::cerr << "Erreur de parsing JSON: " << e.what() << std::endl;
        throw;
    } catch (const json::type_error& e) {
        std::cerr << "Erreur de type JSON: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "Erreur: " << e.what() << std::endl;
        throw;
    }
    
    return data;
}
```

## Notes importantes

### Types de données

- **timestamp** : `double` (secondes Unix avec décimales)
- **frame** : `std::string` (chaîne base64)
- **width/height** : `int` (pixels)

### Status : deux formats

- **À la connexion** : champs `message` et `connected` uniquement (pas de timestamp ni *_connected).
- **Périodique** : champs `timestamp`, `camera_connected`, `clients_connected`. Le parser doit gérer les champs optionnels (ex. `timestamp` absent à la connexion).

### Fréquence de réception

- Les messages `sensor_data` sont envoyés à **10 Hz** (toutes les 100ms)
- Les messages `status` sont envoyés à **10 Hz** également
- Les messages peuvent arriver de manière asynchrone

### Gestion de la mémoire

- Les images caméra peuvent être volumineuses (~50-100 KB en JPEG)
- Pensez à libérer la mémoire après traitement
- Utilisez des pointeurs intelligents (`std::shared_ptr`, `std::unique_ptr`) si nécessaire

### Performance

- Le parsing JSON peut être coûteux pour de gros messages
- Considérez l'utilisation d'un thread séparé pour le parsing
- Les images doivent être décodées dans un thread séparé pour ne pas bloquer la réception

## Exemple CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(robocar_client)

set(CMAKE_CXX_STANDARD 17)

# Trouver les dépendances
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# Ajouter socket.io-client-cpp (à adapter selon votre installation)
# git submodule add https://github.com/socketio/socket.io-client-cpp.git
add_subdirectory(socket.io-client-cpp)

# Ajouter nlohmann/json (header-only)
# git submodule add https://github.com/nlohmann/json.git
add_subdirectory(json)

# Sources
add_executable(robocar_client
    src/main.cpp
    src/robocar_client.cpp
)

# Inclure les répertoires
target_include_directories(robocar_client PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    socket.io-client-cpp/src
    json/include
)

# Lier les bibliothèques
target_link_libraries(robocar_client
    ${OpenCV_LIBS}
    sioclient
)
```

## Tests unitaires recommandés

```cpp
#include <cassert>
#include <iostream>

void testParseSensorData() {
    std::string json = R"({
        "timestamp": 1234567890.123,
        "camera": {
            "frame": "base64_string_here",
            "width": 320,
            "height": 180,
            "format": "jpeg"
        }
    })";
    
    SensorData data = parseSensorData(json);
    
    assert(data.timestamp == 1234567890.123);
    assert(data.camera.width == 320);
    assert(data.camera.height == 180);
    assert(data.camera.format == "jpeg");
    
    std::cout << "Test parseSensorData: OK" << std::endl;
}
```

## Support

Pour toute question sur le protocole ou l'implémentation, référez-vous à :
- Documentation socket.io : https://socket.io/docs/
- Documentation nlohmann/json : https://json.nlohmann.me/
- Documentation OpenCV : https://docs.opencv.org/
