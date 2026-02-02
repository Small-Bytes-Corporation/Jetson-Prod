# Architecture du système de contrôle Robocar

## Structure modulaire

Le code a été refactorisé en classes et modules pour une meilleure organisation et maintenabilité.

### Structure des dossiers

```
drive/
├── core/                    # Modules de base réutilisables
│   ├── motor_controller.py      # Contrôle du moteur VESC
│   ├── joystick_controller.py   # Gestion du joystick/gamepad
│   ├── throttle_controller.py  # Gestion de l'accélération/freinage
│   ├── camera_controller.py    # Capture de frames caméra (sans IA)
│   └── config.py               # Constantes centralisées
├── applications/            # Applications métier
│   └── manual_drive.py         # Conduite manuelle
└── main.py                  # Point d'entrée principal
```

## Classes principales

### MotorController
Gère l'initialisation et le contrôle du moteur VESC avec retry automatique.

### JoystickController
Gère les inputs du joystick/gamepad via pygame. Fournit des méthodes pour lire les boutons et axes.

### ThrottleController
Calcule l'accélération cible depuis les triggers et applique une interpolation progressive pour des transitions fluides.

### CameraController
Capture des frames brutes depuis la caméra DepthAI. Pas de traitement IA, juste la capture.

### ManualDriveApp
Application de conduite manuelle qui orchestre tous les contrôleurs dans une boucle propre.

## Utilisation

```bash
python3 main.py [--max-speed FLOAT] [--serial-port PATH] [--camera]
```

## Boucle principale

Chaque application utilise une boucle principale propre qui :
1. Met à jour les inputs (joystick)
2. Calcule les commandes (throttle)
3. Applique les commandes (motor)
4. Récupère les frames caméra si nécessaire
5. Gère les événements (exit, etc.)
6. Contrôle la fréquence avec sleep

## Notes

- Le système n'inclut plus de traitement IA (segmentation, raycasting, etc.).
- La caméra capture uniquement des frames brutes pour affichage/enregistrement.
- Tous les fichiers liés à l'entraînement, l'agent et l'enregistrement ont été supprimés pour simplifier le codebase.
