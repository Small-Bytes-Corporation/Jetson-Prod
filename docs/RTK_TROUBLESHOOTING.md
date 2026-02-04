# RTK Polaris – Dépannage

## Symptôme : Invalid / NaN dans les poses

Si vous voyez `solution_type: "Invalid"` et `lla_deg: [NaN, NaN, NaN]` :

### 1. Ordre de lancement

Lancer **d’abord** init_rtk, puis main :

```bash
# Terminal 1 (Python 3.10+)
python3 scripts/init_rtk.py --tcp 30201

# Terminal 2 (quand "corrections > 0" apparaît)
python3 main.py --enable-socket --rtk-tcp localhost:30201
```

### 2. Conditions d’environnement (Point One)

Source : [support.pointonenav.com](https://support.pointonenav.com/what-are-environmental-factors-that-affect-the-rtk-user-experience)

- **Vue du ciel** : l’antenne doit voir le ciel. Éviter l’intérieur, près des murs, sous les arbres.
- **Connexion Internet** : Polaris a besoin d’une connexion stable (Wi‑Fi ou 4G).
- **Antenne** : sur le toit ou à l’extérieur, loin des métaux et des écrans.

### 3. Fallback « dernière pose valide »

Quand la solution passe en Invalid (pont, tunnel, intérieur), le contrôleur RTK renvoie la **dernière pose valide** avec `solution_stale: true`. Les clients peuvent continuer à afficher la position, en sachant qu’elle n’est plus à jour.

### 4. Vérifications

- `corrections=X B` dans init_rtk : X > 0 signifie que Polaris envoie des corrections.
- Type de solution : `AutonomousGPS` puis `RTKFixed` / `RTKFloat` indiquent une bonne acquisition.
- Port : init_rtk utilise `--port` et main `--rtk-tcp` (voir `ports_config.py`).

### 5. Références

- [Point One Polaris](https://support.pointonenav.com/polaris)
- [Facteurs environnementaux RTK](https://support.pointonenav.com/what-are-environmental-factors-that-affect-the-rtk-user-experience)

---

## Symptôme : `imu` reste `null` dans sensor_data

Si vous avez des poses valides (`lla_deg`, `solution_type`) mais `rtk.imu` est toujours `null` :

### Causes possibles

1. **Firmware GNSS seul** : Certaines variantes LG69T (SDK-AM) n’ont pas d’IMU. Seules les variantes avec IMU intégré (SDK-AP, ex. LG69T-AP/AQ/AA/AJ) émettent des messages IMU.

2. **IMU désactivée par défaut** : Par défaut, le débit des messages IMU est OFF. Si votre module a une IMU, il faut l’activer :

```bash
cd p1-host-tools
python3 bin/config_tool.py apply uart1 message_rate fe imu 1s --port /dev/ttyUSB0
# Adapter --port selon ports_config.py (RTK_PORT)
python3 bin/config_tool.py save --port /dev/ttyUSB0  # persiste la config
```

Redémarrer main.py après avoir appliqué la config. Utiliser `1s` plutôt que `on_change` (plus fiable quand l’appareil est immobile).

3. **Rien à faire** : Si votre LG69T est en GNSS seul (sans IMU), `imu: null` est normal. La pose reste disponible.

### Vérifier si le module a une IMU

Consultez la référence exacte de votre module (LG69T-AM, LG69T-AP, etc.). Les variantes AP/AQ/AA/AJ incluent une IMU.
