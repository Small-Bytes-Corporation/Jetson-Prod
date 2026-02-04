#!/usr/bin/env python3
"""
Script de configuration RTK pour Point One Navigation (Quectel LG69T).
Affiche les informations de configuration Polaris et fournit une interface pour configurer le récepteur.

Note: La configuration du récepteur RTK se fait généralement dans le firmware du récepteur.
Ce script sert principalement de référence et d'aide à la configuration.

Usage:
    python scripts/configure_rtk.py [--port /dev/ttyUSB0] [--show-config] [--info]
"""

import argparse
import os
import sys
import time

# Ensure project root is on path
if __name__ == "__main__":
    _root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _root not in sys.path:
        sys.path.insert(0, _root)

# Import config directly to avoid loading drive.core.__init__ which imports pygame
import importlib.util
_config_path = os.path.join(_root, 'drive', 'core', 'config.py')
spec = importlib.util.spec_from_file_location("config", _config_path)
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)

DEFAULT_RTK_SERIAL_PORT = config.DEFAULT_RTK_SERIAL_PORT
RTK_BAUDRATE = config.RTK_BAUDRATE
POLARIS_API_KEY = config.POLARIS_API_KEY
POLARIS_NTRIP_HOST = config.POLARIS_NTRIP_HOST
POLARIS_NTRIP_PORT = config.POLARIS_NTRIP_PORT
POLARIS_NTRIP_MOUNT_POINT = config.POLARIS_NTRIP_MOUNT_POINT

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False


def print_polaris_config():
    """Affiche la configuration Polaris depuis config.py."""
    print("=" * 70)
    print("CONFIGURATION POLARIS RTK")
    print("=" * 70)
    print(f"API Key: {POLARIS_API_KEY[:8]}...{POLARIS_API_KEY[-8:]} (masqué)")
    print(f"NTRIP Host: {POLARIS_NTRIP_HOST}")
    print(f"NTRIP Port: {POLARIS_NTRIP_PORT} ({'TLS encrypted' if POLARIS_NTRIP_PORT == 2102 else 'plaintext'})")
    print(f"Mount Point: {POLARIS_NTRIP_MOUNT_POINT}")
    print("=" * 70)
    print()


def print_connection_info(port, baudrate):
    """Affiche les informations de connexion."""
    print(f"Port série: {port}")
    print(f"Baudrate: {baudrate}")
    print()


def check_rtk_connection(port, baudrate, timeout=3.0):
    """
    Vérifie la connexion au récepteur RTK en lisant quelques messages.
    
    Returns:
        tuple: (success: bool, message_count: int, error_message: str)
    """
    if not _SERIAL_AVAILABLE:
        return False, 0, "pyserial non disponible (pip install pyserial)"
    
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        print(f"[RTK] Connexion établie sur {port} @ {baudrate}")
        
        # Essayer de lire des messages pendant timeout secondes
        deadline = time.monotonic() + timeout
        message_count = 0
        bytes_read = 0
        
        while time.monotonic() < deadline:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                bytes_read += len(data)
                message_count += 1
            time.sleep(0.1)
        
        ser.close()
        
        if bytes_read > 0:
            print(f"[RTK] ✓ Récepteur RTK détecté ({message_count} paquets reçus, {bytes_read} bytes)")
            return True, message_count, None
        else:
            print(f"[RTK] ⚠ Aucune donnée reçue du récepteur RTK")
            print(f"[RTK]   Vérifiez que le récepteur est bien connecté et alimenté")
            return False, 0, "Aucune donnée reçue"
            
    except serial.SerialException as e:
        return False, 0, f"Erreur de connexion série: {e}"
    except Exception as e:
        return False, 0, f"Erreur: {e}"


def print_ntrip_config_instructions():
    """Affiche les instructions pour configurer NTRIP sur le récepteur."""
    print("=" * 70)
    print("INSTRUCTIONS DE CONFIGURATION POLARIS RTK")
    print("=" * 70)
    print()
    print("⚠️  IMPORTANT: Le récepteur RTK doit être configuré dans son firmware")
    print("   pour utiliser la clé Polaris. Le code Python ne configure pas")
    print("   automatiquement le récepteur - il lit seulement les données.")
    print()
    print("Pour configurer le récepteur RTK Point One avec Polaris:")
    print()
    print("1. CONNEXION INTERNET")
    print("   Le récepteur RTK doit être connecté à Internet (Wi-Fi ou cellulaire)")
    print()
    print("2. MÉTHODE RECOMMANDÉE: Outils Point One (P1 Host Tools)")
    print("   Installez et utilisez les outils Point One:")
    print("   git clone https://github.com/PointOneNav/p1-host-tools.git")
    print("   cd p1-host-tools")
    print("   # Si vous utilisez conda (recommandé):")
    print("   pip install -r requirements.txt")
    print("   # Ou si vous préférez un venv:")
    print("   # python -m venv venv && source venv/bin/activate && pip install -r requirements.txt")
    print()
    print("   Ensuite, utilisez runner.py avec votre ID Polaris:")
    print(f"   python bin/runner.py --device-id VOTRE_DEVICE_ID --polaris {POLARIS_API_KEY}")
    print()
    print("   Ou utilisez config_tool.py pour configurer le récepteur:")
    print("   python bin/config_tool.py")
    print()
    print("3. CONFIGURATION NTRIP (alternative)")
    print("   Si vous configurez manuellement via NTRIP, utilisez:")
    print(f"   - Caster URL: {POLARIS_NTRIP_HOST}")
    print(f"   - Port: {POLARIS_NTRIP_PORT}")
    print(f"   - Mount Point: {POLARIS_NTRIP_MOUNT_POINT}")
    print(f"   - API Key/ID Polaris: {POLARIS_API_KEY}")
    print()
    print("   Méthodes de configuration:")
    print("   - Via l'interface web du récepteur (si disponible)")
    print("   - Via les outils Point One (P1 Host Tools) - RECOMMANDÉ")
    print("   - Via les commandes AT (si supportées par le firmware)")
    print()
    print("4. VÉRIFICATION")
    print("   Après configuration, utilisez:")
    print("   python scripts/test_rtk.py --port /dev/ttyUSB0")
    print("   pour vérifier que le récepteur obtient une solution RTK valide.")
    print("   Vous devriez voir 'solution_type: RTK_FIX' ou 'RTK_FLOAT' au lieu de 'Invalid'.")
    print()
    print("5. RESSOURCES")
    print("   - Documentation Point One: https://support.pointonenav.com/")
    print("   - P1 Host Tools: https://github.com/PointOneNav/p1-host-tools")
    print(f"   - Votre ID Polaris: {POLARIS_API_KEY}")
    print()
    print("=" * 70)
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Configuration RTK Point One Navigation - Affiche les informations Polaris et vérifie la connexion"
    )
    parser.add_argument(
        "--port",
        type=str,
        default=DEFAULT_RTK_SERIAL_PORT,
        help=f"Port série pour le récepteur RTK (default: {DEFAULT_RTK_SERIAL_PORT})",
    )
    parser.add_argument(
        "--show-config",
        action="store_true",
        help="Afficher la configuration Polaris depuis config.py",
    )
    parser.add_argument(
        "--info",
        action="store_true",
        help="Afficher les informations de connexion et de configuration",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Vérifier la connexion au récepteur RTK",
    )
    parser.add_argument(
        "--instructions",
        action="store_true",
        help="Afficher les instructions de configuration NTRIP",
    )
    
    args = parser.parse_args()
    
    # Par défaut, afficher tout si aucune option spécifique n'est donnée
    show_all = not (args.show_config or args.info or args.check or args.instructions)
    
    if show_all or args.show_config:
        print_polaris_config()
    
    if show_all or args.info:
        print_connection_info(args.port, RTK_BAUDRATE)
    
    if show_all or args.check:
        print("Vérification de la connexion au récepteur RTK...")
        print()
        success, msg_count, error = check_rtk_connection(args.port, RTK_BAUDRATE)
        if error:
            print(f"[RTK] ✗ {error}")
        print()
    
    if show_all or args.instructions:
        print_ntrip_config_instructions()
    
    if not show_all and not args.show_config and not args.info and not args.check and not args.instructions:
        print("Utilisez --help pour voir les options disponibles")
        print()


if __name__ == "__main__":
    main()
