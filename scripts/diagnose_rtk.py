#!/usr/bin/env python3
"""
Script de diagnostic RTK pour comprendre pourquoi les valeurs sont NaN.
Vérifie la connexion série, les messages reçus, et le statut de la solution GNSS.
"""

import argparse
import os
import sys
import time
import math

# Ensure project root is on path
if __name__ == "__main__":
    _root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _root not in sys.path:
        sys.path.insert(0, _root)

from drive.core.rtk_controller import RTKController
from drive.core.config import DEFAULT_RTK_SERIAL_PORT


def check_nan(value):
    """Check if value is NaN."""
    if value is None:
        return True
    try:
        if hasattr(value, "__iter__") and not isinstance(value, (str, bytes)):
            return any(x != x for x in value if x is not None)
        else:
            return value != value
    except (TypeError, ValueError):
        return False


def format_value(value, decimals=6):
    """Format a value, showing NaN clearly."""
    if value is None:
        return "None"
    if check_nan(value):
        return "NaN"
    try:
        if hasattr(value, "__iter__") and not isinstance(value, (str, bytes)):
            return "[" + ", ".join(format_value(v, decimals) for v in value) + "]"
        else:
            return f"{float(value):.{decimals}f}"
    except (TypeError, ValueError):
        return str(value)


def main():
    parser = argparse.ArgumentParser(description="Diagnostic RTK - vérifier pourquoi les valeurs sont NaN")
    parser.add_argument(
        "--port",
        type=str,
        default=DEFAULT_RTK_SERIAL_PORT,
        help=f"Port série RTK (default: {DEFAULT_RTK_SERIAL_PORT})",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Durée du diagnostic en secondes (default: 30)",
    )
    args = parser.parse_args()

    print("=" * 70)
    print("DIAGNOSTIC RTK")
    print("=" * 70)
    print(f"Port série: {args.port}")
    print(f"Durée: {args.duration} secondes")
    print()
    print("Vérifications à faire si vous voyez des NaN:")
    print("  1. Le récepteur RTK a-t-il une vue du ciel? (pas à l'intérieur)")
    print("  2. Le récepteur RTK est-il connecté au réseau de correction RTK?")
    print("  3. Le récepteur RTK a-t-il besoin de temps pour converger? (cold start)")
    print("  4. Le firmware du récepteur RTK est-il à jour?")
    print()
    print("=" * 70)
    print()

    rtk = RTKController(serial_port=args.port, enabled=True)
    if not rtk.initialize():
        print("ERREUR: Impossible d'initialiser le contrôleur RTK")
        sys.exit(1)

    start_time = time.time()
    last_pose_time = None
    last_imu_time = None
    pose_count = 0
    imu_count = 0
    solution_types_seen = set()

    print("En attente de messages RTK...")
    print()

    try:
        while time.time() - start_time < args.duration:
            rtk.update()
            
            pose = rtk.get_latest_pose()
            if pose:
                pose_count += 1
                solution_type = pose.get("solution_type", "Unknown")
                solution_types_seen.add(str(solution_type))
                
                # Print pose info every 2 seconds
                now = time.time()
                if last_pose_time is None or now - last_pose_time >= 2.0:
                    last_pose_time = now
                    
                    print(f"[{now - start_time:.1f}s] POSE #{pose_count}")
                    print(f"  solution_type: {solution_type}")
                    
                    lla = pose.get("lla_deg")
                    if lla is not None:
                        has_nan = check_nan(lla)
                        print(f"  lla_deg: {format_value(lla, 6)} {'⚠️ NaN détecté!' if has_nan else ''}")
                    
                    std = pose.get("position_std_enu_m")
                    if std is not None:
                        has_nan = check_nan(std)
                        print(f"  position_std_enu_m: {format_value(std, 2)} {'⚠️ NaN détecté!' if has_nan else ''}")
                    
                    gps_time = pose.get("gps_time")
                    p1_time = pose.get("p1_time")
                    print(f"  gps_time: {format_value(gps_time, 3)}")
                    print(f"  p1_time: {format_value(p1_time, 3)}")
                    
                    ypr = pose.get("ypr_deg")
                    if ypr is not None:
                        has_nan = check_nan(ypr)
                        print(f"  ypr_deg: {format_value(ypr, 2)} {'⚠️ NaN détecté!' if has_nan else ''}")
                    
                    vel = pose.get("velocity_body_mps")
                    if vel is not None:
                        has_nan = check_nan(vel)
                        print(f"  velocity_body_mps: {format_value(vel, 2)} {'⚠️ NaN détecté!' if has_nan else ''}")
                    
                    print()
            
            imu = rtk.get_latest_imu()
            if imu:
                imu_count += 1
                now = time.time()
                if last_imu_time is None or now - last_imu_time >= 2.0:
                    last_imu_time = now
                    print(f"[{now - start_time:.1f}s] IMU #{imu_count}")
                    acc = imu.get("accel_xyz")
                    if acc is not None:
                        print(f"  accel_xyz (m/s²): {format_value(acc, 3)}")
                    gyro = imu.get("gyro_xyz")
                    if gyro is not None:
                        print(f"  gyro_xyz (rad/s): {format_value(gyro, 5)}")
                    print()
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nInterrompu par l'utilisateur")
    
    finally:
        rtk.stop()
        
        print()
        print("=" * 70)
        print("RÉSUMÉ DU DIAGNOSTIC")
        print("=" * 70)
        print(f"Durée totale: {time.time() - start_time:.1f} secondes")
        print(f"Messages POSE reçus: {pose_count}")
        print(f"Messages IMU reçus: {imu_count}")
        print(f"Types de solution vus: {', '.join(sorted(solution_types_seen))}")
        print()
        
        if pose_count == 0:
            print("⚠️  AUCUN message POSE reçu!")
            print("   → Vérifiez la connexion série")
            print("   → Vérifiez que le récepteur RTK envoie bien des messages Fusion Engine")
        elif "Invalid" in str(solution_types_seen) or "INVALID" in str(solution_types_seen):
            print("⚠️  Solution INVALID détectée")
            print("   → Le récepteur RTK n'a pas de solution GNSS valide")
            print("   → Causes possibles:")
            print("     - Pas de vue du ciel (récepteur à l'intérieur)")
            print("     - Pas de satellites visibles")
            print("     - Pas de correction RTK disponible")
            print("     - Récepteur en cours d'initialisation (cold start)")
        else:
            print("✓ Messages reçus avec succès")
        
        print()


if __name__ == "__main__":
    main()
