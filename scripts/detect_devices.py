#!/usr/bin/env python3
"""
Script standalone pour détecter les périphériques (ports série + caméras DepthAI).
Indépendant du main : aucune dépendance au package drive. Liste Lidar, VESC, Pan/Tilt, OAK-D.

Usage:
  python scripts/detect_devices.py           # détection complète (USB + sondage protocole)
  python scripts/detect_devices.py --no-probe   # identification USB uniquement (rapide)
  python scripts/detect_devices.py --json   # sortie JSON
"""

import argparse
import glob
import json
import os
import platform
import sys
import time

# --- Constantes (alignées avec drive/core) ---
IS_LINUX = platform.system() == "Linux"
LIDAR_BAUDRATE = 230400

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False


def _get_available_serial_ports():
    """Liste des ports série disponibles (Linux: ttyUSB*, ttyACM* ; Mac: tty.*)."""
    ports = []
    if platform.system() == "Darwin":
        for pattern in ["/dev/tty.usbserial-*", "/dev/tty.usbmodem*"]:
            ports.extend(glob.glob(pattern))
        for pattern in ["/dev/cu.usbserial-*", "/dev/cu.usbmodem*"]:
            ports.extend(glob.glob(pattern))
    else:
        for pattern in ["/dev/ttyUSB*", "/dev/ttyACM*"]:
            ports.extend(glob.glob(pattern))
    ports = [p for p in ports if os.path.exists(p)]
    return sorted(ports)


def _get_serial_by_id_name(port_path):
    """Sous Linux, nom du lien by-id pointant vers ce port."""
    if not IS_LINUX or not os.path.exists("/dev/serial/by-id"):
        return None
    try:
        for name in os.listdir("/dev/serial/by-id"):
            link = os.path.join("/dev/serial/by-id", name)
            if not os.path.islink(link):
                continue
            if os.path.realpath(link) == port_path:
                return name
    except OSError:
        pass
    return None


def _get_sysfs_usb_info(port_path):
    """Sous Linux, infos USB (vendor/product) depuis sysfs."""
    if not IS_LINUX:
        return {}
    base = os.path.basename(port_path)
    sysfs_base = os.path.join("/sys/class/tty", base)
    if not os.path.exists(sysfs_base):
        return {}
    device_dir = os.path.join(sysfs_base, "device")
    if not os.path.exists(device_dir):
        return {}
    out = {}
    for parent in [os.path.join(device_dir, ".."), os.path.join(device_dir, "..", "..")]:
        parent = os.path.normpath(parent)
        for key, filename in [
            ("idVendor", "idVendor"),
            ("idProduct", "idProduct"),
            ("product", "product"),
            ("manufacturer", "manufacturer"),
        ]:
            if key in out:
                continue
            path = os.path.join(parent, filename)
            try:
                with open(path, "r") as f:
                    out[key] = f.read().strip()
            except (OSError, IOError):
                pass
    return out


def _identify_by_usb_info(usb_info, by_id):
    """Type de périphérique à partir des infos USB / by-id."""
    if not usb_info and not by_id:
        return None
    manufacturer = (usb_info.get("manufacturer") or "").lower()
    product = (usb_info.get("product") or "").lower()
    by_id_lower = (by_id or "").lower()
    if "arduino" in manufacturer or "arduino" in product or "arduino" in by_id_lower:
        return "Pan/Tilt (Arduino)"
    if "stmicroelectronics" in manufacturer or "chibios" in product or "chibios" in by_id_lower:
        return "VESC (STMicroelectronics)"
    return None


def _probe_lidar(port_path, read_duration=0.2):
    """Sonde le port pour un Lidar D500/LD19 (header 0x54 0x2C)."""
    if not _SERIAL_AVAILABLE:
        return False
    try:
        with serial.Serial(port=port_path, baudrate=LIDAR_BAUDRATE, timeout=0.1) as ser:
            deadline = time.monotonic() + read_duration
            data = bytearray()
            while time.monotonic() < deadline:
                if ser.in_waiting:
                    data.extend(ser.read(ser.in_waiting))
                time.sleep(0.02)
            for i in range(len(data) - 1):
                if data[i] == 0x54 and data[i + 1] == 0x2C:
                    return True
    except Exception:
        pass
    return False


def _probe_vesc(port_path):
    """Sonde le port pour un VESC (pyvesc optionnel)."""
    if not _SERIAL_AVAILABLE:
        return False
    try:
        from pyvesc import VESC
    except ImportError:
        return False
    motor = None
    try:
        motor = VESC(serial_port=port_path)
        if motor.get_firmware_version() is not None:
            return True
    except Exception:
        pass
    finally:
        if motor is not None:
            try:
                motor.stop_heartbeat()
            except Exception:
                pass
    return False


def get_serial_port_info(port_path, probe=True):
    """Infos d’un port : path, by_id, usb, probable (type détecté)."""
    info = {
        "path": port_path,
        "by_id": _get_serial_by_id_name(port_path),
        "usb": _get_sysfs_usb_info(port_path),
        "probable": None,
    }
    usb_identified = _identify_by_usb_info(info["usb"], info["by_id"])

    if probe and _SERIAL_AVAILABLE:
        if _probe_lidar(port_path):
            info["probable"] = "Lidar (D500/LD19)"
        elif _probe_vesc(port_path):
            info["probable"] = "VESC (STMicroelectronics)"
        elif usb_identified:
            info["probable"] = usb_identified
        else:
            info["probable"] = "Serial (VESC, Pan/Tilt ou autre)"
    elif usb_identified:
        info["probable"] = usb_identified
    else:
        info["probable"] = "Serial (VESC, Pan/Tilt ou autre)"

    return info


def get_depthai_devices():
    """Liste des appareils DepthAI/OAK connectés (depthai optionnel)."""
    try:
        import depthai as dai
    except ImportError:
        return []
    devices = []
    try:
        if hasattr(dai, "DeviceBootloader") and hasattr(dai.DeviceBootloader, "getAllAvailableDevices"):
            for info in dai.DeviceBootloader.getAllAvailableDevices():
                devices.append({
                    "name": getattr(info, "name", str(info)),
                    "mxid": getattr(info, "mxid", ""),
                    "state": getattr(info, "state", ""),
                })
        elif hasattr(dai, "Device") and hasattr(dai.Device, "getAllAvailableDevices"):
            for dev in dai.Device.getAllAvailableDevices():
                devices.append({
                    "name": getattr(dev, "getMxId", lambda: "")(),
                    "mxid": getattr(dev, "getMxId", lambda: "")(),
                    "state": getattr(dev, "state", ""),
                })
    except Exception as e:
        print(f"[detect_devices] DepthAI: {e}", file=sys.stderr)
    return devices


def list_devices(probe=True, verbose=True):
    """Détecte tous les périphériques ; si verbose, affiche un résumé. Retourne dict serial + camera."""
    ports = _get_available_serial_ports()
    results = {"serial": [], "camera": []}

    if verbose:
        print("=== Périphériques série (Lidar, VESC, Pan/Tilt) ===\n")
        if not ports:
            print("  Aucun port série trouvé (/dev/ttyUSB*, /dev/ttyACM*).\n")

    for port_path in ports:
        info = get_serial_port_info(port_path, probe=probe)
        results["serial"].append(info)
        if verbose:
            by_id = info["by_id"] or "(pas de by-id)"
            probable = info["probable"] or "(non sondé)"
            usb = info["usb"]
            usb_str = ""
            if usb:
                usb_str = "  USB: "
                if usb.get("manufacturer"):
                    usb_str += usb["manufacturer"] + " "
                if usb.get("product"):
                    usb_str += usb["product"] + " "
                if usb.get("idVendor") and usb.get("idProduct"):
                    usb_str += f"({usb['idVendor']}:{usb['idProduct']})"
            print(f"  {port_path}")
            print(f"    by-id: {by_id}")
            if usb_str:
                print(usb_str.strip())
            print(f"    Probable: {probable}\n")

    depthai_list = get_depthai_devices()
    results["camera"] = depthai_list
    if verbose:
        print("=== Caméra DepthAI / OAK-D ===\n")
        if not depthai_list:
            print("  Aucun appareil DepthAI détecté.\n")
        else:
            for d in depthai_list:
                print(f"  {d.get('name', d.get('mxid', 'OAK'))}  MxId: {d.get('mxid', '')}  State: {d.get('state', '')}\n")

    return results


def get_detected_port_mapping(probe=True):
    """Mapping lidar / vesc / pan_tilt -> chemin du port."""
    results = list_devices(probe=probe, verbose=False)
    mapping = {}
    for info in results.get("serial", []):
        path = info.get("path")
        probable = info.get("probable")
        if not path or not probable:
            continue
        if probable == "Lidar (D500/LD19)":
            mapping["lidar"] = path
        elif probable == "VESC (STMicroelectronics)":
            mapping["vesc"] = path
        elif probable == "Pan/Tilt (Arduino)":
            mapping["pan_tilt"] = path
    return mapping


def main():
    parser = argparse.ArgumentParser(
        description="Détecter les périphériques série (Lidar, VESC, Pan/Tilt) et caméras DepthAI."
    )
    parser.add_argument(
        "--no-probe",
        action="store_true",
        help="Ne pas sonder les ports (identification par USB uniquement, plus rapide)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Afficher le résultat en JSON",
    )
    parser.add_argument(
        "--mapping",
        action="store_true",
        help="Afficher uniquement le mapping lidar/vesc/pan_tilt -> port",
    )
    args = parser.parse_args()

    probe = not args.no_probe

    if args.mapping:
        mapping = get_detected_port_mapping(probe=probe)
        if args.json:
            print(json.dumps(mapping, indent=2))
        else:
            for key, port in sorted(mapping.items()):
                print(f"{key}: {port}")
        return

    results = list_devices(probe=probe, verbose=not args.json)

    if args.json:
        out = {
            "serial": [
                {
                    "path": i["path"],
                    "by_id": i["by_id"],
                    "probable": i["probable"],
                    "usb": i.get("usb", {}),
                }
                for i in results["serial"]
            ],
            "camera": results["camera"],
            "mapping": get_detected_port_mapping(probe=probe),
        }
        print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
