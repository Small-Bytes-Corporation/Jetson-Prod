"""
Device discovery: identify which serial port is Lidar, RTK, camera (DepthAI), etc.
Uses USB info (by-id, sysfs) and optional protocol probing.
"""

import os
import glob
import time

from .config import (
    IS_LINUX,
    LIDAR_BAUDRATE,
    RTK_BAUDRATE,
    _get_available_serial_ports,
)

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False

try:
    from fusion_engine_client.parsers.decoder import FusionEngineDecoder
    _FUSION_ENGINE_AVAILABLE = True
except ImportError:
    _FUSION_ENGINE_AVAILABLE = False

_VESC_AVAILABLE = False  # Will be set to True in _probe_vesc if available


def _get_serial_by_id_name(port_path):
    """
    On Linux, return the /dev/serial/by-id/ link name that points to this port, if any.
    E.g. "usb-FTDI_FT232R_USB_UART_AB0CDEF0-if00-port0"
    """
    if not IS_LINUX or not os.path.exists("/dev/serial/by-id"):
        return None
    try:
        for name in os.listdir("/dev/serial/by-id"):
            link = os.path.join("/dev/serial/by-id", name)
            if not os.path.islink(link):
                continue
            target = os.path.realpath(link)
            if target == port_path:
                return name
    except OSError:
        pass
    return None


def _get_sysfs_usb_info(port_path):
    """
    On Linux, read USB vendor/product from sysfs for this tty device.
    Returns dict with idVendor, idProduct, product, manufacturer if readable.
    """
    if not IS_LINUX:
        return {}
    # port_path is e.g. /dev/ttyUSB0 -> base name ttyUSB0
    base = os.path.basename(port_path)
    sysfs_base = os.path.join("/sys/class/tty", base)
    if not os.path.exists(sysfs_base):
        return {}
    device_dir = os.path.join(sysfs_base, "device")
    if not os.path.exists(device_dir):
        return {}
    # USB device is usually at device/../ or device/../..
    out = {}
    for parent in [os.path.join(device_dir, ".."), os.path.join(device_dir, "..", "..")]:
        parent = os.path.normpath(parent)
        for key, filename in [("idVendor", "idVendor"), ("idProduct", "idProduct"), ("product", "product"), ("manufacturer", "manufacturer")]:
            path = os.path.join(parent, filename)
            if key in out:
                continue
            try:
                with open(path, "r") as f:
                    out[key] = f.read().strip()
            except (OSError, IOError):
                pass
    return out


def _probe_lidar(port_path, read_duration=0.2):
    """
    Open port at LIDAR_BAUDRATE, read for read_duration seconds, look for D500/LD19 header 0x54 0x2C.
    Returns True if at least one such header is found.
    """
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
            # Search for 0x54 0x2C
            for i in range(len(data) - 1):
                if data[i] == 0x54 and data[i + 1] == 0x2C:
                    return True
    except Exception:
        pass
    return False


def _probe_rtk(port_path, read_duration=0.3):
    """
    Open port at RTK_BAUDRATE, read for read_duration seconds, try to decode Fusion Engine messages.
    Returns True if at least one valid message is decoded.
    """
    if not _SERIAL_AVAILABLE or not _FUSION_ENGINE_AVAILABLE:
        return False
    try:
        decoder = FusionEngineDecoder(warn_on_error=False)
        with serial.Serial(port=port_path, baudrate=RTK_BAUDRATE, timeout=0.1) as ser:
            deadline = time.monotonic() + read_duration
            while time.monotonic() < deadline:
                if ser.in_waiting:
                    chunk = ser.read(ser.in_waiting)
                    results = decoder.on_data(chunk)
                    for r in results:
                        if len(r) >= 2:
                            return True
                time.sleep(0.02)
    except Exception:
        pass
    return False


def _probe_vesc(port_path, timeout=2.0):
    """
    Try to connect to VESC and get firmware version.
    Returns True if VESC responds successfully.
    """
    if not _SERIAL_AVAILABLE:
        return False
    # Try to import VESC on demand
    try:
        from pyvesc import VESC
    except ImportError:
        return False
    
    motor = None
    try:
        motor = VESC(serial_port=port_path)
        version = motor.get_firmware_version()
        # If we got here without exception, it's likely a VESC
        if version is not None:
            return True
    except Exception:
        pass
    finally:
        # Clean up VESC connection
        if motor is not None:
            try:
                motor.stop_heartbeat()
            except Exception:
                pass
    return False


def _identify_by_usb_info(usb_info, by_id):
    """
    Identify device type based on USB vendor/product info and by-id name.
    Returns device type string or None if unknown.
    """
    if not usb_info and not by_id:
        return None
    
    manufacturer = (usb_info.get("manufacturer") or "").lower()
    product = (usb_info.get("product") or "").lower()
    by_id_lower = (by_id or "").lower()
    
    # RTK GNSS - Silicon Labs CP2105
    if "silicon" in manufacturer or "cp2105" in product or "cp2105" in by_id_lower:
        return "RTK GNSS (Point One / Fusion Engine)"
    
    # Arduino - typically Pan/Tilt controller
    if "arduino" in manufacturer or "arduino" in product or "arduino" in by_id_lower:
        return "Pan/Tilt (Arduino)"
    
    # STMicroelectronics ChibiOS - typically VESC
    if "stmicroelectronics" in manufacturer or "chibios" in product or "chibios" in by_id_lower:
        return "VESC (STMicroelectronics)"
    
    return None


def get_serial_port_info(port_path, probe=True):
    """
    Return a dict with path, by_id name (Linux), sysfs USB info (Linux), and probable device (if probe=True).
    Uses USB info first, then protocol probing.
    """
    info = {
        "path": port_path,
        "by_id": _get_serial_by_id_name(port_path),
        "usb": _get_sysfs_usb_info(port_path),
        "probable": None,
    }
    
    # First try USB-based identification (fast, no port access needed)
    usb_identified = _identify_by_usb_info(info["usb"], info["by_id"])
    
    if probe and _SERIAL_AVAILABLE:
        # Protocol probing (slower, requires exclusive port access)
        if _probe_lidar(port_path):
            info["probable"] = "Lidar (D500/LD19)"
        elif _probe_rtk(port_path):
            info["probable"] = "RTK GNSS (Point One / Fusion Engine)"
        elif _probe_vesc(port_path):
            info["probable"] = "VESC (STMicroelectronics)"
        elif usb_identified:
            # Use USB identification if probing didn't find anything
            info["probable"] = usb_identified
        else:
            info["probable"] = "Serial (VESC, Pan/Tilt ou autre)"
    elif usb_identified:
        # If not probing, use USB identification
        info["probable"] = usb_identified
    elif not probe:
        info["probable"] = "Serial (VESC, Pan/Tilt ou autre)"
    
    return info


def get_detected_port_mapping(probe=True):
    """
    Identify which serial port is Lidar, RTK, VESC, Pan/Tilt, etc. via probe (or USB info only if probe=False).
    Returns a dict with keys "lidar", "rtk", "vesc", "pan_tilt" -> port path.
    Only includes keys for device types that were detected.
    """
    results = list_devices(probe=probe, verbose=False)
    mapping = {}
    for info in results.get("serial", []):
        path = info.get("path")
        probable = info.get("probable")
        if not path or not probable:
            continue
        if probable == "Lidar (D500/LD19)":
            mapping["lidar"] = path
        elif probable == "RTK GNSS (Point One / Fusion Engine)":
            mapping["rtk"] = path
        elif probable == "VESC (STMicroelectronics)":
            mapping["vesc"] = path
        elif probable == "Pan/Tilt (Arduino)":
            mapping["pan_tilt"] = path
    return mapping


def get_depthai_devices():
    """
    Return list of connected DepthAI/OAK devices, if the library is available.
    """
    try:
        import depthai as dai
    except ImportError:
        return []
    devices = []
    try:
        # New API (depthai 3.x)
        if hasattr(dai, "DeviceBootloader") and hasattr(dai.DeviceBootloader, "getAllAvailableDevices"):
            for info in dai.DeviceBootloader.getAllAvailableDevices():
                devices.append({
                    "name": getattr(info, "name", str(info)),
                    "mxid": getattr(info, "mxid", ""),
                    "state": getattr(info, "state", ""),
                })
        # Fallback: try to get first device by booting (less ideal)
        elif hasattr(dai, "Device") and hasattr(dai.Device, "getAllAvailableDevices"):
            for dev in dai.Device.getAllAvailableDevices():
                devices.append({
                    "name": getattr(dev, "getMxId", lambda: "")(),
                    "mxid": getattr(dev, "getMxId", lambda: "")(),
                    "state": getattr(dev, "state", ""),
                })
    except Exception as e:
        import sys
        print(f"[DeviceDiscovery] DepthAI list failed: {e}", file=sys.stderr)
    return devices


def list_devices(probe=True, verbose=True):
    """
    Print a summary of connected devices: serial ports (with USB info and optional probe)
    and DepthAI cameras. If verbose, print to stdout; returns structured data in any case.
    """
    ports = _get_available_serial_ports()
    results = {"serial": [], "camera": []}

    if verbose:
        print("=== Périphériques série (Lidar, RTK, VESC, Pan/Tilt) ===\n")
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
