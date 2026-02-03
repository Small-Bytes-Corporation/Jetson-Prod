#!/usr/bin/env python3
"""
Main entry point for the robocar control system.
Use --list-devices to list serial ports and DepthAI cameras (Lidar, RTK, etc.).
"""

import argparse
import sys
from drive.core.config import (
    MAX_SPEED, DEFAULT_SERIAL_PORT, DEFAULT_LIDAR_PORT, SOCKETIO_PORT,
    PAN_TILT_SERIAL_PORT, DEFAULT_RTK_SERIAL_PORT,
)

EPILOG = """
Examples:
  # Run manual drive (default: motor, joystick, throttle, pan/tilt, RTK)
  python3 main.py

  # Enable socket stream with camera (and optional lidar/RTK)
  python3 main.py --enable-socket --camera
  python3 main.py --enable-socket --camera --lidar-port /dev/ttyUSB0

  # List devices (identify Lidar, RTK, DepthAI cameras)
  python3 main.py --list-devices
  python3 main.py --list-devices --no-probe   # USB only, no protocol probe

  # Test without motor or pan/tilt
  python3 main.py --no-motor --no-pan-tilt --enable-socket
"""


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Robocar control system",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=EPILOG,
    )

    general = parser.add_argument_group("General")
    general.add_argument(
        "--max-speed",
        type=float,
        default=MAX_SPEED,
        help=f"Maximum speed (default: {MAX_SPEED})",
    )
    general.add_argument(
        "--serial-port",
        type=str,
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port for VESC motor (default: {DEFAULT_SERIAL_PORT})",
    )

    enable = parser.add_argument_group("Modules (enable)")
    enable.add_argument(
        "--camera",
        action="store_true",
        help="Enable camera capture",
    )
    enable.add_argument(
        "--enable-socket",
        action="store_true",
        help="Enable socket.io server for data streaming",
    )

    disable = parser.add_argument_group("Modules (disable)")
    disable.add_argument(
        "--no-motor",
        action="store_true",
        help="Disable motor/VESC (useful for testing without car connected)",
    )
    disable.add_argument(
        "--no-joystick",
        action="store_true",
        help="Disable joystick controller (for testing without gamepad)",
    )
    disable.add_argument(
        "--no-throttle",
        action="store_true",
        help="Disable throttle controller (for testing)",
    )
    disable.add_argument(
        "--no-camera",
        action="store_true",
        help="Explicitly disable camera (overrides --camera)",
    )
    disable.add_argument(
        "--no-lidar",
        action="store_true",
        help="Disable lidar (even if --lidar-port is specified)",
    )
    disable.add_argument(
        "--no-pan-tilt",
        action="store_true",
        help="Disable pan/tilt control",
    )
    disable.add_argument(
        "--no-rtk",
        action="store_true",
        help="Disable RTK GNSS (pose/IMU)",
    )

    ports = parser.add_argument_group("Ports")
    ports.add_argument(
        "--lidar-port",
        type=str,
        default=None,
        help=f"Serial port for lidar (e.g. /dev/ttyUSB0). Default: {DEFAULT_LIDAR_PORT} or auto-detect",
    )
    ports.add_argument(
        "--pan-tilt-port",
        type=str,
        default=None,
        help=f"Serial port for pan/tilt controller (default: {PAN_TILT_SERIAL_PORT})",
    )
    ports.add_argument(
        "--rtk-port",
        type=str,
        default=None,
        help=f"Serial port for RTK GNSS (default: {DEFAULT_RTK_SERIAL_PORT} or auto-detect)",
    )
    ports.add_argument(
        "--socket-port",
        type=int,
        default=SOCKETIO_PORT,
        help=f"Socket.io server port (default: {SOCKETIO_PORT})",
    )

    socket_grp = parser.add_argument_group("Socket")
    socket_grp.add_argument(
        "--socket-debug",
        action="store_true",
        help="Print full socket payloads (sensor_data, status) to console",
    )

    tools = parser.add_argument_group("Tools")
    tools.add_argument(
        "--list-devices",
        action="store_true",
        help="List serial ports and DepthAI cameras with identification (Lidar, RTK, etc.) then exit",
    )
    tools.add_argument(
        "--no-probe",
        action="store_true",
        help="With --list-devices: only show USB info, do not probe protocols (faster, no exclusive port)",
    )

    args = parser.parse_args()
    
    if args.list_devices:
        from drive.core.device_discovery import list_devices
        list_devices(probe=not args.no_probe, verbose=True)
        sys.exit(0)
    
    from drive.applications import ManualDriveApp
    
    # Auto-detect Lidar/RTK ports when not provided (probe once)
    need_lidar = args.lidar_port is None and not args.no_lidar
    need_rtk = args.rtk_port is None and not args.no_rtk
    mapping = {}
    if need_lidar or need_rtk:
        from drive.core.device_discovery import get_detected_port_mapping
        mapping = get_detected_port_mapping(probe=True)
        if mapping and (need_lidar or need_rtk):
            print("[Main] Ports auto-détectés:", {k: v for k, v in mapping.items()})
    
    # Resolve lidar_port: CLI > auto-detected > config fallback (only if enable_socket)
    lidar_port = args.lidar_port or mapping.get("lidar")
    if lidar_port is None and args.enable_socket and not args.no_lidar:
        lidar_port = DEFAULT_LIDAR_PORT
    
    # Resolve rtk_port: CLI > auto-detected > config fallback
    rtk_port = args.rtk_port or mapping.get("rtk") or DEFAULT_RTK_SERIAL_PORT
    
    # Determine use_lidar: enabled if lidar_port provided and not explicitly disabled
    use_lidar = None if args.no_lidar else (lidar_port is not None)
    
    # Camera: enable if --camera, or if --enable-socket and a DepthAI camera is present (same as list-devices)
    cameras_found = []
    if not args.no_camera and (args.camera or args.enable_socket):
        from drive.core.device_discovery import get_depthai_devices
        cameras_found = get_depthai_devices()
    use_camera = (args.camera or (args.enable_socket and len(cameras_found) > 0)) and not args.no_camera
    print(f"[Main] Caméra: use_camera={use_camera}, DepthAI trouvées={len(cameras_found)} (--camera pour forcer, --no-camera pour désactiver)")
    if use_camera and cameras_found and not args.camera:
        print("[Main] Caméra DepthAI détectée, activation pour le stream socket.")
    
    # Create manual drive application
    try:
        print("[Main] Starting manual drive mode...")
        
        app = ManualDriveApp(
            max_speed=args.max_speed,
            serial_port=args.serial_port,
            use_camera=use_camera,
            enable_socket=args.enable_socket,
            lidar_port=lidar_port if not args.no_lidar else None,
            socket_port=args.socket_port,
            socket_debug=args.socket_debug,
            use_motor=not args.no_motor,
            pan_tilt_port=args.pan_tilt_port,
            use_pan_tilt=not args.no_pan_tilt,
            use_joystick=not args.no_joystick,
            use_throttle=not args.no_throttle,
            use_lidar=use_lidar,
            rtk_port=rtk_port,
            use_rtk=not args.no_rtk,
        )
        
        # Run the application
        app.run()
        
    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"[Main] Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
