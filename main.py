#!/usr/bin/env python3
"""
Main entry point for the robocar control system.

Usage:
    python3 main.py [--max-speed FLOAT] [--serial-port PATH] [--camera] 
                    [--enable-socket] [--lidar-port PATH] [--socket-port INT]
                    [--no-motor] [--pan-tilt-port PATH] [--no-pan-tilt]
                    [--no-joystick] [--no-throttle] [--no-lidar] [--no-camera]
"""

import argparse
import sys
from drive.applications import ManualDriveApp
from drive.core.config import MAX_SPEED, DEFAULT_SERIAL_PORT, DEFAULT_LIDAR_PORT, SOCKETIO_PORT, PAN_TILT_SERIAL_PORT


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Robocar control system",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument(
        '--max-speed',
        type=float,
        default=MAX_SPEED,
        help=f'Maximum speed (default: {MAX_SPEED})'
    )
    
    parser.add_argument(
        '--serial-port',
        type=str,
        default=DEFAULT_SERIAL_PORT,
        help=f'Serial port for VESC motor (default: {DEFAULT_SERIAL_PORT})'
    )
    
    parser.add_argument(
        '--camera',
        action='store_true',
        help='Enable camera capture'
    )
    
    parser.add_argument(
        '--enable-socket',
        action='store_true',
        help='Enable socket.io server for data streaming'
    )
    
    parser.add_argument(
        '--lidar-port',
        type=str,
        default=None,
        help=f'Serial port for lidar (e.g., /dev/ttyUSB0). If not specified, lidar is disabled. Default: {DEFAULT_LIDAR_PORT}'
    )
    
    parser.add_argument(
        '--socket-port',
        type=int,
        default=SOCKETIO_PORT,
        help=f'Socket.io server port (default: {SOCKETIO_PORT})'
    )
    
    parser.add_argument(
        '--no-motor',
        action='store_true',
        help='Disable motor/VESC (useful for testing lidar/socket without car connected)'
    )
    
    parser.add_argument(
        '--pan-tilt-port',
        type=str,
        default=None,
        help=f'Serial port for pan/tilt controller (default: {PAN_TILT_SERIAL_PORT})'
    )
    
    parser.add_argument(
        '--no-pan-tilt',
        action='store_true',
        help='Disable pan/tilt control'
    )
    
    parser.add_argument(
        '--no-joystick',
        action='store_true',
        help='Disable joystick controller (for testing without gamepad)'
    )
    
    parser.add_argument(
        '--no-throttle',
        action='store_true',
        help='Disable throttle controller (for testing)'
    )
    
    parser.add_argument(
        '--no-lidar',
        action='store_true',
        help='Disable lidar (even if --lidar-port is specified)'
    )
    
    parser.add_argument(
        '--no-camera',
        action='store_true',
        help='Explicitly disable camera (overrides --camera flag)'
    )
    
    args = parser.parse_args()
    
    # Create manual drive application
    try:
        print("[Main] Starting manual drive mode...")
        
        # Use default lidar port if enable-socket is set but lidar-port not specified
        lidar_port = args.lidar_port if args.lidar_port else (DEFAULT_LIDAR_PORT if args.enable_socket else None)
        
        # Determine use_lidar: enabled if lidar_port provided and not explicitly disabled
        use_lidar = None if args.no_lidar else (lidar_port is not None)
        
        app = ManualDriveApp(
            max_speed=args.max_speed,
            serial_port=args.serial_port,
            use_camera=args.camera and not args.no_camera,
            enable_socket=args.enable_socket,
            lidar_port=lidar_port if not args.no_lidar else None,
            socket_port=args.socket_port,
            use_motor=not args.no_motor,
            pan_tilt_port=args.pan_tilt_port,
            use_pan_tilt=not args.no_pan_tilt,
            use_joystick=not args.no_joystick,
            use_throttle=not args.no_throttle,
            use_lidar=use_lidar
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
