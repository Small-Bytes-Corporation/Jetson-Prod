#!/usr/bin/env python3
"""
Simple script to read lidar data and send it via socket.io.

Usage:
    python3 lidar_socket.py [--lidar-port PORT] [--socket-port PORT] [--no-lidar]
    
    On Mac, use --lidar-port to specify the correct port (e.g., /dev/tty.usbserial-*)
    or use --no-lidar to test socket.io without lidar hardware.
"""

import argparse
import time
import signal
import sys
from drive.core.lidar_controller import LidarController
from drive.core.socket_server import SocketServer
from drive.core.config import DEFAULT_LIDAR_PORT, LIDAR_BAUDRATE, SOCKETIO_PORT, PUBLISH_RATE, IS_MAC, IS_LINUX


class LidarSocketApp:
    """
    Simple application that reads lidar data and sends it via socket.io.
    """
    
    def __init__(self, lidar_port=DEFAULT_LIDAR_PORT, socket_port=SOCKETIO_PORT, enable_lidar=True):
        """
        Initialize the lidar socket application.
        
        Args:
            lidar_port: Serial port for lidar (e.g., '/dev/ttyUSB0' on Linux, '/dev/tty.usbserial-*' on Mac).
            socket_port: Port for socket.io server.
            enable_lidar: If False, lidar is disabled (mock mode) for testing without lidar hardware.
        """
        self.lidar_port = lidar_port
        self.socket_port = socket_port
        self.enable_lidar = enable_lidar
        
        self.lidar = LidarController(serial_port=lidar_port, baudrate=LIDAR_BAUDRATE, enabled=enable_lidar)
        self.socket_server = SocketServer(port=socket_port)
        
        self.running = True
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, sig, frame):
        """Handle shutdown signals."""
        print("\n[Signal] Exiting...")
        self.running = False
    
    def run(self):
        """Run the main loop."""
        try:
            # Initialize lidar
            if self.enable_lidar:
                print("[LidarSocket] Initializing lidar...")
                lidar_ok = self.lidar.initialize()
                if lidar_ok:
                    print(f"[LidarSocket] Lidar initialized on {self.lidar_port}")
                else:
                    print("[LidarSocket] Lidar not available, continuing in mock mode...")
            else:
                print("[LidarSocket] Lidar disabled (mock mode)")
                self.lidar.initialize()
            
            # Start socket server
            print(f"[LidarSocket] Starting socket.io server on port {self.socket_port}...")
            self.socket_server.start()
            print(f"[LidarSocket] Socket.io server running")
            
            print("[LidarSocket] Ready! Press Ctrl+C to exit.")
            
            # Main loop
            sleep_time = 1.0 / PUBLISH_RATE if PUBLISH_RATE > 0 else 0.1
            
            while self.running:
                # Get lidar scan
                lidar_scan = self.lidar.get_scan()
                
                if lidar_scan is not None:
                    # Prepare data packet
                    data = {
                        "timestamp": time.time(),
                        "lidar": {
                            "points": lidar_scan,
                            "scan_complete": True
                        }
                    }
                    
                    # Send via socket.io
                    self.socket_server.emit_sensor_data(data)
                    
                    # Print status
                    point_count = len(lidar_scan) if lidar_scan else 0
                    print(f"[LidarSocket] Sent {point_count} lidar points")
                else:
                    print("[LidarSocket] No lidar data available")
                
                # Sleep to control rate
                time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n[LidarSocket] Keyboard interrupt received.")
        except Exception as e:
            print(f"[LidarSocket] Error: {e}")
            import traceback
            traceback.print_exc()
            raise
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        print("[LidarSocket] Cleaning up...")
        self.socket_server.stop()
        self.lidar.stop()
        print("[LidarSocket] Shutdown complete.")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Lidar to socket.io streamer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    platform_name = "Mac" if IS_MAC else ("Jetson/Linux" if IS_LINUX else "Unknown")
    parser.add_argument(
        '--lidar-port',
        type=str,
        default=DEFAULT_LIDAR_PORT,
        help=f'Serial port for lidar (default: {DEFAULT_LIDAR_PORT} - auto-detected on {platform_name})'
    )
    
    parser.add_argument(
        '--socket-port',
        type=int,
        default=SOCKETIO_PORT,
        help=f'Socket.io server port (default: {SOCKETIO_PORT})'
    )
    
    parser.add_argument(
        '--no-lidar',
        action='store_true',
        help='Disable lidar (mock mode) - useful for testing socket.io without lidar hardware'
    )
    
    args = parser.parse_args()
    
    # Print platform info
    platform_name = "Mac" if IS_MAC else ("Jetson/Linux" if IS_LINUX else "Unknown")
    print(f"[LidarSocket] Platform detected: {platform_name}")
    print(f"[LidarSocket] Using lidar port: {args.lidar_port}")
    
    try:
        app = LidarSocketApp(
            lidar_port=args.lidar_port,
            socket_port=args.socket_port,
            enable_lidar=not args.no_lidar
        )
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
