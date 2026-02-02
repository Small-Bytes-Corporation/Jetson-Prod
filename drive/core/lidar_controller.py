"""
Lidar controller for LDROBOT D500 LiDarKit.
"""

import time
import os
import serial
import platform
from .config import DEFAULT_LIDAR_PORT, LIDAR_BAUDRATE, _get_available_serial_ports, IS_MAC


class LidarController:
    """
    Controller for LDROBOT D500 LiDarKit - captures lidar scan data.
    
    Note: This is a generic implementation. The exact SDK/API for D500
    may need to be adapted based on LDROBOT documentation.
    """
    
    def __init__(self, serial_port=DEFAULT_LIDAR_PORT, baudrate=LIDAR_BAUDRATE, enabled=True):
        """
        Initialize the lidar controller.
        
        Args:
            serial_port: Serial port path (e.g., '/dev/ttyUSB0' on Linux, '/dev/tty.usbserial-*' on Mac).
            baudrate: Serial communication baudrate.
            enabled: If False, lidar is disabled (mock mode) for testing without lidar hardware.
        """
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.enabled = enabled
        self.serial_conn = None
        self._initialized = False
        self.last_scan = None
        self._data_buffer = bytearray()  # Buffer for incomplete packets
        self._debug_mode = False  # Enable to print raw hex data
        self._raw_file = None  # Raw file handle for fallback mode
    
    def initialize(self):
        """
        Initialize the lidar connection.
        
        Returns:
            bool: True if initialization successful, False otherwise.
            
        Raises:
            RuntimeError: If lidar initialization fails and enabled=True.
        """
        if not self.enabled:
            print("[Lidar] Lidar disabled (mock mode)")
            self._initialized = True
            return True
        
        # Check if port exists before attempting initialization
        port_to_try = self.serial_port
        if not os.path.exists(port_to_try):
            available_ports = _get_available_serial_ports()
            error_msg = f"Lidar port '{port_to_try}' does not exist."
            if available_ports:
                error_msg += f"\nAvailable serial ports: {', '.join(available_ports)}"
                error_msg += f"\nTry: --lidar-port {available_ports[0]}"
            else:
                error_msg += "\nNo serial ports found. Make sure the lidar is connected."
            if self.enabled:
                raise RuntimeError(error_msg)
            print(f"[Lidar] {error_msg}")
            return False
        
        # Try multiple configurations to fix termios errors
        tried_ports = []
        tried_configs = []
        first_error = None  # Store first error for diagnostics
        termios_error_count = 0  # Track consecutive termios errors
        
        # Different baudrates to try (common LiDAR baudrates)
        baudrates_to_try = [self.baudrate, 115200, 230400, 460800, 921600, 256000, 512000]
        
        # Different configuration combinations to try
        configs_to_try = [
            # Ultra-minimal configs (port + baudrate only, let pyserial use defaults)
            {},  # Ultra-minimal: just baudrate
            {'timeout': None},  # No timeout
            {'timeout': 1.0},  # Minimal with timeout
            {'timeout': 1.0, 'write_timeout': 0, 'inter_byte_timeout': None},
            {'timeout': 1.0, 'rtscts': False, 'dsrdtr': False},
            # Standard configs
            {'timeout': 1.0, 'bytesize': serial.EIGHTBITS, 'parity': serial.PARITY_NONE, 'stopbits': serial.STOPBITS_ONE},
            {'timeout': 1.0, 'exclusive': False},
            {'timeout': 2.0},
            {'timeout': 1.0, 'bytesize': serial.EIGHTBITS, 'parity': serial.PARITY_NONE, 'stopbits': serial.STOPBITS_TWO},
        ]
        
        for baudrate in baudrates_to_try:
            for config in configs_to_try:
                config_key = f"{baudrate}_{str(config)}"
                if config_key in tried_configs:
                    continue
                
                # Try to open serial connection with this configuration
                try:
                    # Clean up any previous failed connection attempt
                    if self.serial_conn is not None:
                        try:
                            self.serial_conn.close()
                        except:
                            pass
                        self.serial_conn = None
                    
                    # Build serial parameters
                    serial_params = {
                        'port': port_to_try,
                        'baudrate': baudrate,
                        **config
                    }
                    
                    # Ensure default values if not specified
                    if 'bytesize' not in serial_params:
                        serial_params['bytesize'] = serial.EIGHTBITS
                    if 'parity' not in serial_params:
                        serial_params['parity'] = serial.PARITY_NONE
                    if 'stopbits' not in serial_params:
                        serial_params['stopbits'] = serial.STOPBITS_ONE
                    if 'timeout' not in serial_params:
                        serial_params['timeout'] = 1.0
                    
                    self.serial_conn = serial.Serial(**serial_params)
                    
                    # Wait for connection to stabilize
                    time.sleep(0.5)
                    
                    # Test if we can actually read/write
                    if self.serial_conn.is_open:
                        # Update serial_port and baudrate if we used different values
                        if port_to_try != self.serial_port:
                            print(f"[Lidar] Switched from {self.serial_port} to {port_to_try}")
                            self.serial_port = port_to_try
                        if baudrate != self.baudrate:
                            print(f"[Lidar] Using baudrate {baudrate} instead of {self.baudrate}")
                            self.baudrate = baudrate
                        
                        self._initialized = True
                        print(f"[Lidar] Successfully initialized on {self.serial_port} @ {baudrate} baud")
                        return True
                    
                except FileNotFoundError:
                    # Port doesn't exist, try next port
                    available_ports = _get_available_serial_ports()
                    if available_ports:
                        for avail_port in available_ports:
                            if avail_port not in tried_ports:
                                print(f"[Lidar] Port '{port_to_try}' not found, trying '{avail_port}'...")
                                port_to_try = avail_port
                                tried_ports.append(port_to_try)
                                break
                        else:
                            # No more ports to try
                            error_msg = f"Lidar port '{port_to_try}' not found."
                            error_msg += f"\nAvailable serial ports: {', '.join(available_ports)}"
                            if self.enabled:
                                raise RuntimeError(error_msg)
                            return False
                    else:
                        error_msg = "No serial ports found. Make sure the lidar is connected."
                        if self.enabled:
                            raise RuntimeError(error_msg)
                        return False
                    
                except (OSError, IOError) as e:
                    # Handle termios errors
                    error_code = getattr(e, 'errno', None)
                    tried_configs.append(config_key)
                    
                    # Store first error for diagnostics with full traceback
                    if first_error is None:
                        import traceback
                        first_error = {
                            'type': type(e).__name__,
                            'errno': error_code,
                            'message': str(e),
                            'traceback': traceback.format_exc(),
                            'port': port_to_try,
                            'baudrate': baudrate,
                            'config': config
                        }
                    
                    # Clean up failed connection
                    if self.serial_conn is not None:
                        try:
                            self.serial_conn.close()
                        except:
                            pass
                        self.serial_conn = None
                    
                    if error_code == 22:  # Invalid argument (termios error)
                        termios_error_count += 1
                        # Show progress every 10 attempts
                        if len(tried_configs) % 10 == 0:
                            print(f"[Lidar] ... Trying configuration {len(tried_configs)}/{(len(baudrates_to_try) * len(configs_to_try))} (termios errors: {termios_error_count})")
                        
                        # Early exit after 15 consecutive termios errors
                        if termios_error_count >= 15:
                            print(f"[Lidar] Too many termios errors ({termios_error_count}), trying alternative approaches...")
                            break  # Exit pyserial loop early
                        continue
                    else:
                        # Other error, show it
                        if len(tried_configs) <= 3:  # Show first few non-termios errors
                            print(f"[Lidar] ✗ Config failed: {type(e).__name__} (errno {error_code}): {e}")
                        continue
                        
                except Exception as e:
                    # Other exceptions, try next config
                    tried_configs.append(config_key)
                    
                    # Store first error for diagnostics with full traceback
                    if first_error is None:
                        import traceback
                        first_error = {
                            'type': type(e).__name__,
                            'errno': None,
                            'message': str(e),
                            'traceback': traceback.format_exc(),
                            'port': port_to_try,
                            'baudrate': baudrate,
                            'config': config
                        }
                    
                    # Clean up failed connection
                    if self.serial_conn is not None:
                        try:
                            self.serial_conn.close()
                        except:
                            pass
                        self.serial_conn = None
                    
                    if len(tried_configs) <= 3:  # Show first few exceptions
                        print(f"[Lidar] ✗ Exception: {type(e).__name__}: {e}")
                    continue
        
        # Try open-then-configure approach (works for some Mac adapters)
        print("[Lidar] Trying open-then-configure approach...")
        for baudrate in baudrates_to_try[:3]:  # Try first 3 baudrates only
            try:
                # Clean up any previous failed connection attempt
                if self.serial_conn is not None:
                    try:
                        self.serial_conn.close()
                    except:
                        pass
                    self.serial_conn = None
                
                # Open with minimal params first
                self.serial_conn = serial.Serial(port=port_to_try, baudrate=baudrate)
                time.sleep(0.5)
                
                if self.serial_conn.is_open:
                    # Try to configure after opening
                    try:
                        self.serial_conn.timeout = 1.0
                        self.serial_conn.bytesize = serial.EIGHTBITS
                        self.serial_conn.parity = serial.PARITY_NONE
                        self.serial_conn.stopbits = serial.STOPBITS_ONE
                    except:
                        pass  # Configuration may fail, but port is open
                    
                    if port_to_try != self.serial_port:
                        print(f"[Lidar] Switched from {self.serial_port} to {port_to_try}")
                        self.serial_port = port_to_try
                    if baudrate != self.baudrate:
                        print(f"[Lidar] Using baudrate {baudrate} instead of {self.baudrate}")
                        self.baudrate = baudrate
                    
                    self._initialized = True
                    print(f"[Lidar] Successfully initialized on {self.serial_port} @ {baudrate} baud (open-then-configure)")
                    return True
            except Exception as e:
                # Clean up failed connection
                if self.serial_conn is not None:
                    try:
                        self.serial_conn.close()
                    except:
                        pass
                    self.serial_conn = None
                continue
        
        # Try raw file I/O fallback (last resort, bypasses termios)
        if IS_MAC:
            print("[Lidar] Trying raw file I/O fallback (bypasses termios)...")
            try:
                # Clean up any previous failed connection attempt
                if self.serial_conn is not None:
                    try:
                        self.serial_conn.close()
                    except:
                        pass
                    self.serial_conn = None
                
                # Try opening as raw file with non-blocking mode
                import fcntl
                fd = None
                try:
                    # Open file descriptor in non-blocking mode
                    fd = os.open(port_to_try, os.O_RDONLY | os.O_NONBLOCK)
                    
                    # Wrap file descriptor to get file-like object
                    self._raw_file = os.fdopen(fd, 'rb', buffering=0)
                    
                    # Ensure non-blocking mode is set (should already be set by O_NONBLOCK)
                    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
                    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
                    
                    # File opened successfully
                    self._initialized = True
                    print(f"[Lidar] Successfully initialized on {port_to_try} using raw file I/O (fallback mode)")
                    print("[Lidar] WARNING: Raw file mode has limitations (no baudrate control, basic I/O only)")
                    return True
                except OSError as e:
                    # If opening failed, clean up
                    if fd is not None:
                        try:
                            os.close(fd)
                        except:
                            pass
                    raise
            except Exception as e:
                if self._raw_file is not None:
                    try:
                        self._raw_file.close()
                    except:
                        pass
                    self._raw_file = None
                print(f"[Lidar] Raw file I/O fallback failed: {e}")
                # Continue to final error message
        
        # If we get here, all configurations failed
        available_ports = _get_available_serial_ports()
        error_msg = f"Failed to initialize lidar on port '{port_to_try}' after trying multiple configurations."
        error_msg += f"\nTried {len(tried_configs)} different configurations."
        
        # Add first error details for diagnostics
        if first_error:
            error_msg += f"\n\nFirst error encountered:"
            error_msg += f"\n  Type: {first_error['type']}"
            if first_error.get('errno'):
                error_msg += f"\n  Error code: {first_error['errno']}"
            error_msg += f"\n  Message: {first_error['message']}"
            error_msg += f"\n  Port: {first_error['port']}"
            error_msg += f"\n  Baudrate: {first_error['baudrate']}"
            error_msg += f"\n  Config: {first_error['config']}"
            if first_error.get('traceback'):
                error_msg += f"\n\nFull traceback:\n{first_error['traceback']}"
        
        if available_ports:
            error_msg += f"\n\nAvailable serial ports: {', '.join(available_ports)}"
            error_msg += f"\nTry: --lidar-port {available_ports[0]}"
        
        if IS_MAC:
            error_msg += "\n\nOn Mac, termios errors can occur due to:"
            error_msg += "\n- USB adapter compatibility issues"
            error_msg += "\n- Port permissions (try: sudo chmod 666 /dev/tty.usbserial-*)"
            error_msg += "\n- Driver issues with the USB-to-serial adapter"
            error_msg += "\n- Try using /dev/tty.* ports instead of /dev/cu.* ports"
        
        if self.enabled:
            raise RuntimeError(error_msg)
        else:
            print(f"[Lidar] {error_msg}")
            return False
    
    def get_scan(self):
        """
        Get the latest complete lidar scan.
        
        Returns:
            list or None: List of points with format [{"angle": float, "distance": float, "intensity": int}, ...]
                         or None if no scan available.
        """
        if not self._initialized:
            return None
        
        # Handle raw file mode (fallback)
        if self._raw_file is not None:
            try:
                import errno
                # Try to read available data (non-blocking mode)
                try:
                    raw_data = self._raw_file.read(4096)  # Read up to 4KB
                    if raw_data:
                        # Debug: print raw hex data if enabled
                        if self._debug_mode:
                            hex_str = ' '.join(f'{b:02x}' for b in raw_data)
                            print(f"[Lidar] Raw data ({len(raw_data)} bytes): {hex_str}")
                        
                        # Add to buffer
                        self._data_buffer.extend(raw_data)
                        
                        # Parse lidar data according to D500 protocol
                        parsed_points = self._parse_lidar_data(self._data_buffer)
                        
                        if parsed_points:
                            self.last_scan = parsed_points
                            return parsed_points
                        else:
                            # Keep buffer if parsing didn't yield complete packets
                            # Limit buffer size to prevent memory issues
                            if len(self._data_buffer) > 4096:
                                print("[Lidar] Warning: Buffer too large, clearing...")
                                self._data_buffer = bytearray()
                            
                            return self.last_scan
                    else:
                        # No data available (non-blocking read returned empty)
                        return self.last_scan
                except (BlockingIOError, OSError) as e:
                    # Non-blocking read would block - this is normal, just return last scan
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        return self.last_scan
                    # Other error, re-raise
                    raise
            except Exception as e:
                print(f"[Lidar] Error reading from raw file: {e}")
                return self.last_scan
        
        # Normal serial port mode
        if self.serial_conn is None:
            return None
        
        try:
            # Read available data from serial port
            if self.serial_conn.in_waiting > 0:
                # Read raw data
                raw_data = self.serial_conn.read(self.serial_conn.in_waiting)
                
                # Debug: print raw hex data if enabled
                if self._debug_mode:
                    hex_str = ' '.join(f'{b:02x}' for b in raw_data)
                    print(f"[Lidar] Raw data ({len(raw_data)} bytes): {hex_str}")
                
                # Add to buffer
                self._data_buffer.extend(raw_data)
                
                # Parse lidar data according to D500 protocol
                parsed_points = self._parse_lidar_data(self._data_buffer)
                
                if parsed_points:
                    self.last_scan = parsed_points
                    return parsed_points
                else:
                    # Keep buffer if parsing didn't yield complete packets
                    # Limit buffer size to prevent memory issues
                    if len(self._data_buffer) > 4096:
                        print("[Lidar] Warning: Buffer too large, clearing...")
                        self._data_buffer = bytearray()
                    
                    return self.last_scan
            else:
                return self.last_scan
        except Exception as e:
            print(f"[Lidar] Error reading scan: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _parse_lidar_data(self, raw_data):
        """
        Parse raw lidar data according to D500 protocol.
        
        This implementation attempts to parse common LiDAR packet structures.
        If the exact D500 protocol is known, this should be updated accordingly.
        
        Common LiDAR packet formats:
        - Header (1-2 bytes): Often 0xAA, 0x55, or similar sync bytes
        - Angle (2 bytes): 0-36000 (0.01° resolution) or 0-3600 (0.1° resolution)
        - Distance (2-3 bytes): Distance in mm or cm
        - Intensity (1 byte): 0-255
        - Checksum (optional): 1-2 bytes
        
        Args:
            raw_data: Raw bytes from serial port (bytearray).
            
        Returns:
            list: Parsed points with angle, distance, intensity, or empty list if no complete packets found.
        """
        if len(raw_data) < 5:  # Minimum packet size
            return []
        
        points = []
        i = 0
        
        # Common packet header patterns to look for
        # Try to find packet start markers
        while i < len(raw_data) - 4:
            # Look for common header patterns
            # Pattern 1: 0xAA 0x55 (common sync pattern)
            # Pattern 2: 0x55 0xAA
            # Pattern 3: Angle byte pattern (if angle is first)
            
            # Try to parse as potential packet
            # Common structure: [header1, header2, angle_low, angle_high, dist_low, dist_high, intensity]
            # Or: [angle_low, angle_high, dist_low, dist_high, intensity]
            
            # Attempt parsing with different packet structures
            parsed = False
            
            # Structure 1: 7-byte packet [header1, header2, angle_low, angle_high, dist_low, dist_high, intensity]
            if i + 7 <= len(raw_data):
                if raw_data[i] == 0xAA and raw_data[i+1] == 0x55:
                    angle_raw = (raw_data[i+3] << 8) | raw_data[i+2]
                    distance_raw = (raw_data[i+5] << 8) | raw_data[i+4]
                    intensity = raw_data[i+6]
                    
                    angle = angle_raw / 100.0  # Assuming 0.01° resolution
                    distance = distance_raw / 1000.0  # Assuming mm to meters
                    
                    points.append({
                        "angle": angle,
                        "distance": distance,
                        "intensity": intensity
                    })
                    i += 7
                    parsed = True
            
            # Structure 2: 5-byte packet [angle_low, angle_high, dist_low, dist_high, intensity]
            if not parsed and i + 5 <= len(raw_data):
                angle_raw = (raw_data[i+1] << 8) | raw_data[i]
                distance_raw = (raw_data[i+3] << 8) | raw_data[i+2]
                intensity = raw_data[i+4]
                
                # Validate reasonable ranges
                if 0 <= angle_raw <= 36000 and 0 <= distance_raw <= 20000 and 0 <= intensity <= 255:
                    angle = angle_raw / 100.0  # Assuming 0.01° resolution
                    distance = distance_raw / 1000.0  # Assuming mm to meters
                    
                    points.append({
                        "angle": angle,
                        "distance": distance,
                        "intensity": intensity
                    })
                    i += 5
                    parsed = True
            
            # Structure 3: 6-byte packet with 3-byte distance [angle_low, angle_high, dist_low, dist_mid, dist_high, intensity]
            if not parsed and i + 6 <= len(raw_data):
                angle_raw = (raw_data[i+1] << 8) | raw_data[i]
                distance_raw = (raw_data[i+2] | (raw_data[i+3] << 8) | (raw_data[i+4] << 16))
                intensity = raw_data[i+5]
                
                if 0 <= angle_raw <= 36000 and 0 <= distance_raw <= 200000 and 0 <= intensity <= 255:
                    angle = angle_raw / 100.0
                    distance = distance_raw / 1000.0
                    
                    points.append({
                        "angle": angle,
                        "distance": distance,
                        "intensity": intensity
                    })
                    i += 6
                    parsed = True
            
            if not parsed:
                i += 1
        
        # If we parsed any points, remove processed data from buffer
        # Track bytes consumed more accurately
        if points:
            # Calculate bytes consumed based on which structure was used
            # This is approximate - in production, track exact bytes per packet
            bytes_consumed = 0
            for _ in points:
                # Estimate: most packets are 5-7 bytes
                # We'll use a conservative estimate and let the parser resync
                bytes_consumed += 5  # Conservative estimate
            
            if bytes_consumed < len(self._data_buffer):
                self._data_buffer = self._data_buffer[bytes_consumed:]
            else:
                # If we consumed more than buffer, clear it
                self._data_buffer = bytearray()
        
        return points
    
    def is_available(self):
        """
        Check if lidar is available and initialized.
        
        Returns:
            bool: True if lidar is ready, False otherwise.
        """
        if not self._initialized:
            return False
        
        # Handle raw file mode
        if self._raw_file is not None:
            try:
                return not self._raw_file.closed
            except Exception:
                return False
        
        # Normal serial port mode
        if self.serial_conn is None:
            return False
        
        try:
            return self.serial_conn.is_open
        except Exception:
            return False
    
    def close(self):
        """
        Close the lidar connection and cleanup resources.
        """
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        
        if self._raw_file is not None:
            try:
                self._raw_file.close()
            except Exception:
                pass
            self._raw_file = None
        
        self._initialized = False
    
    def enable_debug_mode(self, enable=True):
        """
        Enable or disable debug mode (prints raw hex data).
        
        Args:
            enable: True to enable debug mode, False to disable.
        """
        self._debug_mode = enable
        print(f"[Lidar] Debug mode {'enabled' if enable else 'disabled'}")
    
    def test_connection(self):
        """
        Test the lidar connection by attempting to read a few bytes.
        
        Returns:
            bool: True if connection test successful, False otherwise.
        """
        if not self._initialized or self.serial_conn is None:
            print("[Lidar] Not initialized, cannot test connection")
            return False
        
        try:
            if not self.serial_conn.is_open:
                print("[Lidar] Serial port is not open")
                return False
            
            # Try to read available data
            if self.serial_conn.in_waiting > 0:
                test_data = self.serial_conn.read(min(32, self.serial_conn.in_waiting))
                hex_str = ' '.join(f'{b:02x}' for b in test_data)
                print(f"[Lidar] Connection test successful! Read {len(test_data)} bytes:")
                print(f"[Lidar] Hex: {hex_str}")
                return True
            else:
                print("[Lidar] Connection test: Port is open but no data available")
                print("[Lidar] This is normal if the lidar hasn't sent data yet")
                return True
        except Exception as e:
            print(f"[Lidar] Connection test failed: {e}")
            return False
    
    def print_raw_data(self, num_bytes=64):
        """
        Print raw hex data from the serial port.
        
        Args:
            num_bytes: Number of bytes to read and print (default: 64).
        """
        if not self._initialized or self.serial_conn is None:
            print("[Lidar] Not initialized, cannot read data")
            return
        
        try:
            if not self.serial_conn.is_open:
                print("[Lidar] Serial port is not open")
                return
            
            available = self.serial_conn.in_waiting
            if available == 0:
                print("[Lidar] No data available")
                return
            
            bytes_to_read = min(num_bytes, available)
            raw_data = self.serial_conn.read(bytes_to_read)
            
            # Print hex dump
            print(f"[Lidar] Raw data ({len(raw_data)} bytes):")
            hex_str = ' '.join(f'{b:02x}' for b in raw_data)
            print(f"  Hex: {hex_str}")
            
            # Print ASCII representation (if printable)
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw_data)
            print(f"  ASCII: {ascii_str}")
            
        except Exception as e:
            print(f"[Lidar] Error reading raw data: {e}")
    
    def list_available_configs(self):
        """
        Try different serial port configurations and report which ones work.
        Useful for diagnosing connection issues.
        """
        if not self.enabled:
            print("[Lidar] Lidar is disabled, cannot test configurations")
            return
        
        print("[Lidar] Testing available serial port configurations...")
        print(f"[Lidar] Port: {self.serial_port}")
        
        working_configs = []
        baudrates_to_try = [115200, 230400, 460800, 921600, 256000, 512000]
        
        for baudrate in baudrates_to_try:
            try:
                test_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=baudrate,
                    timeout=0.5
                )
                test_conn.close()
                working_configs.append(f"Baudrate {baudrate}: OK")
                print(f"  ✓ Baudrate {baudrate}: OK")
            except Exception as e:
                error_code = getattr(e, 'errno', None)
                if error_code == 22:
                    print(f"  ✗ Baudrate {baudrate}: Termios error")
                else:
                    print(f"  ✗ Baudrate {baudrate}: {e}")
        
        if working_configs:
            print(f"\n[Lidar] Found {len(working_configs)} working configurations")
        else:
            print("\n[Lidar] No working configurations found")
            print("[Lidar] Check:")
            print("  - Port permissions: sudo chmod 666 /dev/tty.usbserial-*")
            print("  - USB adapter compatibility")
            print("  - Cable connections")
    
    def stop(self):
        """Stop the lidar and clean up resources."""
        self.close()
        self.last_scan = None
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
