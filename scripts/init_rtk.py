#!/usr/bin/env python3
"""
Script d'initialisation RTK Polaris pour Point One Navigation.
Utilise p1-host-tools pour configurer le récepteur RTK avec la clé Polaris.

Usage:
    python scripts/init_rtk.py [--port /dev/ttyUSB0] [--device-id robocar-rtk] [--duration 30] [--check-only]
"""

import sys

# Check Python version FIRST (before any other imports)
# p1-host-tools requires Python 3.10+ for TypeAlias support
if sys.version_info < (3, 10):
    print("=" * 70)
    print("ERREUR: Version Python incompatible")
    print("=" * 70)
    print(f"Python {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro} détecté")
    print("p1-host-tools/fusion_engine_client nécessite Python 3.10+")
    print()
    print("Solutions possibles:")
    print("  1. Utiliser Python 3.10+ explicitement:")
    print("     python3.10 scripts/init_rtk.py")
    print("     python3.11 scripts/init_rtk.py")
    print()
    print("  2. Mettre à jour votre environnement Python")
    print("  3. Utiliser un environnement virtuel avec Python 3.10+")
    print()
    print("Vérification de la version Python disponible:")
    try:
        import subprocess
        for py_cmd in ['python3.10', 'python3.11', 'python3.12']:
            try:
                result = subprocess.run(
                    [py_cmd, '--version'],
                    capture_output=True,
                    timeout=1
                )
                if result.returncode == 0:
                    print(f"  ✓ {py_cmd}: {result.stdout.decode().strip()}")
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass
    except Exception:
        pass
    print("=" * 70)
    sys.exit(1)

import argparse
import os
import subprocess
import time
import signal

# Ensure project root is on path when run as scripts/init_rtk.py
if __name__ == "__main__":
    _root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _root not in sys.path:
        sys.path.insert(0, _root)

from drive.core.config import POLARIS_API_KEY, DEFAULT_RTK_SERIAL_PORT


def find_p1_host_tools():
    """
    Find p1-host-tools directory. Checks project root first, then HOME.
    Returns path if found, None otherwise.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Check project root first
    p1_path = os.path.join(project_root, 'p1-host-tools')
    if os.path.isdir(p1_path):
        return p1_path
    
    # Check HOME
    home_path = os.path.join(os.path.expanduser('~'), 'p1-host-tools')
    if os.path.isdir(home_path):
        return home_path
    
    return None


def check_p1_host_tools(p1_dir):
    """
    Check if p1-host-tools is properly set up.
    Returns (is_valid, error_message).
    """
    if not p1_dir or not os.path.isdir(p1_dir):
        return False, "p1-host-tools directory not found"
    
    runner_path = os.path.join(p1_dir, 'bin', 'runner.py')
    if not os.path.isfile(runner_path):
        return False, f"runner.py not found in {p1_dir}/bin/"
    
    return True, None


def check_rtk_connection(port):
    """
    Simple check to verify RTK device is connected on the specified port.
    Returns True if port exists and is accessible.
    """
    if not os.path.exists(port):
        return False, f"Port {port} does not exist"
    
    if not os.access(port, os.R_OK | os.W_OK):
        return False, f"Port {port} is not accessible (permission denied)"
    
    return True, None


def run_p1_runner(p1_dir, port, device_id, polaris_key, duration=None, tcp_port=None):
    """
    Run p1-host-tools/bin/runner.py with Polaris configuration.
    Returns subprocess.Popen object.
    If tcp_port is set, forwards Fusion Engine to TCP so main.py can connect via --rtk-tcp.
    """
    runner_path = os.path.join(p1_dir, 'bin', 'runner.py')
    
    # Use python3 explicitly, or check if we need a specific version
    python_cmd = sys.executable
    
    # Verify Python version for subprocess
    try:
        version_check = subprocess.run(
            [python_cmd, '--version'],
            capture_output=True,
            text=True,
            timeout=2
        )
        if version_check.returncode == 0:
            version_str = version_check.stdout.strip()
            print(f"[InitRTK] Using Python: {version_str}")
    except Exception:
        pass
    
    cmd = [
        python_cmd,
        runner_path,
        '--device-id', device_id,
        '--polaris', polaris_key,
        '--port', port,
    ]
    if tcp_port is not None:
        cmd.extend(['--tcp', str(tcp_port)])
    
    print(f"[InitRTK] Running: {' '.join(cmd)}")
    print(f"[InitRTK] Working directory: {p1_dir}")
    print()
    
    # Change to p1-host-tools directory for runner.py
    process = subprocess.Popen(
        cmd,
        cwd=p1_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        bufsize=1
    )
    
    return process


def main():
    parser = argparse.ArgumentParser(
        description="Initialize RTK Polaris connection using p1-host-tools",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect RTK port and initialize
  python scripts/init_rtk.py

  # Specify port manually
  python scripts/init_rtk.py --port /dev/ttyUSB1

  # Check connection only (no configuration)
  python scripts/init_rtk.py --check-only

  # Custom device ID and duration
  python scripts/init_rtk.py --device-id my-device --duration 60

  # Forward Fusion Engine to TCP for main.py (valid RTK fix)
  python scripts/init_rtk.py --tcp 30201
  # Then in another terminal: python main.py --enable-socket --rtk-tcp localhost:30201
        """
    )
    
    parser.add_argument(
        '--port',
        type=str,
        default=None,
        help=f'Serial port for RTK device (default: {DEFAULT_RTK_SERIAL_PORT} from ports_config.py)'
    )
    parser.add_argument(
        '--device-id',
        type=str,
        default='robocar-rtk',
        help='Device ID for Polaris (default: robocar-rtk)'
    )
    parser.add_argument(
        '--duration',
        type=int,
        default=30,
        help='Duration in seconds to run initialization (default: 30)'
    )
    parser.add_argument(
        '--check-only',
        action='store_true',
        help='Only check connection, do not configure Polaris'
    )
    parser.add_argument(
        '--tcp',
        type=int,
        default=None,
        metavar='PORT',
        help='Forward Fusion Engine to TCP port for main.py (use main.py --rtk-tcp localhost:PORT)'
    )
    
    args = parser.parse_args()
    
    # Find p1-host-tools
    p1_dir = find_p1_host_tools()
    is_valid, error_msg = check_p1_host_tools(p1_dir)
    
    if not is_valid:
        print(f"[InitRTK] ERROR: {error_msg}")
        print(f"[InitRTK] Please install p1-host-tools:")
        print(f"  git clone https://github.com/PointOneNav/p1-host-tools.git")
        print(f"  # Or clone it in the project root or $HOME")
        sys.exit(1)
    
    print(f"[InitRTK] Using p1-host-tools: {p1_dir}")
    
    # Get RTK port: CLI > config fallback (from ports_config.py)
    rtk_port = args.port or DEFAULT_RTK_SERIAL_PORT
    
    # Check port
    port_ok, port_error = check_rtk_connection(rtk_port)
    if not port_ok:
        print(f"[InitRTK] ERROR: {port_error}")
        sys.exit(1)
    
    print(f"[InitRTK] RTK port: {rtk_port}")
    
    # Get Polaris API key
    if not POLARIS_API_KEY or POLARIS_API_KEY == 'your_polaris_api_key_here':
        print("[InitRTK] ERROR: POLARIS_API_KEY not configured")
        print("[InitRTK] Please set POLARIS_API_KEY in .env file or config.py")
        sys.exit(1)
    
    print(f"[InitRTK] Device ID: {args.device_id}")
    print(f"[InitRTK] Polaris Key: {POLARIS_API_KEY[:8]}...{POLARIS_API_KEY[-8:]}")
    print()
    
    if args.check_only:
        print("[InitRTK] Check-only mode: verifying RTK connection...")
        print("[InitRTK] Port exists and is accessible: OK")
        print("[InitRTK] To configure Polaris, run without --check-only")
        sys.exit(0)
    
    # Check Python version before running
    python_version = sys.version_info
    if python_version < (3, 10):
        print("=" * 70)
        print("ERREUR: Version Python incompatible")
        print("=" * 70)
        print(f"Python {python_version.major}.{python_version.minor}.{python_version.micro} détecté")
        print("p1-host-tools/fusion_engine_client nécessite Python 3.10+")
        print()
        print("Solutions possibles:")
        print("  1. Utiliser Python 3.10+ explicitement:")
        print("     python3.10 scripts/init_rtk.py")
        print("     python3.11 scripts/init_rtk.py")
        print()
        print("  2. Mettre à jour votre environnement Python")
        print("  3. Utiliser un environnement virtuel avec Python 3.10+")
        print()
        print("Vérification de la version Python disponible:")
        try:
            import subprocess
            for py_cmd in ['python3.10', 'python3.11', 'python3.12']:
                try:
                    result = subprocess.run(
                        [py_cmd, '--version'],
                        capture_output=True,
                        timeout=1
                    )
                    if result.returncode == 0:
                        print(f"  ✓ {py_cmd}: {result.stdout.decode().strip()}")
                except (FileNotFoundError, subprocess.TimeoutExpired):
                    pass
        except Exception:
            pass
        print("=" * 70)
        sys.exit(1)
    
    # Run initialization
    print("=" * 70)
    print("INITIALISATION RTK POLARIS")
    print("=" * 70)
    print()
    print(f"Configuration:")
    print(f"  Port: {rtk_port}")
    print(f"  Device ID: {args.device_id}")
    print(f"  Duration: {args.duration} seconds")
    print(f"  Python: {python_version.major}.{python_version.minor}.{python_version.micro}")
    if args.tcp is not None:
        print(f"  TCP output: port {args.tcp} (main.py --rtk-tcp localhost:{args.tcp})")
    print()
    print("Instructions:")
    print("  - Watch for 'corrections=X B' messages")
    print("  - X should be > 0 when Polaris connection is established")
    print("  - Press Ctrl+C to stop early if connection is confirmed")
    print()
    print("=" * 70)
    print()
    
    process = None
    try:
        process = run_p1_runner(p1_dir, rtk_port, args.device_id, POLARIS_API_KEY, args.duration, tcp_port=args.tcp)
        
        # Monitor output with timeout
        start_time = time.time()
        deadline = start_time + args.duration
        
        while True:
            if process.poll() is not None:
                # Process ended
                break
            
            if time.time() >= deadline:
                # Timeout reached
                print(f"\n[InitRTK] Duration ({args.duration}s) reached. Stopping...")
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                break
            
            # Read output line by line
            if process.stdout:
                line = process.stdout.readline()
                if line:
                    print(line.rstrip())
            
            time.sleep(0.1)
        
        # Read remaining output and check for errors
        output_lines = []
        if process.stdout:
            remaining = process.stdout.read()
            if remaining:
                output_lines = remaining.split('\n')
                print(remaining.rstrip())
        
        return_code = process.wait()
        
        print()
        print("=" * 70)
        print("RÉSUMÉ")
        print("=" * 70)
        
        if return_code == 0:
            print("[InitRTK] Initialization completed successfully")
        else:
            print(f"[InitRTK] Process exited with code {return_code}")
            print("[InitRTK] Check output above for errors")
            
            # Check for common errors and provide helpful messages
            output_text = '\n'.join(output_lines) if output_lines else ''
            if "TypeAlias" in output_text or "cannot import name 'TypeAlias'" in output_text:
                print()
                print("[InitRTK] ⚠ ERREUR: Incompatibilité de version Python détectée")
                print("[InitRTK] fusion_engine_client nécessite Python 3.10+")
                print(f"[InitRTK] Version Python utilisée: {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")
                print()
                print("[InitRTK] Solutions:")
                print("  1. Utiliser Python 3.10+ explicitement:")
                print("     python3.10 scripts/init_rtk.py")
                print("     python3.11 scripts/init_rtk.py")
                print()
                print("  2. Vérifier les versions Python disponibles:")
                print("     which python3.10")
                print("     which python3.11")
                print()
                print("  3. Créer un nouvel environnement avec Python 3.10+")
                print("     conda create -n robocar python=3.10")
                print("     conda activate robocar")
                print("     pip install -r requirements.txt")
        
    except KeyboardInterrupt:
        print("\n[InitRTK] Interrupted by user")
        if process:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        sys.exit(0)
    except Exception as e:
        print(f"\n[InitRTK] ERROR: {e}")
        import traceback
        traceback.print_exc()
        if process:
            process.terminate()
        sys.exit(1)


if __name__ == "__main__":
    main()
