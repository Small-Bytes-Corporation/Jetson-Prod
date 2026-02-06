"""
Rich-based parameter tuner for Lidar Navigation.
Threaded version to prevent input lag caused by the main motor loop.
"""

import sys
import time
import select
import re
import threading

from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.layout import Layout
from rich.text import Text
from rich import box

# Optional: termios for raw keyboard input (Unix only)
try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False

# Key constants
KEY_UP = "up"
KEY_DOWN = "down"
KEY_LEFT = "left"
KEY_RIGHT = "right"

# Slider config
SLIDER_WIDTH = 18
MOUSE_FIRST_PARAM_ROW = 6
MOUSE_SLIDER_START_COL = 21

MOUSE_ENABLE = "\x1b[?1000h"
MOUSE_DISABLE = "\x1b[?1000l"


class ParamTuner:
    def __init__(self, navigator, app_context):
        self.nav = navigator
        self.app = app_context
        self.console = Console()
        self.live = None
        self.running = False
        self._termios_attrs = None
        self._thread = None

        # List of parameters: (Label, attribute_name, step_size, min_val, max_val, format)
        self.params = [
            ("Max Speed", "max_speed", 0.01, 0.05, 1.0, "{:.2f}"),
            ("Min Speed", "min_speed", 0.01, 0.0, 0.5, "{:.2f}"),
            ("Safety Dist", "safety_distance", 0.05, 0.1, 2.0, "{:.2f}m"),
            ("Bubble Rad", "robot_bubble_radius", 0.01, 0.1, 1.0, "{:.2f}m"),
            ("Safe Buffer", "safety_buffer", 0.01, 0.0, 0.5, "{:.2f}m"),
            ("Smoothing", "alpha_steering", 0.05, 0.0, 1.0, "{:.2f}"),
            ("Reverse Spd", "reverse_speed", 0.01, -0.5, 0.0, "{:.2f}"),
            ("Gap Thresh", "gap_threshold", 0.1, 0.1, 3.0, "{:.1f}m"),
        ]
        self.selected_idx = 0
        
        # UI Refresh configuration
        self.target_fps = 10
        self.refresh_interval = 1.0 / self.target_fps

    def start(self):
        """Start the Tuner in a separate thread."""
        if self.running: return
        
        # Setup raw input
        if HAS_TERMIOS and sys.stdin.isatty():
            try:
                self._termios_attrs = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                sys.stdout.write(MOUSE_ENABLE)
                sys.stdout.flush()
            except Exception:
                pass

        self.running = True
        
        # Start the UI thread
        self._thread = threading.Thread(target=self._ui_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the thread and restore terminal."""
        if not self.running: return
        self.running = False
        
        if self._thread:
            self._thread.join(timeout=1.0)
        
        if self._termios_attrs is not None and HAS_TERMIOS:
            try:
                sys.stdout.write(MOUSE_DISABLE)
                sys.stdout.flush()
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._termios_attrs)
            except Exception: pass
            self._termios_attrs = None

    def update(self):
        """
        Deprecated: No longer needed in main loop as the thread handles updates.
        Kept for compatibility with existing code calls.
        """
        pass

    def _ui_loop(self):
        """The main loop running in the thread."""
        try:
            with Live(console=self.console, auto_refresh=False, screen=True) as live:
                self.live = live
                last_draw = 0
                
                while self.running:
                    # 1. Process Input (Blocking with small timeout is efficient here)
                    # This allows us to eat multiple keypresses quickly
                    input_processed = False
                    while self._input_available():
                        evt = self._read_input_byte()
                        if evt:
                            self._handle_input(evt)
                            input_processed = True
                    
                    # 2. Draw Interface
                    now = time.time()
                    if input_processed or (now - last_draw > self.refresh_interval):
                        live.update(self._build_interface(), refresh=True)
                        last_draw = now
                    
                    # Short sleep to prevent 100% CPU usage in the thread
                    time.sleep(0.01)
                    
        except Exception as e:
            # Emergency restore if thread crashes
            if self._termios_attrs is not None and HAS_TERMIOS:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._termios_attrs)
            print(f"Tuner UI Error: {e}")

    def _input_available(self):
        if not HAS_TERMIOS: return False
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        return bool(rlist)

    def _read_input_byte(self):
        try:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                buf = self._read_escape_seq(timeout=0.01)
                if buf == "[A": return KEY_UP
                if buf == "[B": return KEY_DOWN
                if buf == "[C": return KEY_RIGHT
                if buf == "[D": return KEY_LEFT
                m = re.match(r"\[<(\d+);(\d+);(\d+);([Mm])", buf)
                if m:
                    return ("mouse", int(m.group(3)), int(m.group(2)), int(m.group(1)))
            return None
        except OSError: return None

    def _read_escape_seq(self, timeout=0.01):
        buf = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._input_available(): break
            c = sys.stdin.read(1)
            if not c: break
            buf.append(c)
            if buf[-1] in "Mm" or (len(buf) >= 2 and buf[-1] in "ABCD"): break
        return "".join(buf)

    def _handle_input(self, evt):
        if evt == KEY_UP:
            self.selected_idx = max(0, self.selected_idx - 1)
        elif evt == KEY_DOWN:
            self.selected_idx = min(len(self.params) - 1, self.selected_idx + 1)
        elif evt == KEY_LEFT:
            self._change_value(-1)
        elif evt == KEY_RIGHT:
            self._change_value(1)
        elif isinstance(evt, tuple) and evt[0] == "mouse":
            self._handle_mouse_click(evt[1], evt[2], evt[3])

    def _handle_mouse_click(self, row, col, button):
        param_idx = row - MOUSE_FIRST_PARAM_ROW
        if 0 <= param_idx < len(self.params):
            self.selected_idx = param_idx
            rel = (col - MOUSE_SLIDER_START_COL) / SLIDER_WIDTH
            rel = max(0.0, min(1.0, rel))
            label, attr, step, min_v, max_v, fmt = self.params[param_idx]
            new_val = min_v + rel * (max_v - min_v)
            setattr(self.nav, attr, new_val)

    def _change_value(self, direction):
        label, attr, step, min_v, max_v, fmt = self.params[self.selected_idx]
        current_val = getattr(self.nav, attr)
        new_val = current_val + (step * direction)
        new_val = max(min_v, min(max_v, new_val))
        setattr(self.nav, attr, new_val)

    def _slider_bar(self, val, min_v, max_v):
        r = max_v - min_v
        rel = (val - min_v) / r if r > 0 else 0
        rel = max(0, min(1, rel))
        filled = int(rel * SLIDER_WIDTH)
        return "[" + "█" * filled + "░" * (SLIDER_WIDTH - filled) + "]"

    def _build_interface(self):
        layout = Layout()

        params_table = Table(
            show_header=True, header_style="bold cyan",
            title="PARAMETERS (Arrows / Mouse)", box=box.ROUNDED, border_style="cyan"
        )
        params_table.add_column("Sel", width=3)
        params_table.add_column("Parameter", style="cyan", width=12)
        params_table.add_column("Slider", width=SLIDER_WIDTH + 2)
        params_table.add_column("Value", width=8)

        for i, (label, attr, step, min_v, max_v, fmt) in enumerate(self.params):
            val = getattr(self.nav, attr)
            val_str = fmt.format(val)
            bar = self._slider_bar(val, min_v, max_v)
            marker = " > " if i == self.selected_idx else "   "
            style = "bold reverse cyan" if i == self.selected_idx else None
            params_table.add_row(marker, label, bar, val_str, style=style)

        state_str = str(self.nav.state).replace("NavState.", "")
        state_style = "red bold" if "BRAK" in state_str or "REV" in state_str else "green"
        steering = getattr(self.nav, "last_steering", 0.0)
        critical = getattr(self.nav, "critical_distance", 0.0)

        telem_table = Table(show_header=True, header_style="bold yellow", title="TELEMETRY", box=box.ROUNDED)
        telem_table.add_column("Metric", width=15)
        telem_table.add_column("Value", width=10)
        telem_table.add_row("State", Text(state_str, style=state_style))
        telem_table.add_row("Steering", f"{steering:.2f}")
        telem_table.add_row("Critical Dist", f"{critical:.2f}m")

        layout.split_row(Layout(params_table), Layout(telem_table))
        return Panel(layout, title="ROBOCAR TUNER (Threaded)", border_style="blue", box=box.DOUBLE)
