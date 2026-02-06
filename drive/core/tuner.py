"""
Rich-based parameter tuner for Lidar Navigation.
Allows modifying variables at runtime.
Optimized to prevent main-loop lag.
"""

import sys
import time
import select
import re

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
        self.last_draw = 0
        
        # OPTIMIZATION: Reduce UI refresh rate to 5Hz to prevent blocking the motor loop
        self.draw_rate = 0.2 

    def start(self):
        if self.running: return
        try:
            self.live = Live(
                console=self.console,
                refresh_per_second=4, # Low auto-refresh
                transient=False,
                auto_refresh=False    # We will manually update
            )
            self.live.start()
            
            if HAS_TERMIOS and sys.stdin.isatty():
                self._termios_attrs = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                sys.stdout.write(MOUSE_ENABLE)
                sys.stdout.flush()
            self.running = True
        except Exception as e:
            print(f"[Tuner] Start failed: {e}")

    def stop(self):
        if not self.running: return
        self.running = False
        if self._termios_attrs is not None and HAS_TERMIOS:
            try:
                sys.stdout.write(MOUSE_DISABLE)
                sys.stdout.flush()
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._termios_attrs)
            except Exception: pass
            self._termios_attrs = None
        if self.live is not None:
            try:
                self.live.stop()
            except Exception: pass
            self.live = None

    def _read_input(self):
        if not HAS_TERMIOS or not sys.stdin.isatty(): return None
        # Non-blocking check
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if not rlist: return None
        try:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                buf = self._read_escape_seq(timeout=0.01) # Very short timeout
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
            rlist, _, _ = select.select([sys.stdin], [], [], 0) # Instant check
            if not rlist: break
            c = sys.stdin.read(1)
            if not c: break
            buf.append(c)
            if buf[-1] in "Mm" or (len(buf) >= 2 and buf[-1] in "ABCD"): break
        return "".join(buf)

    def update(self):
        """Called inside the main loop."""
        if not self.running or self.live is None: return

        # 1. Handle Input (Fast)
        evt = self._read_input()
        input_processed = False
        if evt is not None:
            self._handle_input(evt)
            input_processed = True

        # 2. Draw (Throttled)
        # Only draw if input happened OR enough time passed
        now = time.time()
        if input_processed or (now - self.last_draw > self.draw_rate):
            self.live.update(self._build_interface(), refresh=True)
            self.last_draw = now

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
            title="PARAMETERS (Arrows to change)", box=box.ROUNDED, border_style="cyan"
        )
        params_table.add_column("Sel", width=3)
        params_table.add_column("Param", style="cyan", width=12)
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
        return Panel(layout, title="ROBOCAR TUNER", border_style="blue", box=box.DOUBLE)
