"""
Rich-based parameter tuner for Lidar Navigation.
Allows modifying variables at runtime. Works without TTY (unlike curses).
"""

import sys
import time
import select

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

# Key constants (match curses semantics)
KEY_UP = "up"
KEY_DOWN = "down"
KEY_LEFT = "left"
KEY_RIGHT = "right"


class ParamTuner:
    def __init__(self, navigator, app_context):
        """
        Args:
            navigator: Instance of LidarNavigator.
            app_context: Instance of ManualDriveApp (to read sensor data).
        """
        self.nav = navigator
        self.app = app_context
        self.console = Console()
        self.live = None
        self.running = False
        self._termios_attrs = None

        # List of parameters to tune: (Label, attribute_name, step_size, min_val, max_val, format)
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
        self.draw_rate = 0.1  # 10Hz UI update

    def start(self):
        """Initialize Rich Live display. Keyboard works if stdin is a TTY (Unix)."""
        if self.running:
            return
        try:
            self.live = Live(
                console=self.console,
                refresh_per_second=10,
                transient=False,
            )
            self.live.start()
            # Setup raw stdin for keyboard (Unix only, when TTY)
            if HAS_TERMIOS and sys.stdin.isatty():
                self._termios_attrs = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
            self.running = True
        except Exception as e:
            print(f"[Tuner] Start failed: {e}")

    def stop(self):
        """Restore terminal and stop Live."""
        if not self.running:
            return
        self.running = False
        if self._termios_attrs is not None and HAS_TERMIOS:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._termios_attrs)
            except Exception:
                pass
            self._termios_attrs = None
        if self.live is not None:
            try:
                self.live.stop()
            except Exception:
                pass
            self.live = None

    def _read_key(self):
        """Non-blocking read of a single key. Returns None if no key, or KEY_UP/DOWN/LEFT/RIGHT."""
        if not HAS_TERMIOS or not sys.stdin.isatty():
            return None
        # Check if stdin has data (non-blocking)
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if not rlist:
            return None
        try:
            ch = sys.stdin.read(1)
            if ch == "\x1b":  # ESC - might be arrow key
                # Read rest of escape sequence (e.g. [A for up)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    seq = sys.stdin.read(2)
                    if seq == "[A":
                        return KEY_UP
                    if seq == "[B":
                        return KEY_DOWN
                    if seq == "[C":
                        return KEY_RIGHT
                    if seq == "[D":
                        return KEY_LEFT
            return None
        except (BlockingIOError, OSError):
            return None

    def update(self):
        """Called inside the main loop to handle input and drawing."""
        if not self.running or self.live is None:
            return

        # 1. Handle Input
        key = self._read_key()
        if key is not None:
            self._handle_input(key)

        # 2. Draw (limit FPS to save CPU)
        now = time.time()
        if now - self.last_draw > self.draw_rate:
            self.live.update(self._build_interface())
            self.last_draw = now

    def _handle_input(self, key):
        if key == KEY_UP:
            self.selected_idx = max(0, self.selected_idx - 1)
        elif key == KEY_DOWN:
            self.selected_idx = min(len(self.params) - 1, self.selected_idx + 1)
        elif key == KEY_LEFT:
            self._change_value(-1)
        elif key == KEY_RIGHT:
            self._change_value(1)

    def _change_value(self, direction):
        label, attr, step, min_v, max_v, fmt = self.params[self.selected_idx]
        current_val = getattr(self.nav, attr)
        new_val = current_val + (step * direction)
        new_val = max(min_v, min(max_v, new_val))
        setattr(self.nav, attr, new_val)

    def _build_interface(self):
        """Build the Rich renderable for the tuner UI."""
        layout = Layout()

        # Parameters table (left)
        params_table = Table(
            show_header=True,
            header_style="bold cyan",
            title="PARAMETERS (↑↓ select, ←→ change)",
            box=box.ROUNDED,
            border_style="cyan",
        )
        params_table.add_column("", width=2)
        params_table.add_column("Parameter", style="cyan", width=15)
        params_table.add_column("Value", width=12)

        for i, (label, attr, step, min_v, max_v, fmt) in enumerate(self.params):
            val = getattr(self.nav, attr)
            val_str = fmt.format(val)
            marker = "▶" if i == self.selected_idx else " "
            row_style = "bold reverse cyan" if i == self.selected_idx else None
            params_table.add_row(marker, label, val_str, style=row_style)

        # Telemetry table (right)
        state_str = str(self.nav.state).replace("NavState.", "")
        state_style = "red bold" if "BRAK" in state_str or "REV" in state_str else "green"
        steering = getattr(self.nav, "last_steering", 0.0)
        critical = getattr(self.nav, "critical_distance", 0.0)

        telemetry_table = Table(
            show_header=True,
            header_style="bold yellow",
            title="TELEMETRY",
            box=box.ROUNDED,
            border_style="yellow",
        )
        telemetry_table.add_column("Metric", style="cyan", width=14)
        telemetry_table.add_column("Value", width=12)
        telemetry_table.add_row("State", Text(state_str, style=state_style))
        telemetry_table.add_row("Steering", f"{steering:.2f}")
        telemetry_table.add_row("Critical Dist", f"{critical:.2f}m")

        layout.split_row(
            Layout(params_table, name="params"),
            Layout(telemetry_table, name="telemetry"),
        )

        footer = Text("Press Ctrl+C to Exit", style="dim")
        return Panel(
            layout,
            title="[bold] ROBOCAR RUNTIME TUNER [/bold]",
            border_style="blue",
            box=box.DOUBLE,
            subtitle=footer,
        )
