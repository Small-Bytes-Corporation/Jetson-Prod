"""
Ncurses-based parameter tuner for Lidar Navigation.
Allows modifying variables at runtime.
"""

import curses
import time

class ParamTuner:
    def __init__(self, navigator, app_context):
        """
        Args:
            navigator: Instance of LidarNavigator.
            app_context: Instance of ManualDriveApp (to read sensor data).
        """
        self.nav = navigator
        self.app = app_context
        self.stdscr = None
        self.running = False
        
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
        self.draw_rate = 0.1 # 10Hz UI update

    def start(self):
        """Initialize curses."""
        if self.running: return
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.keypad(True)
        self.stdscr.nodelay(True) # Non-blocking input
        curses.curs_set(0) # Hide cursor
        
        # Colors
        curses.start_color()
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN) # Highlight
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK) # Values
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)   # Alert
        
        self.running = True

    def stop(self):
        """Restore terminal."""
        if not self.running: return
        self.running = False
        curses.nocbreak()
        self.stdscr.keypad(False)
        curses.echo()
        curses.endwin()

    def update(self):
        """Called inside the main loop to handle input and drawing."""
        if not self.running: return

        # 1. Handle Input
        try:
            key = self.stdscr.getch()
            if key != -1:
                self._handle_input(key)
        except curses.error:
            pass

        # 2. Draw (Limit FPS to save CPU)
        now = time.time()
        if now - self.last_draw > self.draw_rate:
            self._draw_interface()
            self.last_draw = now

    def _handle_input(self, key):
        if key == curses.KEY_UP:
            self.selected_idx = max(0, self.selected_idx - 1)
        elif key == curses.KEY_DOWN:
            self.selected_idx = min(len(self.params) - 1, self.selected_idx + 1)
        elif key == curses.KEY_LEFT:
            self._change_value(-1)
        elif key == curses.KEY_RIGHT:
            self._change_value(1)

    def _change_value(self, direction):
        # Get current param definition
        label, attr, step, min_v, max_v, fmt = self.params[self.selected_idx]
        
        # Get current value
        current_val = getattr(self.nav, attr)
        
        # Calculate new value
        new_val = current_val + (step * direction)
        new_val = max(min_v, min(max_v, new_val))
        
        # Set new value
        setattr(self.nav, attr, new_val)

    def _draw_interface(self):
        try:
            self.stdscr.erase()
            h, w = self.stdscr.getmaxyx()
            
            # Title
            title = " ROBOCAR RUNTIME TUNER "
            self.stdscr.addstr(0, (w - len(title))//2, title, curses.A_BOLD | curses.A_REVERSE)
            
            # Telemetry Column (Right side)
            col_x = w // 2 + 2
            self.stdscr.addstr(2, col_x, "--- TELEMETRY ---", curses.A_BOLD)
            
            state_str = str(self.nav.state).replace("NavState.", "")
            color = curses.color_pair(3) if "BRAK" in state_str or "REV" in state_str else curses.color_pair(2)
            self.stdscr.addstr(4, col_x, f"State: {state_str}", color)
            
            # Compute live vars
            steering = getattr(self.nav, 'last_steering', 0.0)
            critical = getattr(self.nav, 'critical_distance', 0.0)
            
            self.stdscr.addstr(5, col_x, f"Steering: {steering:.2f}")
            self.stdscr.addstr(6, col_x, f"Critical Dist: {critical:.2f}m")
            
            # Parameters Column (Left side)
            self.stdscr.addstr(2, 2, "--- PARAMETERS (UP/DWN to select, L/R to change) ---", curses.A_BOLD)
            
            for i, (label, attr, step, min_v, max_v, fmt) in enumerate(self.params):
                val = getattr(self.nav, attr)
                val_str = fmt.format(val)
                
                row_str = f"{label:<15}: {val_str:>8}"
                
                style = curses.A_NORMAL
                if i == self.selected_idx:
                    style = curses.color_pair(1)
                    row_str = f"> {row_str} <"
                else:
                    row_str = f"  {row_str}  "
                
                self.stdscr.addstr(4 + i, 2, row_str, style)

            # Footer
            footer = "Press Ctrl+C to Exit"
            self.stdscr.addstr(h-1, 2, footer, curses.A_DIM)

            self.stdscr.refresh()
        except curses.error:
            pass
