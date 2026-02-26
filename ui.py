import curses
import random
import time
from controller_store import ControllerStore
from joy_manager import JoyManager
from ros_manager import RosManager

# ====================================================================
#                    KEY CONFIGURATION SECTION
# ====================================================================
KEYS = {
    "UP":      [curses.KEY_UP, ord('8')],
    "DOWN":    [curses.KEY_DOWN, ord('2')],
    "LEFT":    [curses.KEY_LEFT, ord('4')],
    "RIGHT":   [curses.KEY_RIGHT, ord('6')],
    "HOME":    [curses.KEY_HOME, ord('7')],
    "END":     [curses.KEY_END, ord('1')],
    "PGUP":    [curses.KEY_PPAGE, ord('9')],
    "PGDN":    [curses.KEY_NPAGE, ord('3')],
    "SELECT":  [10, 13, ord('5')],
    "BACK":    [curses.KEY_LEFT, ord('4'), curses.KEY_DC, ord('.')],
    "REFRESH": [curses.KEY_IC, ord('0')],
    "QUIT":    [ord('.'), curses.KEY_DC],
}

PAGE_JUMP_SIZE = 3
HELP_TEXT_MAIN  = "8/2:Up/Dn | 7/1:Home/End | 5:Select | 0:Refresh | .:Quit"
HELP_TEXT_POPUP = "8/2:Nav | 5:Select | 0:Refresh | 4/.:Back"
# ====================================================================

def key_match(key, action):
    return key in KEYS.get(action, [])

class ControllerUI:
    def __init__(self):
        self.ros_manager = RosManager()
        self.joy_manager = JoyManager()
        self.controllers = []
        self.drive = None
        self.arm = None
        self.selection = 0
        
        # BMS Simulation state
        self.bms_voltage = 25.2
        self.bms_status = "ONLINE"
        self.start_time = time.time() 
        self.last_bms_update = 0
        self.update_interval = 3.0  # Update battery values every 2 seconds
        self.cells = [4.20] * 6 
        self.battery_depleted = False
        self.final_runtime = None
        
        self.menu = [
            "DRIVE Controller Settings", 
            "ARM Controller Settings",
            "Toggle DRIVE Joy Node", 
            "Toggle ARM Joy Node",
            "Clear Screen", 
            "Reset Configuration", 
            "Exit"
        ]

    def _render_header(self, stdscr, w):
        stdscr.addstr(1, 2, "ROS CONTROLLER CONFIGURATION", curses.A_BOLD | curses.color_pair(4))
        status_text = "[ROS2 OK]" if self.ros_manager.available else "[ROS2 N/A]"
        status_color = curses.color_pair(2) if self.ros_manager.available else curses.color_pair(1)
        stdscr.addstr(1, 35, status_text, status_color)

    def _render_role_info(self, stdscr, role_name, controller, y, is_active):
        stdscr.addstr(y, 2, role_name, curses.A_BOLD | curses.color_pair(3))
        
        assignments = ControllerStore.load_assignments()
        is_deselected = assignments.get(role_name.lower(), {}).get("deselected", False)
        
        if controller:
            owner = controller.get("owner")
            name = controller.get("name", "Unknown")
            mac = controller.get("mac", "N/A")
            display = owner if owner else name
            stdscr.addstr(y + 1, 4, f"{display}", curses.color_pair(2) | curses.A_BOLD)
            stdscr.addstr(y + 2, 4, f"{name[:30]}  |  {mac[:24]}", curses.A_DIM)
        elif is_deselected:
            stdscr.addstr(y + 1, 4, "DESELECTED", curses.color_pair(1))
        else:
            stdscr.addstr(y + 1, 4, "No Controller", curses.color_pair(1))
        
        status = "* RUNNING" if is_active else "o STOPPED"
        stdscr.addstr(y + 3, 4, f"Joy: {status}", curses.color_pair(2 if is_active else 1))

    def _get_progress_status(self, percent):
        """Calculates a wide 20-segment bar synchronized with percentage."""
        percent = max(0, min(100, percent))
        bar_width = 20
        num_blocks = int((percent / 100) * bar_width)
        bar = "[" + "█" * num_blocks + "░" * (bar_width - num_blocks) + "]"

        if percent >= 90:
            return "🟢", bar, "Excellent", curses.color_pair(2)
        elif percent >= 70:
            return "🟢", bar, "High", curses.color_pair(2)
        elif percent >= 40:
            return "🟡", bar, "Medium", curses.color_pair(4)
        elif percent >= 15:
            return "🟠", bar, "Low", curses.color_pair(4)
        else:
            return "🔴", bar, "Critical", curses.color_pair(1)

    def _render_bms_panel(self, stdscr, x_offset):
        """Renders 6S monitoring with throttled updates and depletion logic."""
        current_time = time.time()

        # 1. Throttled Logic: Only calculate if not depleted and interval has passed
        if not self.battery_depleted and (current_time - self.last_bms_update) > self.update_interval:
            # Randomize Each Cell independently
            for i in range(6):
                # Slower drain for testing: -0.005 to 0.001
                self.cells[i] += random.uniform(-0.005, 0.001)
                self.cells[i] = max(3.5, min(4.2, self.cells[i]))
            
            self.last_bms_update = current_time

            # Check for depletion (3.5V per cell avg = 21.0V)
            if sum(self.cells) <= 21.01:
                self.battery_depleted = True
                self.final_runtime = int(time.time() - self.start_time)

        # 2. Derived Values
        total_voltage = sum(self.cells)
        percentage = int(((total_voltage - 21.0) / (25.2 - 21.0)) * 100)
        percentage = max(0, min(100, percentage))
        emoji, bar, status_txt, color = self._get_progress_status(percentage)

        # 3. Time Calculations
        if self.battery_depleted:
            elapsed_sec = self.final_runtime
        else:
            elapsed_sec = int(time.time() - self.start_time)
            
        elapsed_str = f"{elapsed_sec // 60:02d}:{elapsed_sec % 60:02d}"
        est_minutes_left = int((percentage / 100) * 60) if not self.battery_depleted else 0

        # --- UI Rendering ---
        stdscr.addstr(2, x_offset, "─── BMS MONITOR ───", curses.A_BOLD | curses.color_pair(4))
        
        # Status Line
        if self.battery_depleted:
            stdscr.addstr(4, x_offset, "Status: DEPLETED ⚠️", curses.color_pair(1) | curses.A_BLINK | curses.A_BOLD)
        else:
            stdscr.addstr(4, x_offset, f"Status: ONLINE {emoji}")

        stdscr.addstr(5, x_offset, f"Total:  {total_voltage:.2f}V ({percentage}%)", color | curses.A_BOLD)
        
        # Wide Bar
        stdscr.addstr(6, x_offset, f"{bar}", color)
        stdscr.addstr(7, x_offset, f"Condition: {status_txt}", color)

        # 4. Individual Cell Display
        stdscr.addstr(9, x_offset, "Cells (V):", curses.A_UNDERLINE | curses.A_BOLD)
        stdscr.addstr(10, x_offset, f"C1: {self.cells[0]:.2f}  C2: {self.cells[1]:.2f}  C3: {self.cells[2]:.2f}")
        stdscr.addstr(11, x_offset, f"C4: {self.cells[3]:.2f}  C5: {self.cells[4]:.2f}  C6: {self.cells[5]:.2f}")

        # 5. Runtime & Estimation
        stdscr.addstr(13, x_offset, f"Run Time:   {elapsed_str}", curses.A_BOLD | (curses.color_pair(1) if self.battery_depleted else color))
        
        if self.battery_depleted:
            stdscr.addstr(14, x_offset, "!!! CRITICAL: RECHARGE !!!", curses.color_pair(1) | curses.A_BOLD | curses.A_BLINK)
        else:
            rem_color = color if percentage > 15 else curses.color_pair(1) | curses.A_BLINK
            stdscr.addstr(14, x_offset, f"Est. Rem:  {est_minutes_left} min", rem_color)

        # Current draw (zeros out if depleted)
        current = (7.5 + random.uniform(-0.1, 0.3)) if not self.battery_depleted else 0.0
        stdscr.addstr(15, x_offset, f"Total Draw: {current:.1f}A", curses.A_BOLD)

    def draw(self, stdscr):
        self.refresh_controllers()
        h, w = stdscr.getmaxyx()
        stdscr.erase()
        
        self._render_header(stdscr, w)
        
        # Left Column: Controllers
        self._render_role_info(stdscr, "DRIVE", self.drive, 3, self.joy_manager.drive_process is not None)
        self._render_role_info(stdscr, "ARM", self.arm, 10, self.joy_manager.arm_process is not None)

        # Right Column: BMS Status (Offset by 45 characters)
        if w > 60:
            self._render_bms_panel(stdscr, 65)

        # Render Menu
        for i, item in enumerate(self.menu):
            label = item
            if i == 2: label = f"{'Stop' if self.joy_manager.drive_process else 'Start'} DRIVE Node"
            if i == 3: label = f"{'Stop' if self.joy_manager.arm_process else 'Start'} ARM Node"
            
            if i == self.selection: stdscr.attron(curses.A_REVERSE)
            stdscr.addstr(16 + i, 2, f" {label} ")
            stdscr.attroff(curses.A_REVERSE)

        stdscr.addstr(h - 2, 0, HELP_TEXT_MAIN.center(w)[:w-1], curses.A_REVERSE)
        stdscr.refresh()

    def refresh_controllers(self):
        self.controllers = ControllerStore.find_joysticks()
        assignments = ControllerStore.load_assignments()
        
        for role in ["drive", "arm"]:
            controller = ControllerStore.find_controller_for_role(role)
            if controller:
                current_data = assignments.get(role, {})
                if not current_data.get("deselected", False):
                    setattr(self, role, controller)
                    assignments[role] = {"mac": controller["mac"], "path": controller["path"]}
                else:
                    setattr(self, role, None)
            else:
                setattr(self, role, None)
                if role in assignments:
                    del assignments[role]
        
        ControllerStore.save_assignments(assignments)

    def _handle_navigation(self, key, current_sel, max_val, available_indices=None):
        indices = available_indices if available_indices is not None else list(range(max_val))
        if not indices: return current_sel

        curr_idx = indices.index(current_sel) if current_sel in indices else 0
        if key_match(key, "UP"): return indices[(curr_idx - 1) % len(indices)]
        if key_match(key, "DOWN"): return indices[(curr_idx + 1) % len(indices)]
        if key_match(key, "HOME"): return indices[0]
        if key_match(key, "END"): return indices[-1]
        if key_match(key, "PGUP"): return indices[max(0, curr_idx - PAGE_JUMP_SIZE)]
        if key_match(key, "PGDN"): return indices[min(len(indices) - 1, curr_idx + PAGE_JUMP_SIZE)]
        return current_sel

    def popup_select(self, stdscr, title, role, exclude=None):
        need_redraw = True
        sel = 0
        stdscr.timeout(200) 

        while True:
            controller = ControllerStore.find_controller_for_role(role)
            assignments = ControllerStore.load_assignments()
            is_deselected = assignments.get(role, {}).get("deselected", False)
            
            if need_redraw:
                stdscr.erase()
                h, w = stdscr.getmaxyx()
                stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(4))
                
                from controller_store import ROLE_PATHS
                expected_path = ROLE_PATHS.get(role, "Unknown")
                stdscr.addstr(4, 4, f"Scanning: {expected_path}", curses.A_DIM)
                
                if controller:
                    stdscr.addstr(6, 4, "Controller Found:", curses.color_pair(2) | curses.A_BOLD)
                    name = controller.get("name", "Unknown")
                    owner = controller.get("owner")
                    mac = controller.get("mac", "N/A")
                    path = controller.get("path", "N/A")
                    
                    if owner: stdscr.addstr(8, 6, f"Owner: {owner}", curses.A_BOLD)
                    stdscr.addstr(9, 6, f"Name:  {name}")
                    stdscr.addstr(10, 6, f"MAC:   {mac[:24]}")
                    stdscr.addstr(11, 6, f"Path:  {path}")
                    
                    status_lbl = "DESELECTED (Manual)" if is_deselected else "AUTO-ASSIGNED"
                    status_col = curses.color_pair(1) if is_deselected else curses.color_pair(2)
                    stdscr.addstr(13, 6, f"Status: {status_lbl}", status_col)
                    
                    opts = ["Re-select (Enable)", "Deselect (Ignore)"] if is_deselected else ["Deselect (Ignore)"]
                    for i, opt in enumerate(opts):
                        if i == sel: stdscr.attron(curses.A_REVERSE)
                        stdscr.addstr(16 + i, 6, f" {opt} ")
                        stdscr.attroff(curses.A_REVERSE)
                else:
                    stdscr.addstr(6, 4, "No Controller Connected", curses.color_pair(1) | curses.A_BOLD)
                    if is_deselected:
                        if sel == 0: stdscr.attron(curses.A_REVERSE)
                        stdscr.addstr(12, 6, " Re-enable Auto-assign ")
                        stdscr.attroff(curses.A_REVERSE)

                stdscr.addstr(h - 2, 4, "5:Select | 4/.:Back", curses.A_REVERSE)
                stdscr.refresh()
                need_redraw = False
            
            key = stdscr.getch()
            if key == -1:
                new_c = ControllerStore.find_controller_for_role(role)
                if (new_c is None) != (controller is None):
                    need_redraw = True
                    sel = 0
                continue
            
            if key_match(key, "UP") or key_match(key, "DOWN"):
                if controller and is_deselected: sel = 1 - sel
                need_redraw = True
            elif key_match(key, "SELECT"):
                if controller:
                    if is_deselected and sel == 0:
                        assignments[role] = {"mac": controller["mac"], "path": controller["path"]}
                    else:
                        assignments[role] = {"deselected": True}
                    ControllerStore.save_assignments(assignments)
                elif is_deselected:
                    if role in assignments: del assignments[role]
                    ControllerStore.save_assignments(assignments)
                break
            elif key_match(key, "BACK") or key_match(key, "QUIT"): break

    def confirm_reset(self, stdscr):
        c_sel = 1
        options = ["CONFIRM RESET (Wipe All Data)", "CANCEL (Keep Settings)"]
        need_redraw = True
        stdscr.timeout(200)
        
        while True:
            if need_redraw:
                stdscr.erase()
                stdscr.addstr(4, 4, "⚠️  RESET CONFIGURATION?", curses.A_BOLD | curses.color_pair(1))
                for i, opt in enumerate(options):
                    if i == c_sel: stdscr.attron(curses.A_REVERSE)
                    stdscr.addstr(9 + i, 6, f" {opt} ")
                    stdscr.attroff(curses.A_REVERSE)
                stdscr.refresh()
                need_redraw = False
            
            key = stdscr.getch()
            if key_match(key, "UP") or key_match(key, "DOWN"):
                c_sel = 1 - c_sel
                need_redraw = True
            elif key_match(key, "SELECT"):
                if c_sel == 0:
                    ControllerStore.save_assignments({})
                    return True
                return False
            elif key_match(key, "BACK") or key_match(key, "QUIT"): return False

    def confirm_exit(self, stdscr):
        c_sel = 1
        options = ["CONFIRM EXIT (Shutdown)", "CANCEL (Stay in Menu)"]
        need_redraw = True
        stdscr.timeout(200)
        while True:
            if need_redraw:
                stdscr.erase()
                stdscr.addstr(4, 4, "⚠️  EXIT APPLICATION?", curses.A_BOLD | curses.color_pair(1))
                for i, opt in enumerate(options):
                    if i == c_sel: stdscr.attron(curses.A_REVERSE)
                    stdscr.addstr(9 + i, 6, f" {opt} ")
                    stdscr.attroff(curses.A_REVERSE)
                stdscr.refresh()
                need_redraw = False
            key = stdscr.getch()
            if key_match(key, "UP") or key_match(key, "DOWN"):
                c_sel = 1 - c_sel
                need_redraw = True
            elif key_match(key, "SELECT"): return c_sel == 0
            elif key_match(key, "BACK") or key_match(key, "QUIT"): return False
            
    def _handle_menu_selection(self, stdscr):
        if self.selection == 0: self.popup_select(stdscr, "DRIVE Controller Settings", "drive", self.arm)
        elif self.selection == 1: self.popup_select(stdscr, "ARM Controller Settings", "arm", self.drive)
        elif self.selection in (2, 3):
            from controller_store import ROLE_PATHS
            role = "drive" if self.selection == 2 else "arm"
            path = ROLE_PATHS.get(role)
            start_fn = getattr(self.joy_manager, f"start_{role}")
            stop_fn = getattr(self.joy_manager, f"stop_{role}")
            is_running = getattr(self.joy_manager, f"{role}_process") is not None
            if not is_running:
                start_fn(path)
                if role == "drive": self.ros_manager.enable_drive()
            else:
                stop_fn()
                if role == "drive": self.ros_manager.disable_drive()
        elif self.selection == 4: stdscr.clear(); stdscr.refresh()
        elif self.selection == 5: self.confirm_reset(stdscr)
        elif self.selection == 6:
            if self.confirm_exit(stdscr): return True
        return False

    def run(self, stdscr):
        curses.curs_set(0)
        curses.start_color()
        for i, color in enumerate([curses.COLOR_RED, curses.COLOR_GREEN, curses.COLOR_CYAN, curses.COLOR_YELLOW], 1):
            curses.init_pair(i, color, curses.COLOR_BLACK)
        
        stdscr.timeout(100)
        stdscr.keypad(True)
        self.ros_manager.start()

        from controller_store import ROLE_PATHS
        self.joy_manager.start_drive(ROLE_PATHS.get("drive"))
        self.ros_manager.enable_drive()
        self.joy_manager.start_arm(ROLE_PATHS.get("arm"))

        try:
            while True:
                self.draw(stdscr)
                key = stdscr.getch()
                if key == -1: continue
                if key_match(key, "QUIT"):
                    if self.confirm_exit(stdscr): break
                elif key_match(key, "SELECT"):
                    if self._handle_menu_selection(stdscr): break
                else: self.selection = self._handle_navigation(key, self.selection, len(self.menu))
        finally:
            self.joy_manager.cleanup()
            self.ros_manager.stop()
            # Removed the assignment wipe from finally to persist config on exit