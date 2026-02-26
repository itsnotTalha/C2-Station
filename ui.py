import curses
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
    "HOME":    [curses.KEY_HOME, ord('7')],
    "END":     [curses.KEY_END, ord('1')],
    "PGUP":    [curses.KEY_PPAGE, ord('9')],
    "PGDN":    [curses.KEY_NPAGE, ord('3')],
    "SELECT":  [10, 13, ord('5')],
    "BACK":    [curses.KEY_LEFT, ord('4'), curses.KEY_DC, ord('.')],
    "REFRESH": [curses.KEY_IC, ord('0')],
    "CLEAR":   [ord('6')],
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

    def draw(self, stdscr):
        self.refresh_controllers()
        h, w = stdscr.getmaxyx()
        stdscr.erase()
        
        self._render_header(stdscr, w)
        self._render_role_info(stdscr, "DRIVE", self.drive, 3, self.joy_manager.drive_process is not None)
        self._render_role_info(stdscr, "ARM", self.arm, 10, self.joy_manager.arm_process is not None)

        self._render_telemetry(stdscr, w)

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
            elif key_match(key, "CLEAR"):
                stdscr.clear(); stdscr.refresh()
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
    
    def _render_telemetry(self, stdscr, w):
        col_x = 60 # Position on the right side of the screen
        
        # Header
        stdscr.addstr(3, col_x, "SYSTEM KINEMATICS & STATE", curses.A_BOLD | curses.color_pair(3))
        stdscr.addstr(4, col_x, "═" * 30, curses.A_DIM)

        # Dynamic Data
        def add_stat(row, label, value, color=None):
            stdscr.addstr(row, col_x, f"{label:<18}:", curses.A_BOLD)
            if color:
                stdscr.addstr(row, col_x + 20, str(value), color)
            else:
                stdscr.addstr(row, col_x + 20, str(value))

        # Render rows using professional terminology
        add_stat(6, "Linear Velocity", f"{self.ros_manager.linear_vel:>5.2f} m/s")
        add_stat(7, "Angular Velocity", f"{self.ros_manager.angular_vel:>5.2f} rad/s")
        
        # Color code the Drive Mode
        if "Manual" in self.ros_manager.drive_mode:
            mode_color = curses.color_pair(3) # Blue
        else:
            mode_color = curses.color_pair(1)
        add_stat(9, "Drive Mode", self.ros_manager.drive_mode, mode_color)
        
        add_stat(10, "Steering Mode", self.ros_manager.steering_mode)

        # Connection status for the telemetry thread
        status_str = "ACTIVE" if self.ros_manager.is_subscribed else "LISTENING..."
        status_col = curses.color_pair(2) if self.ros_manager.is_subscribed else curses.color_pair(4)
        stdscr.addstr(13, col_x, f"Link Status: {status_str}", status_col | curses.A_DIM)

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
