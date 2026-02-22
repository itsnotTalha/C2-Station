import curses
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
        self.menu = [
            "Select DRIVE Controller", 
            "Select ARM Controller",
            "Toggle DRIVE ROS Node", 
            "Toggle ARM ROS Node",
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
        if controller:
            owner = controller.get("owner", "Unknown") or "Unknown"
            mode = (self.ros_manager.get_drive_mode() if role_name == "DRIVE" else self.ros_manager.get_arm_mode()) if is_active else "Manual"
            stdscr.addstr(y + 1, 4, f"[P] {owner}", curses.color_pair(2) | curses.A_BOLD)
            stdscr.addstr(y + 3, 4, f"Mode: {mode}", (curses.color_pair(2) if is_active else curses.color_pair(1)) | curses.A_BOLD)
        else:
            stdscr.addstr(y + 1, 4, "No Controller Selected", curses.color_pair(1))
        
        status = "* PUBLISHING" if is_active else "o INACTIVE"
        stdscr.addstr(y + 4, 4, f"ROS: {status}", curses.color_pair(2 if is_active else 1))

    def draw(self, stdscr):
        self.refresh_controllers()
        h, w = stdscr.getmaxyx()
        stdscr.erase()
        
        self._render_header(stdscr, w)
        self._render_role_info(stdscr, "DRIVE", self.drive, 3, self.ros_manager.get_topic_status("/buswala"))
        self._render_role_info(stdscr, "ARM", self.arm, 9, self.ros_manager.get_topic_status("/aram"))

        # Render Menu
        assignments = ControllerStore.load_assignments()
        for i, item in enumerate(self.menu):
            label = item
            if i == 2: label = f"{'Stop' if assignments.get('drive', {}).get('ros') else 'Start'} DRIVE Node"
            if i == 3: label = f"{'Stop' if assignments.get('arm', {}).get('ros') else 'Start'} ARM Node"
            
            if i == self.selection: stdscr.attron(curses.A_REVERSE)
            stdscr.addstr(15 + i, 2, f" {label} ")
            stdscr.attroff(curses.A_REVERSE)

        stdscr.addstr(h - 2, 0, HELP_TEXT_MAIN.center(w)[:w-1], curses.A_REVERSE)
        stdscr.refresh()

    def _sync_role_state(self, role, data, is_running):
        topic = "/buswala" if role == "drive" else "/aram"
        if data.get("ros", False):
            if not is_running and not self.ros_manager.get_topic_status(topic) and getattr(self, role):
                start_fn = getattr(self.joy_manager, f"start_{role}")
                enable_fn = getattr(self.ros_manager, f"enable_{role}")
                if start_fn(getattr(self, role)["path"]):
                    time.sleep(0.3)
                    enable_fn()
        elif is_running:
            getattr(self.ros_manager, f"disable_{role}")()
            getattr(self.joy_manager, f"stop_{role}")()

    def refresh_controllers(self):
        self.controllers = ControllerStore.find_joysticks()
        assignments = ControllerStore.load_assignments()
        
        for role in ["drive", "arm"]:
            data = assignments.get(role, {})
            if isinstance(data, str): data = {"mac": data, "ros": False}
            mac = data.get("mac")
            
            # Update internal selection
            match = next((c for c in self.controllers if c["mac"] == mac), None)
            setattr(self, role, match)

            # Manage Lifecycle
            is_running = self.joy_manager.drive_process if role == "drive" else self.joy_manager.arm_process
            self._sync_role_state(role, data, is_running)
        
        self.ros_manager.publish_status(self.drive, self.arm)

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
        sel = 0
        while True:
            self.controllers = ControllerStore.find_joysticks()
            assignments = ControllerStore.load_assignments()
            in_use = {v.get("mac") if isinstance(v, dict) else v for k, v in assignments.items() if k != role}
            available = [i for i, d in enumerate(self.controllers) if d.get("mac") not in in_use and d.get("mac") != (exclude.get("mac") if exclude else None)]

            stdscr.clear()
            stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(4))
            
            if not self.controllers:
                stdscr.addstr(6, 6, "⚠️  NO CONTROLLERS FOUND", curses.color_pair(1) | curses.A_BOLD)
                stdscr.addstr(8, 6, "Check USB connections or Bluetooth pairing.", curses.A_DIM)
            else:
                for i, d in enumerate(self.controllers):
                    row = 4 + (i * 3)
                    is_unavail = i not in available
                    if not is_unavail and i == sel: stdscr.attron(curses.A_REVERSE)
                    stdscr.addstr(row, 6, f"{'[X] ' if is_unavail else ''}{d.get('owner') or d['name']}", curses.color_pair(1) if is_unavail else 0)
                    stdscr.attroff(curses.A_REVERSE)

            stdscr.addstr(stdscr.getmaxyx()[0] - 2, 4, HELP_TEXT_POPUP, curses.A_REVERSE)
            stdscr.refresh()
            
            key = stdscr.getch()
            if key_match(key, "SELECT") and self.controllers and sel in available:
                chosen = self.controllers[sel]
                assignments[role] = {"mac": chosen["mac"], "ros": False}
                ControllerStore.save_assignments(assignments)
                break
            elif key_match(key, "BACK") or key_match(key, "QUIT"): break
            else: sel = self._handle_navigation(key, sel, len(self.controllers), available)

    def confirm_reset(self, stdscr):
        """Dedicated sub-menu for confirming configuration reset."""
        c_sel = 1 # Default to 'Cancel'
        options = ["CONFIRM RESET (Wipe All Data)", "CANCEL (Keep Settings)"]
        
        while True:
            stdscr.clear()
            stdscr.addstr(4, 4, "⚠️  RESET CONFIGURATION?", curses.A_BOLD | curses.color_pair(1))
            stdscr.addstr(6, 4, "This will disconnect all controllers and stop ROS nodes.")
            
            for i, opt in enumerate(options):
                if i == c_sel: stdscr.attron(curses.A_REVERSE)
                stdscr.addstr(9 + i, 6, f" {opt} ")
                stdscr.attroff(curses.A_REVERSE)
            
            stdscr.refresh()
            key = stdscr.getch()
            
            if key_match(key, "UP") or key_match(key, "DOWN"):
                c_sel = 1 - c_sel # Toggle between 0 and 1
            elif key_match(key, "SELECT"):
                if c_sel == 0:
                    ControllerStore.save_assignments({})
                    self._show_message(stdscr, "Success", "Configuration has been wiped.")
                    return True
                return False
            elif key_match(key, "BACK") or key_match(key, "QUIT"):
                return False

    def confirm_exit(self, stdscr):
        """Dedicated sub-menu for confirming exit."""
        c_sel = 1 # Default to 'Cancel'
        options = ["CONFIRM EXIT (Shutdown)", "CANCEL (Stay in Menu)"]
        
        while True:
            stdscr.clear()
            stdscr.addstr(4, 4, "⚠️  EXIT APPLICATION?", curses.A_BOLD | curses.color_pair(1))
            stdscr.addstr(6, 4, "This will stop all ROS nodes and exit the program.")
            
            for i, opt in enumerate(options):
                if i == c_sel: stdscr.attron(curses.A_REVERSE)
                stdscr.addstr(9 + i, 6, f" {opt} ")
                stdscr.attroff(curses.A_REVERSE)
            
            stdscr.refresh()
            key = stdscr.getch()
            
            if key_match(key, "UP") or key_match(key, "DOWN"):
                c_sel = 1 - c_sel # Toggle between 0 and 1
            elif key_match(key, "SELECT"):
                return c_sel == 0  # True if confirmed, False if cancelled
            elif key_match(key, "BACK") or key_match(key, "QUIT"):
                return False
            
    def _handle_menu_selection(self, stdscr):
        if self.selection == 0: self.popup_select(stdscr, "Select DRIVE Controller", "drive", self.arm)
        elif self.selection == 1: self.popup_select(stdscr, "Select ARM Controller", "arm", self.drive)
        elif self.selection in (2, 3):
            role = "drive" if self.selection == 2 else "arm"
            assignments = ControllerStore.load_assignments()
            data = assignments.get(role, {})
            if data.get("mac"):
                data["ros"] = not data.get("ros", False)
                ControllerStore.save_assignments(assignments)
            else: self._show_message(stdscr, "Error", f"Please select a {role.upper()} controller first.")
        elif self.selection == 4: stdscr.clear(); stdscr.refresh()
        elif self.selection == 5:
            self.confirm_reset(stdscr) # Call the manual confirmation menu
        elif self.selection == 6:
            if self.confirm_exit(stdscr): return True  # Only exit if confirmed
        return False

    def run(self, stdscr):
        curses.curs_set(0)
        curses.start_color()
        for i, color in enumerate([curses.COLOR_RED, curses.COLOR_GREEN, curses.COLOR_CYAN, curses.COLOR_YELLOW], 1):
            curses.init_pair(i, color, curses.COLOR_BLACK)
        
        stdscr.timeout(100)
        stdscr.keypad(True)
        self.ros_manager.start()

        try:
            while True:
                self.draw(stdscr)
                key = stdscr.getch()
                if key == -1: continue
                if key_match(key, "QUIT"):
                    if self.confirm_exit(stdscr):
                        break
                if key_match(key, "REFRESH"): self.refresh_controllers()
                elif key_match(key, "SELECT"):
                    if self._handle_menu_selection(stdscr): break
                else: self.selection = self._handle_navigation(key, self.selection, len(self.menu))
        finally:
            self.joy_manager.cleanup()
            self.ros_manager.stop()
            ControllerStore.save_assignments({})

    def _show_message(self, stdscr, title, message):
        stdscr.clear()
        stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(4, 4, message)
        stdscr.addstr(6, 4, "Press any key to continue...")
        stdscr.refresh()
        stdscr.getch()