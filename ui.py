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
        
        # Check if deselected
        assignments = ControllerStore.load_assignments()
        is_deselected = assignments.get(role_name.lower(), {}).get("deselected", False)
        
        if controller:
            owner = controller.get("owner")
            name = controller.get("name", "Unknown")
            display = owner if owner else name
            stdscr.addstr(y + 1, 4, f"{display}", curses.color_pair(2) | curses.A_BOLD)
            stdscr.addstr(y + 2, 4, f"{name[:30]}", curses.A_DIM)
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
        self._render_role_info(stdscr, "ARM", self.arm, 9, self.joy_manager.arm_process is not None)

        # Render Menu
        for i, item in enumerate(self.menu):
            label = item
            if i == 2: label = f"{'Stop' if self.joy_manager.drive_process else 'Start'} DRIVE Node"
            if i == 3: label = f"{'Stop' if self.joy_manager.arm_process else 'Start'} ARM Node"
            
            if i == self.selection: stdscr.attron(curses.A_REVERSE)
            stdscr.addstr(15 + i, 2, f" {label} ")
            stdscr.attroff(curses.A_REVERSE)

        stdscr.addstr(h - 2, 0, HELP_TEXT_MAIN.center(w)[:w-1], curses.A_REVERSE)
        stdscr.refresh()

    def refresh_controllers(self):
        """Refresh hardware state and auto-assign controllers based on their paths."""
        self.controllers = ControllerStore.find_joysticks()
        assignments = ControllerStore.load_assignments()
        
        # Auto-assign controllers based on which path they're connected to
        for role in ["drive", "arm"]:
            # Check if a controller is connected to this role's specific path
            controller = ControllerStore.find_controller_for_role(role)
            
            if controller:
                # Controller found on this role's path - auto-assign if not manually deselected
                current_data = assignments.get(role, {})
                if not current_data.get("deselected", False):
                    # Auto-assign this controller
                    setattr(self, role, controller)
                    assignments[role] = {"mac": controller["mac"], "path": controller["path"]}
                else:
                    # User manually deselected - don't auto-assign
                    setattr(self, role, None)
            else:
                # No controller on this path
                setattr(self, role, None)
                # Clear assignment if controller disconnected
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
        """
        Shows controller status for a specific role with deselect option.
        Controllers are auto-assigned based on path, but user can deselect.
        """
        need_redraw = True
        sel = 0  # 0 = controller info, 1 = deselect option
        
        stdscr.timeout(200) 

        while True:
            # Scan for controller on this role's specific path
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
                    # Show controller info
                    stdscr.addstr(6, 4, "Controller Found:", curses.color_pair(2) | curses.A_BOLD)
                    
                    name = controller.get("name", "Unknown")
                    owner = controller.get("owner")
                    mac = controller.get("mac", "N/A")
                    path = controller.get("path", "N/A")
                    
                    if owner:
                        stdscr.addstr(8, 6, f"Owner: {owner}", curses.A_BOLD)
                    stdscr.addstr(9, 6, f"Name:  {name}")
                    stdscr.addstr(10, 6, f"MAC:   {mac[:24]}")
                    stdscr.addstr(11, 6, f"Path:  {path}")
                    
                    if is_deselected:
                        stdscr.addstr(13, 6, "Status: DESELECTED (Manual Override)", curses.color_pair(1))
                    else:
                        stdscr.addstr(13, 6, "Status: AUTO-ASSIGNED", curses.color_pair(2))
                    
                    # Options
                    options = ["Re-select (Use this controller)", "Deselect (Ignore this controller)"] if is_deselected else ["Deselect (Ignore this controller)"]
                    
                    for i, opt in enumerate(options):
                        row = 16 + i
                        if i == sel:
                            stdscr.attron(curses.A_REVERSE)
                        stdscr.addstr(row, 6, f" {opt} ")
                        stdscr.attroff(curses.A_REVERSE)
                else:
                    stdscr.addstr(6, 4, "No Controller Connected", curses.color_pair(1) | curses.A_BOLD)
                    stdscr.addstr(8, 6, f"Plug a controller into cable {expected_path.split('/')[-1]}")
                    
                    # If was deselected, offer to re-enable auto-assign
                    if is_deselected:
                        stdscr.addstr(10, 6, "Auto-assign is disabled for this role.", curses.A_DIM)
                        if sel == 0:
                            stdscr.attron(curses.A_REVERSE)
                        stdscr.addstr(12, 6, " Re-enable Auto-assign ")
                        stdscr.attroff(curses.A_REVERSE)

                stdscr.addstr(h - 2, 4, "5:Select | 4/.:Back", curses.A_REVERSE)
                stdscr.refresh()
                need_redraw = False
            
            key = stdscr.getch()
            if key == -1:
                # Check if state changed (controller plugged/unplugged)
                new_controller = ControllerStore.find_controller_for_role(role)
                if (new_controller is None) != (controller is None):
                    need_redraw = True
                    sel = 0
                continue
            
            if key_match(key, "UP") or key_match(key, "DOWN"):
                if controller and is_deselected:
                    sel = 1 - sel  # Toggle between 0 and 1
                need_redraw = True
            elif key_match(key, "SELECT"):
                if controller:
                    if is_deselected:
                        if sel == 0:
                            # Re-select: remove deselected flag
                            assignments[role] = {"mac": controller["mac"], "path": controller["path"]}
                            ControllerStore.save_assignments(assignments)
                        else:
                            # Keep deselected (do nothing, just exit)
                            pass
                    else:
                        # Deselect: set flag
                        assignments[role] = {"deselected": True}
                        ControllerStore.save_assignments(assignments)
                else:
                    # No controller - if deselected, re-enable auto-assign
                    if is_deselected:
                        if role in assignments:
                            del assignments[role]
                        ControllerStore.save_assignments(assignments)
                break
            elif key_match(key, "REFRESH"):
                need_redraw = True
            elif key_match(key, "BACK") or key_match(key, "QUIT"):
                break
    
    def confirm_reset(self, stdscr):
        """Dedicated sub-menu for confirming configuration reset without flickering."""
        c_sel = 1 # Default to 'Cancel'
        options = ["CONFIRM RESET (Wipe All Data)", "CANCEL (Keep Settings)"]
        need_redraw = True
        stdscr.timeout(200)
        
        while True:
            if need_redraw:
                stdscr.erase()
                stdscr.addstr(4, 4, "⚠️  RESET CONFIGURATION?", curses.A_BOLD | curses.color_pair(1))
                stdscr.addstr(6, 4, "This will disconnect all controllers and stop ROS nodes.")
                
                for i, opt in enumerate(options):
                    if i == c_sel: stdscr.attron(curses.A_REVERSE)
                    stdscr.addstr(9 + i, 6, f" {opt} ")
                    stdscr.attroff(curses.A_REVERSE)
                
                stdscr.refresh()
                need_redraw = False
            
            key = stdscr.getch()
            if key == -1: continue
            
            if key_match(key, "UP") or key_match(key, "DOWN"):
                c_sel = 1 - c_sel # Toggle selection
                need_redraw = True
            elif key_match(key, "SELECT"):
                if c_sel == 0:
                    ControllerStore.save_assignments({})
                    self._show_message(stdscr, "Success", "Configuration has been wiped.")
                    return True
                return False
            elif key_match(key, "BACK") or key_match(key, "QUIT"):
                return False

    def confirm_exit(self, stdscr):
        """Dedicated sub-menu for confirming exit without flickering."""
        c_sel = 1 # Default to 'Cancel'
        options = ["CONFIRM EXIT (Shutdown)", "CANCEL (Stay in Menu)"]
        need_redraw = True
        stdscr.timeout(200)
        
        while True:
            if need_redraw:
                stdscr.erase()
                stdscr.addstr(4, 4, "⚠️  EXIT APPLICATION?", curses.A_BOLD | curses.color_pair(1))
                stdscr.addstr(6, 4, "This will stop all ROS nodes and exit the program.")
                
                for i, opt in enumerate(options):
                    if i == c_sel: stdscr.attron(curses.A_REVERSE)
                    stdscr.addstr(9 + i, 6, f" {opt} ")
                    stdscr.attroff(curses.A_REVERSE)
                
                stdscr.refresh()
                need_redraw = False
            
            key = stdscr.getch()
            if key == -1: continue
            
            if key_match(key, "UP") or key_match(key, "DOWN"):
                c_sel = 1 - c_sel # Toggle selection
                need_redraw = True
            elif key_match(key, "SELECT"):
                return c_sel == 0  # True if confirmed
            elif key_match(key, "BACK") or key_match(key, "QUIT"):
                return False
            
    def _handle_menu_selection(self, stdscr):
        if self.selection == 0: self.popup_select(stdscr, "DRIVE Controller Settings", "drive", self.arm)
        elif self.selection == 1: self.popup_select(stdscr, "ARM Controller Settings", "arm", self.drive)
        elif self.selection in (2, 3):
            from controller_store import ROLE_PATHS
            role = "drive" if self.selection == 2 else "arm"
            path = ROLE_PATHS.get(role)
            controller = getattr(self, role)

            if not controller:
                self._show_message(stdscr, "Error", f"No {role.upper()} controller connected.")
                return False

            start_fn = getattr(self.joy_manager, f"start_{role}")
            stop_fn = getattr(self.joy_manager, f"stop_{role}")
            is_running = getattr(self.joy_manager, f"{role}_process") is not None

            if not is_running:
                # START JOY with fixed path
                start_fn(path)
                # ros_manager.enable_drive() # Uncomment when ready
            else:
                # STOP JOY
                # ros_manager.disable_drive() # Uncomment when ready
                stop_fn()
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