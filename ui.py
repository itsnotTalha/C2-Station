import curses
import time
import statistics
from controller_store import ControllerStore
from joy_manager import JoyManager
from ros_manager import RosManager


AUTO_REFRESH_DELAY = 1.0


HARDWARE_BUTTONS_AVAILABLE = False
chan = None
BUTTON_MAP = {
    "BLUE": 0,
    "RED": 3600,
    "YELLOW": 8400,
    "SOBUJ": 12900,
    "SHADA": 19000,
}

# ==============================
# HIGH PRECISION FAST BUTTON SYSTEM
# ==============================

PRESS_THRESHOLD = 700      # max distance allowed from center
RELEASE_THRESHOLD = 1200   # hysteresis release distance
SAMPLE_COUNT = 5           # fast averaging (low latency)

last_button = None
button_locked = False


def read_stable_adc():
    """Median over several samples to reject outliers."""
    samples = []
    for _ in range(7):
        samples.append(chan.value)
    return statistics.median(samples)


def get_button_from_value(val):
    """Return closest button if within threshold"""
    closest = None
    smallest_diff = float("inf")

    for name, center in BUTTON_MAP.items():
        diff = abs(val - center)
        if diff < smallest_diff:
            smallest_diff = diff
            closest = name

    if smallest_diff < PRESS_THRESHOLD:
        return closest
    return None


def read_button():
    """Ultra-fast, stable, hysteresis protected button read"""
    global last_button, button_locked

    if not HARDWARE_BUTTONS_AVAILABLE or chan is None:
        return None

    try:
        val = read_stable_adc()
        current = get_button_from_value(val)

        # --- Hysteresis Logic ---
        if button_locked:
            target_center = BUTTON_MAP.get(last_button)
            if target_center is None or abs(val - target_center) > RELEASE_THRESHOLD:
                button_locked = False
                last_button = None
            return None

        if current and current != last_button:
            last_button = current
            button_locked = True
            return current

        return None

    except Exception:
        return None

try:
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_ads1x15.analog_in import AnalogIn
    import board
    import busio
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS1115(i2c)
    ads.data_rate = 860  # Max speed
    chan = AnalogIn(ads, 0)
    HARDWARE_BUTTONS_AVAILABLE = True
except Exception:
    pass


def device_uid(dev):
    """Unique ID based on device_id."""
    if not dev:
        return None
    device_id = dev.get("device_id")
    if device_id is not None:
        return f"id:{device_id}"
    mac = dev.get("mac")
    if mac and mac != "N/A":
        return mac
    return dev.get("path")

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
            "Reboot",
            "EXIT TO TERMINAL"
        ]

    def draw(self, stdscr):
        # 1. Logic / Sync
        self.refresh_controllers()
        assignments = ControllerStore.load_assignments()
        d_data = assignments.get("drive", {})
        a_data = assignments.get("arm", {})
        d_ros_active = d_data.get("ros", False) if isinstance(d_data, dict) else False
        a_ros_active = a_data.get("ros", False) if isinstance(a_data, dict) else False

        # 2. Render
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        
        stdscr.addstr(1, 2, "ROS CONTROLLER CONFIGURATION", curses.A_BOLD | curses.color_pair(4))
        status_text = "[ROS2 OK]" if self.ros_manager.available else "[ROS2 N/A]"
        status_color = curses.color_pair(2) if self.ros_manager.available else curses.color_pair(1)
        stdscr.addstr(1, 35, status_text, status_color)

        # DRIVE Info
        y = 3
        stdscr.addstr(y, 2, "DRIVE", curses.A_BOLD | curses.color_pair(3))
        if self.drive:
            owner = self.drive.get("owner", "Unknown") or "Unknown"
            try:
                stdscr.addstr(y + 1, 4, f"[P] {owner}", curses.color_pair(2) | curses.A_BOLD)
                stdscr.addstr(y + 2, 4, f"MAC: {self.drive['mac'][:w-10]}", curses.color_pair(4))
            except curses.error: pass
        else:
            stdscr.addstr(y + 1, 4, "Not selected", curses.color_pair(1))
        
        d_ros = "* ACTIVE" if d_ros_active else "o INACTIVE"
        try: stdscr.addstr(y + 3, 4, f"ROS: {d_ros}", curses.color_pair(2 if d_ros_active else 1))
        except curses.error: pass

        # ARM Info
        y = 8
        stdscr.addstr(y, 2, "ARM", curses.A_BOLD | curses.color_pair(3))
        if self.arm:
            owner = self.arm.get("owner", "Unknown") or "Unknown"
            try:
                stdscr.addstr(y + 1, 4, f"[P] {owner}", curses.color_pair(2) | curses.A_BOLD)
                stdscr.addstr(y + 2, 4, f"MAC: {self.arm['mac'][:w-10]}", curses.color_pair(4))
            except curses.error: pass
        else:
            stdscr.addstr(y + 1, 4, "Not selected", curses.color_pair(1))

        a_ros = "* ACTIVE" if a_ros_active else "o INACTIVE"
        try: stdscr.addstr(y + 3, 4, f"ROS: {a_ros}", curses.color_pair(2 if a_ros_active else 1))
        except curses.error: pass

        # MENU
        y = 13
        display_menu = list(self.menu)
        display_menu[2] = f"{'Stop' if d_ros_active else 'Start'} DRIVE ROS Node"
        display_menu[3] = f"{'Stop' if a_ros_active else 'Start'} ARM ROS Node"

        for i, item in enumerate(display_menu):
            if i == self.selection: stdscr.attron(curses.A_REVERSE)
            if y + i < h - 1: stdscr.addstr(y + i, 2, f" {item} ")
            if i == self.selection: stdscr.attroff(curses.A_REVERSE)

        try: stdscr.addstr(h - 2, 0, "Navigate(↑↓/BLUE,3) Select(Ent/SHADA) Refresh(R/SOBUJ) Quit(Q/RED)".center(w)[:w-1], curses.A_REVERSE)
        except curses.error: pass
        stdscr.refresh()

    def refresh_controllers(self):
        """Load saved assignments - no scanning"""
        assignments = ControllerStore.load_assignments()

        # Load DRIVE from saved assignment
        drive_data = assignments.get("drive", {})
        if isinstance(drive_data, dict) and drive_data.get("device_id") is not None:
            self.drive = drive_data
        else:
            self.drive = None

        # Load ARM from saved assignment
        arm_data = assignments.get("arm", {})
        if isinstance(arm_data, dict) and arm_data.get("device_id") is not None:
            self.arm = arm_data
        else:
            self.arm = None

        # Enforce lifecycle
        if not (drive_data and drive_data.get("ros", False)):
            if self.joy_manager.is_drive_running():
                self.ros_manager.disable_drive()
                self.joy_manager.stop_drive()

        if not (arm_data and arm_data.get("ros", False)):
            if self.joy_manager.is_arm_running():
                self.ros_manager.disable_arm()
                self.joy_manager.stop_arm()

    def popup_select(self, stdscr, title, assign, role, exclude_controller=None):
        # Scan controllers once when popup opens
        self.controllers = ControllerStore.find_joysticks()
        sel = 0  # Will be corrected below after computing available_indices
        while True:
            assignments = ControllerStore.load_assignments()
            
            current_ctrl = getattr(self, role, None)
            current_uid = device_uid(current_ctrl)
            global_in_use_ids = set()
            for key, val in assignments.items():
                dev_id = None
                if isinstance(val, dict):
                    dev_id = val.get("device_id")
                if dev_id is not None and (current_ctrl is None or dev_id != current_ctrl.get("device_id")):
                    global_in_use_ids.add(dev_id)

            available_indices = []
            for i, d in enumerate(self.controllers):
                dev_id = d.get("device_id")
                if exclude_controller and dev_id == exclude_controller.get("device_id"):
                    continue
                if dev_id in global_in_use_ids:
                    continue
                available_indices.append(i)
            
            # Ensure sel is valid - set to first available if not in available list
            if sel not in available_indices and available_indices:
                sel = available_indices[0]
            
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(4))

            if not self.controllers:
                stdscr.addstr(4, 6, "No controllers found.", curses.color_pair(1))
            else:
                for i, d in enumerate(self.controllers):
                    dev_id = d.get("device_id")
                    mac = d.get("mac")
                    is_unavailable = (
                        (exclude_controller and dev_id == exclude_controller.get("device_id"))
                        or (dev_id in global_in_use_ids)
                    )
                    owner = d.get("owner")
                    line1 = f"[{dev_id}] {owner} ({d['name']})" if owner else f"[{dev_id}] {d['name']}"
                    if is_unavailable: line1 = f"[X] {line1} [IN USE]"
                    mac_display = mac[:30] if mac else "N/A"
                    line2 = f"  Path: {d['path']}  MAC: {mac_display}"
                    row = 4 + (i * 3)
                    
                    if is_unavailable:
                        stdscr.addstr(row, 6, line1[:w-8], curses.color_pair(1))
                        stdscr.addstr(row + 1, 6, line2[:w-8], curses.color_pair(1))
                    else:
                        if i == sel: stdscr.attron(curses.A_REVERSE)
                        stdscr.addstr(row, 6, line1[:w-8])
                        if i == sel: stdscr.attroff(curses.A_REVERSE)
                        stdscr.addstr(row + 1, 6, line2[:w-8], curses.color_pair(4))

            stdscr.addstr(h - 2, 4, "↑↓ Navigate  ENTER Select  R/SOBUJ Refresh  B/RED Back", curses.A_REVERSE)
            stdscr.refresh()

            key = stdscr.getch()
            btn = None
            if HARDWARE_BUTTONS_AVAILABLE: btn = read_button()
            if key == -1 and btn is None: continue

            if (key == curses.KEY_UP or btn == "BLUE") and available_indices:
                current_pos = available_indices.index(sel) if sel in available_indices else 0
                new_pos = (current_pos - 1) % len(available_indices)
                sel = available_indices[new_pos]
            elif (key == curses.KEY_DOWN or btn == "YELLOW") and available_indices:
                current_pos = available_indices.index(sel) if sel in available_indices else 0
                new_pos = (current_pos + 1) % len(available_indices)
                sel = available_indices[new_pos]
            elif (key in (10, 13) or btn == "SHADA") and available_indices and sel in available_indices:
                chosen = self.controllers[sel]
                assign(chosen)
                assignments = ControllerStore.load_assignments()
                assignments[role] = {
                    "device_id": chosen.get("device_id"),
                    "name": chosen.get("name"),
                    "mac": chosen.get("mac"),
                    "ros": False
                }
                ControllerStore.save_assignments(assignments)
                break
            elif key in (ord('r'), ord('R')) or btn == "SOBUJ":
                self.controllers = ControllerStore.find_joysticks()
                sel = 0  # Will be corrected to first available on next iteration
            elif key in (ord('b'), ord('B')) or btn == "RED":
                break

    def run(self, stdscr):
        curses.curs_set(0)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        stdscr.timeout(50 if HARDWARE_BUTTONS_AVAILABLE else 500)

        # Initialize ROS once at startup
        self.ros_manager.start()
        
        # Auto-refresh tracking
        last_refresh = time.time()

        try:
            while True:
                # Auto-refresh after delay
                if time.time() - last_refresh >= AUTO_REFRESH_DELAY:
                    self.refresh_controllers()
                    last_refresh = time.time()
                
                self.draw(stdscr)
                key = stdscr.getch()
                btn = None
                if HARDWARE_BUTTONS_AVAILABLE: btn = read_button()
                if key == -1 and btn is None: continue

                if key == curses.KEY_UP or btn == "BLUE":
                    self.selection = (self.selection - 1) % len(self.menu)
                elif key == curses.KEY_DOWN or btn == "YELLOW":
                    self.selection = (self.selection + 1) % len(self.menu)
                elif key in (10, 13) or btn == "SHADA":
                    if self.selection == 0:
                        self.popup_select(stdscr, "Select DRIVE Controller", lambda d: setattr(self, "drive", d), "drive", self.arm)
                    elif self.selection == 1:
                        self.popup_select(stdscr, "Select ARM Controller", lambda d: setattr(self, "arm", d), "arm", self.drive)
                    elif self.selection == 2:
                        # Toggle Drive (Safe, Universal)
                        assignments = ControllerStore.load_assignments()
                        
                        drive_data = assignments.get("drive", {})
                        if not isinstance(drive_data, dict):
                            drive_data = {}
                        
                        if drive_data.get("device_id") is None:
                            self._show_message(stdscr, "No Controller", "Select DRIVE first.")
                        else:
                            current_intent = drive_data.get("ros", False)
                            new_intent = not current_intent
                            
                            drive_data["ros"] = new_intent
                            assignments["drive"] = drive_data
                            ControllerStore.save_assignments(assignments)
                            
                            if new_intent:
                                drive_id = drive_data.get("device_id")
                                if self.joy_manager.start_drive(drive_id):
                                    self.ros_manager.enable_drive()
                                else:
                                    drive_data["ros"] = False
                                    assignments["drive"] = drive_data
                                    ControllerStore.save_assignments(assignments)
                                    self._show_message(stdscr, "Error", "Failed to start DRIVE joy_node.")
                            else:
                                self.ros_manager.disable_drive()
                                self.joy_manager.stop_drive()
                            self.refresh_controllers()

                    elif self.selection == 3:
                        # Toggle Arm (Safe, Universal)
                        assignments = ControllerStore.load_assignments()
                        
                        arm_data = assignments.get("arm", {})
                        if not isinstance(arm_data, dict):
                            arm_data = {}
                        
                        if arm_data.get("device_id") is None:
                            self._show_message(stdscr, "No Controller", "Select ARM first.")
                        else:
                            current_intent = arm_data.get("ros", False)
                            new_intent = not current_intent
                            
                            arm_data["ros"] = new_intent
                            assignments["arm"] = arm_data
                            ControllerStore.save_assignments(assignments)
                            
                            if new_intent:
                                arm_id = arm_data.get("device_id")
                                if self.joy_manager.start_arm(arm_id):
                                    self.ros_manager.enable_arm()
                                else:
                                    arm_data["ros"] = False
                                    assignments["arm"] = arm_data
                                    ControllerStore.save_assignments(assignments)
                                    self._show_message(stdscr, "Error", "Failed to start ARM joy_node.")
                            else:
                                self.ros_manager.disable_arm()
                                self.joy_manager.stop_arm()
                            self.refresh_controllers()
                    elif self.selection == 4:
                        stdscr.clear(); stdscr.refresh()
                    elif self.selection == 5:
                        if self._confirm_reset(stdscr):
                            self.joy_manager.cleanup()
                            self.ros_manager.disable_drive()
                            self.ros_manager.disable_arm()
                            ControllerStore.save_assignments({})
                            self.drive = None
                            self.arm = None
                            self.refresh_controllers()
                            self._show_message(stdscr, "Success", "Configuration has been reset.")
                    elif self.selection == 6:
                        self._confirm_reboot(stdscr)
                    elif self.selection == 7: 
                        if(self.terminate(stdscr)): break
                elif key in (ord('r'), ord('R')) or btn == "SOBUJ":
                    stdscr.clear()
                    self.refresh_controllers()
                elif key in (ord('q'), ord('Q')):
                    break
        finally:
            self.joy_manager.cleanup()
            self.ros_manager.stop()
            ControllerStore.save_assignments({}) # Cleanup on exit

    def _show_message(self, stdscr, title, message):
        stdscr.clear()
        stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(4, 4, message)
        stdscr.addstr(6, 4, "Press any key or SHADA to continue...")
        stdscr.refresh()
        # Use short timeout to poll both keyboard and hardware buttons
        stdscr.timeout(50)
        while True:
            key = stdscr.getch()
            btn = read_button() if HARDWARE_BUTTONS_AVAILABLE else None
            if key != -1 or btn:
                break
            time.sleep(0.05)
        # Restore normal timeout
        stdscr.timeout(50 if HARDWARE_BUTTONS_AVAILABLE else 500)

    def _confirm_reset(self, stdscr):
        """Confirmation dialog for Reset Configuration"""
        stdscr.clear()
        stdscr.addstr(2, 4, "RESET CONFIGURATION", curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(4, 4, "This will clear all controller assignments.")
        stdscr.addstr(5, 4, "Are you sure?")
        stdscr.addstr(7, 4, "[Y/SHADA] Confirm    [N/RED] Cancel", curses.color_pair(4))
        stdscr.refresh()
        # Use short timeout to poll both keyboard and hardware buttons
        stdscr.timeout(50)
        while True:
            key = stdscr.getch()
            btn = read_button() if HARDWARE_BUTTONS_AVAILABLE else None
            if key in (ord('y'), ord('Y')) or btn == "SHADA":
                # Restore timeout before returning
                stdscr.timeout(50 if HARDWARE_BUTTONS_AVAILABLE else 500)
                return True
            elif key in (ord('n'), ord('N'), 27) or btn == "RED" or key != -1:
                # ESC, N, RED, or any other key cancels
                stdscr.timeout(50 if HARDWARE_BUTTONS_AVAILABLE else 500)
                return False
            time.sleep(0.05)

    def _confirm_reboot(self, stdscr):
        """Confirmation dialog for System Reboot"""
        import os
        stdscr.clear()
        stdscr.addstr(2, 4, "REBOOT SYSTEM", curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(4, 4, "The system will reboot immediately.")
        stdscr.addstr(5, 4, "Are you sure?")
        stdscr.addstr(7, 4, "[Y/SHADA] Confirm    [N/RED] Cancel", curses.color_pair(4))
        stdscr.refresh()
        # Use short timeout to poll both keyboard and hardware buttons
        stdscr.timeout(50)
        while True:
            key = stdscr.getch()
            btn = read_button() if HARDWARE_BUTTONS_AVAILABLE else None
            if key in (ord('y'), ord('Y')) or btn == "SHADA":
                stdscr.clear()
                stdscr.addstr(4, 4, "Rebooting...", curses.A_BOLD | curses.color_pair(2))
                stdscr.refresh()
                self.joy_manager.cleanup()
                self.ros_manager.stop()
                ControllerStore.save_assignments({})
                time.sleep(0.5)
                os.system("sudo reboot")
                return
            elif key in (ord('n'), ord('N'), 27) or btn == "RED" or key != -1:
                # ESC, N, RED, or any other key cancels
                stdscr.timeout(50 if HARDWARE_BUTTONS_AVAILABLE else 500)
                return
    def terminate(self, stdscr):
        """Confirmation dialog for System Reboot"""
        import os
        stdscr.clear()
        stdscr.addstr(2, 4, "TERMINATE SYSTEM", curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(4, 4, "The Terminal will terminate immediately.")
        stdscr.addstr(5, 4, "Are you sure?")
        stdscr.addstr(7, 4, "[Y/SHADA] Confirm    [N/RED] Cancel", curses.color_pair(4))
        stdscr.refresh()
        # Use short timeout to poll both keyboard and hardware buttons
        stdscr.timeout(50)
        while True:
            key = stdscr.getch()
            btn = read_button() if HARDWARE_BUTTONS_AVAILABLE else None
            if key in (ord('y'), ord('Y')) or btn == "SHADA":
                stdscr.clear()
                stdscr.addstr(4, 4, "Terminating...", curses.A_BOLD | curses.color_pair(2))
                stdscr.refresh()
                self.joy_manager.cleanup()
                self.ros_manager.stop()
                ControllerStore.save_assignments({})
                time.sleep(0.5)
                return True
            elif key in (ord('n'), ord('N'), 27) or btn == "RED" or key != -1:
                # ESC, N, RED, or any other key cancels
                stdscr.timeout(50 if HARDWARE_BUTTONS_AVAILABLE else 500)
                return False