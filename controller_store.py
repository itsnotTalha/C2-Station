import os
import glob
import json

# ================================================================================
# MODULAR BUTTON/AXIS MAPPING CONFIGURATION
# ================================================================================

CONTROLLER_PROFILES = {
    "Xbox-360 Controller": {"buttons": 11, "axes": 8},
    "PS4 Controller": {"buttons": 13, "axes": 8},
    "PS4 Wireless Controller": {"buttons": 16, "axes": 6},
    "Logitech X-3D Pro": {"buttons": 12, "axes": 6},
}

CONTROLLER_OWNERS = {
    "50:ee:32:04:32:53": "Chagol Chor",
    "84:30:95:41:0e:74": "Goru Chor",
}

ASSIGNMENTS_FILE = "/tmp/ros_controller_assignments.json"


class ControllerStore:
    @staticmethod
    def load_assignments():
        try:
            with open(ASSIGNMENTS_FILE) as f:
                return json.load(f)
        except Exception:
            return {}

    @staticmethod
    def save_assignments(assignments):
        try:
            with open(ASSIGNMENTS_FILE, "w") as f:
                json.dump(assignments, f)
        except Exception:
            pass

    @staticmethod
    def get_controller_mac(js_path):
        """Get MAC address of a controller via its input device."""
        try:
            js_name = os.path.basename(js_path)
            uniq_path = f"/sys/class/input/{js_name}/device/uniq"
            if os.path.exists(uniq_path):
                with open(uniq_path) as f:
                    uniq = f.read().strip()
                    if uniq and len(uniq) >= 12:
                        return uniq
            phys_path = f"/sys/class/input/{js_name}/device/phys"
            if os.path.exists(phys_path):
                with open(phys_path) as f:
                    phys = f.read().strip()
                    if phys:
                        return phys
            return js_path
        except:
            return js_path

    @staticmethod
    def find_joysticks():
        """Find all connected joystick devices by scanning /dev/input/js*"""
        devices = []
        for js in sorted(glob.glob("/dev/input/js*")):
            js_num = int(os.path.basename(js).replace('js', ''))
            name = "Unknown"
            try:
                with open(f"/sys/class/input/{os.path.basename(js)}/device/name") as f:
                    name = f.read().strip()
            except:
                pass
            mac = ControllerStore.get_controller_mac(js)
            owner = CONTROLLER_OWNERS.get(mac, None)
            devices.append({
                "device_id": js_num,
                "name": name,
                "path": js,
                "mac": mac,
                "owner": owner
            })
        return devices
