import os
import glob
import json

# ================================================================================
# MODULAR BUTTON/AXIS MAPPING CONFIGURATION
# ================================================================================

CONTROLLER_PROFILES = {
    "Xbox-360 Controller": {"buttons": 11, "axes": 8},
    "PS4 Controller": {"buttons": 13, "axes": 8},
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
        """
        Get MAC address of a wired controller via its input device.
        """
        try:
            js_name = os.path.basename(js_path)
            
            # Try physical path
            phys_path = f"/sys/class/input/{js_name}/device/phys"
            if os.path.exists(phys_path):
                with open(phys_path) as f:
                    phys = f.read().strip()
                    if phys:
                        return phys
            
            # Try unique ID
            uniq_path = f"/sys/class/input/{js_name}/device/uniq"
            if os.path.exists(uniq_path):
                with open(uniq_path) as f:
                    uniq = f.read().strip()
                    if uniq:
                        return uniq
            
            # Fallback: vendor:product
            vendor_path = f"/sys/class/input/{js_name}/device/id/vendor"
            product_path = f"/sys/class/input/{js_name}/device/id/product"
            if os.path.exists(vendor_path) and os.path.exists(product_path):
                with open(vendor_path) as f:
                    vendor = f.read().strip()
                with open(product_path) as f:
                    product = f.read().strip()
                return f"{vendor}:{product}@{js_path}"
            
            return "N/A"
        except:
            return "N/A"

    @staticmethod
    def find_joysticks():
        """Find all connected joystick devices with MAC addresses"""
        devices = []
        for js in sorted(glob.glob("/dev/input/js*")):
            name = "Unknown"
            try:
                with open(f"/sys/class/input/{os.path.basename(js)}/device/name") as f:
                    name = f.read().strip()
            except:
                pass
            
            mac = ControllerStore.get_controller_mac(js)
            owner = CONTROLLER_OWNERS.get(mac, None)
            devices.append({"path": js, "name": name, "mac": mac, "owner": owner})
        return devices
