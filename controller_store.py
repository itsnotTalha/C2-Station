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

# Role-specific paths (udev symlinks)
ROLE_PATHS = {
    "drive": "/dev/input/69",
    "arm": "/dev/input/96",
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
    def get_controller_info(symlink_path):
        """
        Get real controller info (MAC, name) from a symlink path like /dev/input/69.
        Resolves the symlink to find the real js* device and reads sysfs.
        """
        try:
            # Resolve symlink to real path (e.g., /dev/input/69 -> /dev/input/js0)
            if os.path.islink(symlink_path):
                real_path = os.path.realpath(symlink_path)
            else:
                real_path = symlink_path
            
            js_name = os.path.basename(real_path)  # e.g., "js0"
            
            # Get controller name from sysfs
            name = "Unknown Controller"
            name_path = f"/sys/class/input/{js_name}/device/name"
            if os.path.exists(name_path):
                with open(name_path) as f:
                    name = f.read().strip()
            
            # Get MAC address - try multiple sources
            mac = None
            
            # 1. Try uniq (Bluetooth MAC)
            uniq_path = f"/sys/class/input/{js_name}/device/uniq"
            if os.path.exists(uniq_path):
                with open(uniq_path) as f:
                    uniq = f.read().strip()
                    if uniq:
                        mac = uniq
            
            # 2. Try phys (USB path - use as fallback identifier)
            if not mac:
                phys_path = f"/sys/class/input/{js_name}/device/phys"
                if os.path.exists(phys_path):
                    with open(phys_path) as f:
                        phys = f.read().strip()
                        if phys:
                            mac = phys
            
            # 3. Fallback: vendor:product
            if not mac:
                vendor_path = f"/sys/class/input/{js_name}/device/id/vendor"
                product_path = f"/sys/class/input/{js_name}/device/id/product"
                if os.path.exists(vendor_path) and os.path.exists(product_path):
                    with open(vendor_path) as f:
                        vendor = f.read().strip()
                    with open(product_path) as f:
                        product = f.read().strip()
                    mac = f"{vendor}:{product}"
            
            if not mac:
                mac = f"UNKNOWN@{symlink_path}"
            
            # Look up owner from MAC
            owner = CONTROLLER_OWNERS.get(mac, None)
            
            return {
                "path": symlink_path,  # Keep the symlink path for joy_node
                "real_path": real_path,
                "name": name,
                "mac": mac,
                "owner": owner,
            }
        except Exception as e:
            return None

    @staticmethod
    def find_controller_for_role(role):
        """
        Scan the specific path for a role (drive or arm).
        Returns controller info if found, None otherwise.
        """
        path = ROLE_PATHS.get(role)
        if not path:
            return None
        
        if os.path.exists(path):
            return ControllerStore.get_controller_info(path)
        return None

    @staticmethod
    def find_joysticks():
        """
        Scans both role-specific paths and returns all connected controllers.
        """
        controllers = []
        for role, path in ROLE_PATHS.items():
            if os.path.exists(path):
                info = ControllerStore.get_controller_info(path)
                if info:
                    info["role_path"] = role  # Tag which role this path is for
                    controllers.append(info)
        return controllers