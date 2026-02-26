# ROS Controller Configuration System


## System Overview

The system solves a common robotics problem: reliably mapping two physical controllers to two distinct robot roles (drive and arm) across reconnections, without them ever conflicting or swapping.

The solution uses **udev symlinks** (fixed device paths per role) instead of dynamic `/dev/input/js0`, `/dev/input/js1` enumeration. Each controller is plugged into a designated cable/port whose udev rule always maps it to the same path — `/dev/input/69` for DRIVE and `/dev/input/96` for ARM. The software then reads from these fixed paths and launches independent `joy_linux_node` processes per role.

```
Physical Controller (USB)
        │
        ▼
  udev symlink rule
        │
        ▼
 /dev/input/69 (DRIVE)       /dev/input/96 (ARM)
        │                           │
        ▼                           ▼
  joy_linux_node               joy_linux_node
  (drive_joy)                  (arm_joy)
        │                           │
        ▼                           ▼
  /joy/drive topic             /joy/arm topic
        │                           │
        ▼                           ▼
  Drive ROS2 Node              Arm ROS2 Node
```

---

## Architecture

```
ros_menu.py          ← Entry point (curses wrapper)
    └── ui.py        ← UI layer (curses rendering, input handling, menus)
         ├── controller_store.py  ← Hardware detection & assignment persistence
         ├── joy_manager.py       ← joy_linux_node subprocess lifecycle
         └── ros_manager.py       ← ROS2 context & drive/arm node processes
```

---

## Module Breakdown

### `ros_menu.py` — Entry Point

---

### `ui.py` — User Interface Layer (`ControllerUI`)

**Responsibilities:**
- Rendering the curses TUI (header, role status panels, menu)
- Reading keyboard input and dispatching to sub-menus
- Calling `ControllerStore` to refresh hardware state
- Calling `JoyManager` to start/stop joy nodes
- Calling `RosManager` to start/stop ROS2 nodes

**Main screens:**
- **Main menu** — shows DRIVE and ARM status, provides 7 menu actions
- **Controller settings popup** (`popup_select`) — per-role detail view with deselect/reselect capability
- **Reset confirmation** (`confirm_reset`) — two-option confirmation before wiping assignments
- **Exit confirmation** (`confirm_exit`) — confirmation before shutting down

**State managed:**
- `self.drive` / `self.arm` — currently assigned controller info dicts
- `self.selection` — currently highlighted menu item
- `self.controllers` — list of all detected controllers

**Key methods:**

| Method | Purpose |
|---|---|
| `run(stdscr)` | Main event loop |
| `draw(stdscr)` | Full screen redraw |
| `refresh_controllers()` | Re-scan hardware and update assignments |
| `popup_select(...)` | Controller settings sub-menu |
| `_handle_menu_selection(stdscr)` | Dispatches menu item actions |
| `_handle_navigation(...)` | Generic directional key handler |

---

### `controller_store.py` — Hardware Detection & Assignment Store (`ControllerStore`)

A stateless utility class (all static methods) responsible for reading the Linux input subsystem and persisting assignments.

**Key constants:**

| Constant | Purpose |
|---|---|
| `ROLE_PATHS` | Maps `"drive"` → `/dev/input/69`, `"arm"` → `/dev/input/96` |
| `CONTROLLER_OWNERS` | Maps Bluetooth MAC addresses to human-readable owner names |
| `CONTROLLER_PROFILES` | Maps controller model names to button/axis counts |
| `ASSIGNMENTS_FILE` | `/tmp/ros_controller_assignments.json` — runtime persistence |

**Controller info resolution (`get_controller_info`):**

Reads from the Linux `sysfs` filesystem for a given device path:

1. Resolves the symlink to real path (e.g. `/dev/input/69` → `/dev/input/js0`)
2. Reads `/sys/class/input/js0/device/name` → controller name
3. Reads `/sys/class/input/js0/device/uniq` → Bluetooth MAC (primary)
4. Falls back to `/sys/class/input/js0/device/phys` → USB physical path
5. Falls back to `vendor:product` ID
6. Looks up owner name from `CONTROLLER_OWNERS` dict

Returns a dict:
```python
{
    "path":      "/dev/input/69",    # symlink path (used by joy_node)
    "real_path": "/dev/input/js0",   # actual device
    "name":      "Xbox-360 Controller",
    "mac":       "50:ee:32:04:32:53",
    "owner":     "Chagol Chor",      # or None
}
```

**Assignment persistence:**

Assignments are saved as JSON to `/tmp/ros_controller_assignments.json`. Each role entry can be either:

```json
{
    "drive": { "mac": "50:ee:32:04:32:53", "path": "/dev/input/69" },
    "arm":   { "deselected": true }
}

```

### `joy_manager.py` — Joy Node Lifecycle (`JoyManager`)

Manages the lifecycle of `joy_linux_node` ROS2 processes — one for DRIVE, one for ARM.

**Process management:**

Each role gets its own subprocess via `subprocess.Popen` with `preexec_fn=os.setsid` (creates a new process group), allowing clean termination of the entire process group via `os.killpg`.

**Launched command (example for DRIVE):**
```bash
ros2 run joy_linux joy_linux_node \
    --ros-args \
    -r __node:=drive_joy \
    -p dev:=/dev/input/69 \
    -r joy:=/joy/drive
```

**Key methods:**

| Method | Purpose |
|---|---|
| `start_drive(device_path)` | Launches DRIVE joy_linux_node |
| `stop_drive()` | Terminates DRIVE joy_linux_node |
| `start_arm(device_path)` | Launches ARM joy_linux_node |
| `stop_arm()` | Terminates ARM joy_linux_node |
| `is_drive_running()` | Checks if DRIVE process is alive |
| `is_arm_running()` | Checks if ARM process is alive |
| `cleanup()` | Stops all processes on exit |

---

### `ros_manager.py` — ROS2 Context Manager (`RosManager`)

Manages the ROS2 Python context (`rclpy`) and higher-level ROS2 drive/arm node processes.
---

## ROS2 Nodes & Topics

### Nodes Created

| Node Name | ROS2 Package | Launched By | Purpose |
|---|---|---|---|
| `drive_joy` | `joy_linux` | `JoyManager.start_drive()` | Reads DRIVE controller input |
| `arm_joy` | `joy_linux` | `JoyManager.start_arm()` | Reads ARM controller input |
| `drive_node_executable` *(stub)* | `your_drive_package` | `RosManager.enable_drive()` | Processes drive commands |
| `arm_node_executable` *(stub)* | `your_arm_package` | `RosManager.enable_arm()` | Processes arm commands |

### Topics Published

| Topic | Type | Publisher Node | Subscriber |
|---|---|---|---|
| `/joy/drive` | `sensor_msgs/msg/Joy` | `drive_joy` | Drive control node |
| `/joy/arm` | `sensor_msgs/msg/Joy` | `arm_joy` | Arm control node |


## Controller Assignment Workflow

```
Application start
       │
       ▼
refresh_controllers() called
       │
       ├── Check /dev/input/69 (DRIVE path)
       │       │
       │       ├── Device exists?
       │       │       YES → get_controller_info() → read sysfs
       │       │               │
       │       │               ├── Check assignments.json for "deselected" flag
       │       │               │       YES → self.drive = None (user override)
       │       │               │       NO  → self.drive = controller info
       │       │               └── Save assignment to JSON
       │       │
       │       └── Device missing → self.drive = None, clear assignment
       │
       └── Same logic for /dev/input/96 (ARM path)
               └── self.arm = ...

User presses "Toggle DRIVE Joy Node"
       │
       ├── Is controller assigned? NO → show error message
       │
       └── YES → JoyManager.start_drive("/dev/input/69")
                       │
                       └── Popen: ros2 run joy_linux joy_linux_node
                               --ros-args -r __node:=drive_joy
                               -p dev:=/dev/input/69
                               -r joy:=/joy/drive
```

---

## Configuration Reference

### Registering a Controller Owner

In `controller_store.py`, add the Bluetooth MAC address and owner name:

```python
CONTROLLER_OWNERS = {
    "50:ee:32:04:32:53": "Chagol Chor",
    "84:30:95:41:0e:74": "Goru Chor",
    "aa:bb:cc:dd:ee:ff": "New Owner",   # ← add here
}
```

### Wiring in Your ROS2 Nodes

In `ros_manager.py`, replace the stub commands:

```python
def enable_drive(self):
    cmd = ["ros2", "run", "your_drive_package", "drive_node_executable"]
    # Replace with e.g.:
    cmd = ["ros2", "run", "my_robot_drive", "drive_controller"]
    self.drive_process = self._run_node(cmd)
```

---

## Key Bindings

| Key | Numpad Key | Action |
|---|---|---|
| ↑ Arrow | 8 | Navigate up |
| ↓ Arrow | 2 | Navigate down |
| ← Arrow | 4 | Navigate left / go back |
| → Arrow | 6 | Navigate right |
| Home | 7 | Jump to first item |
| End | 1 | Jump to last item |
| Page Up | 9 | Jump up 3 items |
| Page Down | 3 | Jump down 3 items |
| Enter | 5 | Select / confirm |
| Insert | 0 | Refresh controller list |
| Delete | . | Quit / go back |

---

## Running the Application

```bash
# Ensure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Run the menu
python3 ros_menu.py
```

**Dependencies:**
- Python 3.8+
- ROS2 (Humble or later)
- `joy_linux` ROS2 package: `sudo apt install ros-humble-joy-linux`
- `rclpy` Python bindings (included with ROS2)
- Standard Linux: `curses`, `subprocess`, `json`, `os`, `signal`

**Permissions:**

The application reads from `/dev/input/*` and `/sys/class/input/*`. If you encounter permission errors, add your user to the `input` group:

```bash
sudo usermod -aG input $USER
```

**Persistence file:**

Runtime assignments are stored in `/tmp/ros_controller_assignments.json` and are cleared on reboot. This is intentional — the application reconciles hardware state fresh on each start.
