# ROS Controller Menu

Simple TUI menu to manage ROS2 controllers.

## Setup Before Running

Edit `ros_manager.py` and change these variables:

```python
# Your ROS2 package and node names
DRIVE_PACKAGE = "your_drive_pkg"    # Change to your drive package name
DRIVE_NODE = "your_drive_node"      # Change to your drive node name

ARM_PACKAGE = "your_arm_pkg"        # Change to your arm package name
ARM_NODE = "your_arm_node"          # Change to your arm node name

# Path to your ROS2 workspace (change if different)
ROS2_WS_SETUP = os.path.expanduser("~/ros2_ws/install/setup.bash")
```

## Run

```bash
python3 ros_menu.py
```

## Controls

| Key | Action |
|-----|--------|
| ↑/↓ | Navigate menu |
| Enter | Select |
| R | Refresh |
| Q | Quit |

## What It Does

1. Select a controller for DRIVE
2. Select a controller for ARM
3. Toggle ROS → starts joy_node, then runs `ros2 run <package> <node>`
