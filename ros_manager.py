import subprocess
import os

# ============================================================
# SIMPLE ROS MANAGER - Just runs ros2 commands via subprocess
# ============================================================

# CHANGE THESE TO YOUR ACTUAL PACKAGE AND NODE NAMES
DRIVE_PACKAGE = "your_drive_pkg"      # e.g., "my_teleop"
DRIVE_NODE = "your_drive_node"        # e.g., "teleop_node"

ARM_PACKAGE = "your_arm_pkg"          # e.g., "my_arm_control"
ARM_NODE = "your_arm_node"            # e.g., "arm_node"

# Path to your ROS2 workspace setup file
ROS2_WS_SETUP = os.path.expanduser("~/ros2_ws/install/setup.bash")


class RosManager:
    def __init__(self):
        self.available = os.path.exists(ROS2_WS_SETUP)
        self.drive_process = None
        self.arm_process = None
        self.drive_active = False
        self.arm_active = False

    def start(self):
        """Called once at startup - nothing to do"""
        return self.available

    def stop(self):
        """Cleanup on exit"""
        self.disable_drive()
        self.disable_arm()

    def _run_ros2_cmd(self, package, node):
        """Run ros2 run <package> <node> in background"""
        cmd = f"source {ROS2_WS_SETUP} && ros2 run {package} {node}"
        proc = subprocess.Popen(
            cmd,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setpgrp
        )
        return proc

    def _kill_process(self, proc):
        """Kill a subprocess and its children"""
        if proc is None:
            return
        try:
            import signal
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    # ==================== DRIVE ====================

    def enable_drive(self):
        """Start the drive ROS2 node"""
        if self.drive_process is None:
            self.drive_process = self._run_ros2_cmd(DRIVE_PACKAGE, DRIVE_NODE)
            self.drive_active = True

    def disable_drive(self):
        """Stop the drive ROS2 node"""
        self._kill_process(self.drive_process)
        self.drive_process = None
        self.drive_active = False

    # ==================== ARM ====================

    def enable_arm(self):
        """Start the arm ROS2 node"""
        if self.arm_process is None:
            self.arm_process = self._run_ros2_cmd(ARM_PACKAGE, ARM_NODE)
            self.arm_active = True

    def disable_arm(self):
        """Stop the arm ROS2 node"""
        self._kill_process(self.arm_process)
        self.arm_process = None
        self.arm_active = False

    # ==================== STATUS ====================

    def get_topic_status(self, topic_name):
        """Check if drive/arm is active based on our process"""
        if topic_name == "/buswala":
            return self.drive_active
        elif topic_name == "/aram":
            return self.drive_active
        return False

    def publish_status(self, drive, arm):
        """No-op - not needed for simple mode"""
        pass

    def get_drive_mode(self):
        return "Manual"

    def get_arm_mode(self):
        return "Manual"

    def get_arm_state(self):
        return "Manual"