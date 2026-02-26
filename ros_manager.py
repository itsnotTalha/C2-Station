import subprocess
import signal
import os
import threading
from rclpy.node import Node
import rclpy

ROS2_AVAILABLE = True
try:
    import rclpy
except ImportError:
    ROS2_AVAILABLE = False

class RosManager:

    def __init__(self):
        self.ros_initialized = False
        self.available = ROS2_AVAILABLE
        self.drive_process = None
        self.arm_process = None

    def start(self):
        if not self.available: return False
        if not self.ros_initialized:
            rclpy.init()
            self.ros_initialized = True
        return True

    def _run_node(self, package, executable):
        """
        Runs a ROS2 node in the background.
        Sources the environment and suppresses all output.
        """
        bash_command = (
            f"source /opt/ros/humble/setup.bash && "
            f"source ~/ros2_ws/install/setup.bash && "
            f"ros2 run {package} {executable}"
        )

        return subprocess.Popen(
            bash_command,
            shell=True,
            executable='/bin/bash',
            stdout=subprocess.DEVNULL,  # Discards standard output
            stderr=subprocess.STDOUT,   # Redirects errors to DEVNULL via STDOUT
            preexec_fn=os.setsid        # Allows killing the whole process group later
        )

    def enable_drive(self):
        """Starts the Drive node as a background process."""
        if self.drive_process is None:
            # We pass the package and node name separately to our helper
            self.drive_process = self._run_node("rover_teleop", "rover_teleop_node")

    def enable_arm(self):
        """Starts the Arm node as a background process."""
        if self.arm_process is None:
            self.arm_process = self._run_node("rover_teleop", "rover_teleop_node")
            

    def disable_drive(self):
        """Kills the Drive node process group."""
        if self.drive_process:
            os.killpg(os.getpgid(self.drive_process.pid), signal.SIGTERM)
            self.drive_process.wait()
            self.drive_process = None

    def disable_arm(self):
        """Kills the Arm node process group."""
        if self.arm_process:
            os.killpg(os.getpgid(self.arm_process.pid), signal.SIGTERM)
            self.arm_process.wait()
            self.arm_process = None

    def stop(self):
        """Cleanup all processes on exit."""
        self.disable_drive()
        self.disable_arm()
        if self.ros_initialized:
            rclpy.shutdown()
            self.ros_initialized = False

    def publish_status(self, drive, arm):
        """No longer used as internal publishing logic is removed."""
        pass
