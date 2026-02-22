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
        
        # Process handles for simultaneous execution
        self.drive_process = None
        self.arm_process = None

    def start(self):
        """Initializes ROS2 context if not already started."""
        if not self.available:
            return False
        if not self.ros_initialized:
            rclpy.init()
            self.ros_initialized = True
        return True

    def _run_node(self, command):
        """Helper to run a command in a way that allows simultaneous execution."""
        # Use start_new_session to ensure the process group can be killed later
        return subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL, # Keep terminal clean
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid 
        )

    def enable_drive(self):
        """Starts the Drive node as a background process."""
        if self.drive_process is None:
            # Replace with your actual package and node names
            cmd = ["ros2", "run", "your_drive_package", "drive_node_executable"]
            self.drive_process = self._run_node(cmd)

    def disable_drive(self):
        """Kills the Drive node process group."""
        if self.drive_process:
            os.killpg(os.getpgid(self.drive_process.pid), signal.SIGTERM)
            self.drive_process.wait()
            self.drive_process = None

    def enable_arm(self):
        """Starts the Arm node as a background process."""
        if self.arm_process is None:
            # Replace with your actual package and node names
            cmd = ["ros2", "run", "your_arm_package", "arm_node_executable"]
            self.arm_process = self._run_node(cmd)

    def disable_arm(self):
        """Kills the Arm node process group."""
        if self.arm_process:
            os.killpg(os.getpgid(self.arm_process.pid), signal.SIGTERM)
            self.arm_process.wait()
            self.arm_process = None

    def get_topic_status(self, topic_name):
        """
        Checks if the node is effectively 'active' by looking at 
        running process handles.
        """
        if topic_name == "/buswala":
            return self.drive_process is not None and self.drive_process.poll() is None
        if topic_name == "/aram":
            return self.arm_process is not None and self.arm_process.poll() is None
        return False

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
