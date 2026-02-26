import subprocess
import signal
import os
import threading
import time

ROS2_AVAILABLE = False
Node = object  # Fallback base class

try:
    import rclpy
    from rclpy.node import Node as RclpyNode
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
    Node = RclpyNode
except ImportError:
    pass


class RosManager(Node):

    def __init__(self):
        self.ros_initialized = False
        self.available = ROS2_AVAILABLE
        self.drive_process = None
        self.arm_process = None

        # Telemetry Data
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.steering_mode = "Standby"
        
        # FIX: Default to Autonomous instead of Disconnected
        self.drive_mode = "Autonomous" 
        
        # Subscription Management
        self.is_subscribed = False
        self._node_initialized = False

    def start(self):
        if not self.available: return False
        if not self.ros_initialized:
            try:
                rclpy.init()
            except Exception:
                pass  # Already initialized
            
            # Initialize the Node parent class now that rclpy is initialized
            if not self._node_initialized:
                super().__init__('ui_telemetry_node')
                self._node_initialized = True
            
            self.ros_initialized = True

            # Subscribe immediately — indicator_topic is one-shot, can't afford to wait
            self.create_subscription(Twist, 'cmd_vel', self._twist_cb, 10)
            self.create_subscription(String, 'indicator', self._mode_cb, 10)
            self.is_subscribed = True

            # Start background spin
            self.spin_thread = threading.Thread(target=lambda: rclpy.spin(self), daemon=True)
            self.spin_thread.start()
        return True

    def _twist_cb(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        # angular.y = 200.0 → Spot Turn, 404.0 → Normal Mode
        self.steering_mode = "Spot Turn (360)" if msg.angular.y == 200.0 else "Differential Drive"

    def _mode_cb(self, msg):
        """
        Parses the indicator strings: 
        "Blue -> Manual Mode" or "RED -> Autonomous Mode"
        """
        data = msg.data
        if "Blue" in data:
            self.drive_mode = "Manual"
        elif "RED" in data:
            self.drive_mode = "Autonomous"

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

    # def stop(self):
    #     self.disable_drive()
    #     self.disable_arm()
    #     if self.ros_initialized:
    #         self.destroy_node()

    def publish_status(self, drive, arm):
        """No longer used as internal publishing logic is removed."""
        pass