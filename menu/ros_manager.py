import threading
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from controller_store import CONTROLLER_PROFILES

ROS2_AVAILABLE = True
try:
    import rclpy
except ImportError:
    ROS2_AVAILABLE = False

if ROS2_AVAILABLE:
    class ConditionalTeleopNode(Node):
        """
        Single ROS2 Node for both DRIVE and ARM control.
        """
        def __init__(self):
            super().__init__("conditional_teleop")
            self.drive_cmd_publisher = None
            self.arm_cmd_publisher = None

            self.drive_sub = None
            self.arm_sub = None

            self.drive_dummy_node = None
            self.safety_timer = None

            self.drive_mode = "Manual"
            self.arm_mode = "Manual"

            self._last_drive_model = None
            self._last_arm_model = None

            # Mode Publishers & Subscribers
            self.drive_mode_pub = self.create_publisher(String, "/drive/mode", 10)
            self.arm_mode_pub = self.create_publisher(String, "/arm/mode", 10)
            
            self.drive_mode_sub = self.create_subscription(String, "/drive/mode", self._drive_mode_callback, 10)
            self.arm_mode_sub = self.create_subscription(String, "/arm/mode", self._arm_mode_callback, 10)

            # Status Publisher
            self.status_publisher = self.create_publisher(String, "/controller_status", 10)

        def publish_status(self, drive_info, arm_info):
            if self.status_publisher:
                msg = String()
                drive_mode = self.drive_mode
                arm_mode = self.arm_mode
                
                data = {
                    "drive": drive_info,
                    "drive_mode": drive_mode,
                    "arm": arm_info,
                    "arm_mode": arm_mode
                }
                try:
                    msg.data = json.dumps(data)
                    self.status_publisher.publish(msg)
                except Exception:
                    pass

        def _drive_mode_callback(self, msg):
            self.drive_mode = msg.data

        def _arm_mode_callback(self, msg):
            self.arm_mode = msg.data

        def enable_drive(self):
            if self.drive_sub is None:
                self.drive_sub = self.create_subscription(Joy, "/joy/drive", self._drive_joy_callback, 10)
            if self.drive_cmd_publisher is None:
                self.drive_cmd_publisher = self.create_publisher(Twist, "/buswala", 10)
            self.drive_dummy_node = object()

        def disable_drive(self):
            if self.drive_sub is not None:
                self.destroy_subscription(self.drive_sub)
                self.drive_sub = None
            self.drive_dummy_node = None
            
            # Publish zero once then remove publisher
            if self.drive_cmd_publisher is not None:
                twist = Twist()
                self.drive_cmd_publisher.publish(twist)
                self.destroy_publisher(self.drive_cmd_publisher)
                self.drive_cmd_publisher = None

        def enable_arm(self):
            if self.arm_sub is None:
                self.arm_sub = self.create_subscription(Joy, "/joy/arm", self._arm_joy_callback, 10)
            if self.arm_cmd_publisher is None:
                self.arm_cmd_publisher = self.create_publisher(Twist, "/aram", 10)

        def disable_arm(self):
            if self.arm_sub is not None:
                self.destroy_subscription(self.arm_sub)
                self.arm_sub = None

            if self.arm_cmd_publisher is not None:
                twist = Twist()
                self.arm_cmd_publisher.publish(twist)
                self.destroy_publisher(self.arm_cmd_publisher)
                self.arm_cmd_publisher = None

        def _detect_controller_model(self, joy: Joy) -> str:
            buttons = len(joy.buttons)
            axes = len(joy.axes)
            for name, profile in CONTROLLER_PROFILES.items():
                if profile["buttons"] == buttons and profile["axes"] == axes:
                    return name
            return "Unknown"

        def _drive_joy_callback(self, joy: Joy):
            model = self._detect_controller_model(joy)
            self._last_drive_model = model

            # Mode switching logic
            target_mode = None
            if model == "Xbox-360 Controller":
                if len(joy.buttons) > 2 and joy.buttons[2]: target_mode = "Manual"
                elif len(joy.buttons) > 1 and joy.buttons[1]: target_mode = "Autonomous"
            elif model == "PS4 Controller":
                if len(joy.buttons) > 3 and joy.buttons[3]: target_mode = "Manual"
                elif len(joy.buttons) > 1 and joy.buttons[1]: target_mode = "Autonomous"
            elif model == "Logitech X-3D Pro":
                if len(joy.buttons) > 1 and joy.buttons[1]: target_mode = "Manual"
                elif len(joy.buttons) > 2 and joy.buttons[2]: target_mode = "Autonomous"
            
            if target_mode and target_mode != self.drive_mode:
                msg = String()
                msg.data = target_mode
                self.drive_mode_pub.publish(msg)
            
            if self.drive_mode != "Manual":
                return

            # Drive logic
            twist = Twist()
            if model == "Xbox-360 Controller":
                twist.linear.x = (joy.axes[1] / 2.0) + (joy.axes[4] / 2.0)
                twist.angular.z = (joy.axes[0] / 2.0) + (joy.axes[3] / 2.0)
            elif model == "PS4 Controller":
                twist.linear.x = (joy.axes[1] / 2.0) + (joy.axes[4] / 2.0)
                twist.angular.z = (joy.axes[0] / 2.0) + (joy.axes[3] / 2.0)
                if -0.17 < joy.axes[0] < 0.17: twist.angular.z -= joy.axes[0] / 2.0
                if -0.17 < joy.axes[3] < 0.17: twist.angular.z -= joy.axes[3] / 2.0
                if -0.17 < joy.axes[1] < 0.17: twist.linear.x -= joy.axes[1] / 2.0
                if -0.17 < joy.axes[4] < 0.17: twist.linear.x -= joy.axes[4] / 2.0
            elif model == "Logitech X-3D Pro":
                if len(joy.buttons) > 0 and joy.buttons[0]:
                    twist.linear.x = 0.0; twist.angular.z = 0.0
                elif (len(joy.axes) > 2 and (joy.axes[2] > 0.17 or joy.axes[2] < -0.17)) and not (len(joy.axes) > 1 and (joy.axes[1] > 0.1 or joy.axes[1] < -0.1)):
                    twist.linear.x = 0.0; twist.angular.z = joy.axes[2]
                else:
                    if len(joy.axes) > 1: twist.linear.x = joy.axes[1]
                    if len(joy.axes) > 0: twist.angular.z = joy.axes[0]
                    if -0.1 < twist.angular.z < 0.1: twist.angular.z = 0.0

            if self.drive_cmd_publisher is not None:
                self.drive_cmd_publisher.publish(twist)

        def _arm_joy_callback(self, joy: Joy):
            model = self._detect_controller_model(joy)
            self._last_arm_model = model

            # Mode switching logic (same as drive for now?)
            target_mode = None
            if model == "Xbox-360 Controller":
                if len(joy.buttons) > 2 and joy.buttons[2]: target_mode = "Manual"
                elif len(joy.buttons) > 1 and joy.buttons[1]: target_mode = "Autonomous"
            elif model == "PS4 Controller":
                if len(joy.buttons) > 3 and joy.buttons[3]: target_mode = "Manual"
                elif len(joy.buttons) > 1 and joy.buttons[1]: target_mode = "Autonomous"
            elif model == "Logitech X-3D Pro":
                if len(joy.buttons) > 1 and joy.buttons[1]: target_mode = "Manual"
                elif len(joy.buttons) > 2 and joy.buttons[2]: target_mode = "Autonomous"
            
            if target_mode and target_mode != self.arm_mode:
                msg = String()
                msg.data = target_mode
                self.arm_mode_pub.publish(msg)
            
            if self.arm_mode != "Manual":
                return

            twist = Twist()
            if model in ("Xbox-360 Controller", "PS4 Controller"):
                if len(joy.axes) >= 2:
                    twist.linear.x = joy.axes[1]
                    twist.angular.z = joy.axes[0]
            elif model == "Logitech X-3D Pro":
                if len(joy.axes) >= 2:
                    twist.linear.x = joy.axes[1]
                    twist.angular.z = joy.axes[0]

            if self.arm_cmd_publisher is not None:
                self.arm_cmd_publisher.publish(twist)

        def cleanup(self):
            self.disable_drive()
            self.disable_arm()
            if self.safety_timer:
                self.safety_timer.cancel()
                self.destroy_timer(self.safety_timer)


class RosManager:
    def __init__(self):
        self.ros_initialized = False
        self.teleop_node = None
        self.ros_spin_thread = None
        self.available = ROS2_AVAILABLE

    def start(self):
        if not self.available:
            return False
        if not self.ros_initialized:
            rclpy.init()
            self.ros_initialized = True
        
        if self.teleop_node is None:
            self.teleop_node = ConditionalTeleopNode()
            self._start_ros_spin_thread()
        return True

    def stop(self):
        if self.ros_initialized:
            try:
                if self.teleop_node:
                    self.teleop_node.cleanup()
                    self.teleop_node.destroy_node()
                rclpy.shutdown()
            except:
                pass
            self.ros_initialized = False
            self.teleop_node = None

    def _start_ros_spin_thread(self):
        if self.ros_spin_thread is None or not self.ros_spin_thread.is_alive():
            self.ros_spin_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
            self.ros_spin_thread.start()

    def _ros_spin_loop(self):
        while self.ros_initialized:
            try:
                if self.teleop_node:
                    rclpy.spin_once(self.teleop_node, timeout_sec=0.01)
                time.sleep(0.01)
            except:
                break

    def enable_drive(self):
        if self.teleop_node:
            self.teleop_node.enable_drive()

    def disable_drive(self):
        if self.teleop_node:
            self.teleop_node.disable_drive()

    def enable_arm(self):
        if self.teleop_node:
            self.teleop_node.enable_arm()

    def disable_arm(self):
        if self.teleop_node:
            self.teleop_node.disable_arm()

    def get_topic_status(self, topic_name):
        """Safe non-throwing topic check"""
        if not self.available or not self.ros_initialized or not self.teleop_node:
            return False
        try:
            return self.teleop_node.count_publishers(topic_name) > 0
        except Exception:
            return False

    def publish_status(self, drive, arm):
        if self.teleop_node:
            self.teleop_node.publish_status(drive, arm)

    def get_drive_mode(self):
        return self.teleop_node.drive_mode if self.teleop_node else "Unknown"

    def get_arm_mode(self):
        return self.teleop_node.arm_mode if self.teleop_node else "Unknown"
