import os
import signal
import subprocess
import time


class JoyManager:
    def __init__(self):
        self.drive_process = None
        self.arm_process = None
        self.drive_device_id = None
        self.arm_device_id = None

    def launch_joy_node(self, device_id, topic_remap, node_name):
        """
        Launch joy_node with device_id.
        Args:
            device_id: integer (0, 1, 2...)
            topic_remap: e.g., '/joy/drive' or '/joy/arm'
            node_name: unique ROS node name
        Returns:
            subprocess.Popen handle or None
        """
        try:
            cmd = [
                'ros2', 'run', 'joy', 'joy_node',
                '--ros-args',
                '-r', f'__node:={node_name}',
                '-p', f'device_id:={device_id}',
                '-r', f'joy:={topic_remap}'
            ]
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            return process
        except Exception:
            return None

    def terminate_joy_node(self, process):
        """Terminate a joy_node subprocess cleanly"""
        if process is None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=1.0)
            except Exception:
                pass

    def start_drive(self, device_id):
        """Start joy_node for DRIVE with given device_id."""
        if device_id is None:
            return False
        # Avoid same device for both roles
        if self.arm_device_id is not None and device_id == self.arm_device_id:
            return False
        if self.drive_process:
            self.stop_drive()
        self.drive_process = self.launch_joy_node(device_id, '/joy/drive', 'drive_joy')
        if self.drive_process:
            self.drive_device_id = device_id
            time.sleep(0.3)
            return True
        self.drive_device_id = None
        return False

    def stop_drive(self):
        if self.drive_process:
            self.terminate_joy_node(self.drive_process)
            self.drive_process = None
        self.drive_device_id = None

    def start_arm(self, device_id):
        """Start joy_node for ARM with given device_id."""
        if device_id is None:
            return False
        # Avoid same device for both roles
        if self.drive_device_id is not None and device_id == self.drive_device_id:
            return False
        if self.arm_process:
            self.stop_arm()
        self.arm_process = self.launch_joy_node(device_id, '/joy/arm', 'arm_joy')
        if self.arm_process:
            self.arm_device_id = device_id
            time.sleep(0.3)
            return True
        self.arm_device_id = None
        return False

    def stop_arm(self):
        if self.arm_process:
            self.terminate_joy_node(self.arm_process)
            self.arm_process = None
        self.arm_device_id = None

    def is_drive_running(self):
        if self.drive_process is None:
            return False
        return self.drive_process.poll() is None

    def is_arm_running(self):
        if self.arm_process is None:
            return False
        return self.arm_process.poll() is None

    def cleanup(self):
        self.stop_drive()
        self.stop_arm()
