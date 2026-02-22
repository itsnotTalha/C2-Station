import os
import signal
import subprocess
import time


class JoyManager:
    def __init__(self):
        self.drive_process = None
        self.arm_process = None
        self.drive_device_path = None
        self.arm_device_path = None

    def launch_joy_node(self, device_path, topic_remap, node_name):
        """
        Launch joy_linux_node with a direct device path.
        Uses joy_linux instead of joy because it supports /dev/input/js* paths
        directly via the 'dev' parameter — fully compatible with udev symlinks.
        Args:
            device_path: full path e.g. '/dev/input/69'
            topic_remap: e.g. '/joy/drive' or '/joy/arm'
            node_name:   unique ROS node name
        Returns:
            subprocess.Popen handle or None
        """
        try:
            cmd = [
                'ros2', 'run', 'joy_linux', 'joy_linux_node',
                '--ros-args',
                '-r', f'__node:={node_name}',
                '-p', f'dev:={device_path}',
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
        """Terminate a joy_linux_node subprocess cleanly."""
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

    # ================= DRIVE =================

    def start_drive(self, device_path):
        """Start joy_linux_node for DRIVE. Accepts path like '/dev/input/69'."""
        if device_path is None:
            return False

        # Avoid same device for both roles
        if self.arm_device_path is not None and device_path == self.arm_device_path:
            return False

        if self.drive_process:
            self.stop_drive()

        self.drive_process = self.launch_joy_node(device_path, '/joy/drive', 'drive_joy')
        if self.drive_process:
            self.drive_device_path = device_path
            time.sleep(0.3)
            return True

        self.drive_device_path = None
        return False

    def stop_drive(self):
        if self.drive_process:
            self.terminate_joy_node(self.drive_process)
            self.drive_process = None
        self.drive_device_path = None

    # ================= ARM =================

    def start_arm(self, device_path):
        """Start joy_linux_node for ARM. Accepts path like '/dev/input/96'."""
        if device_path is None:
            return False

        # Avoid same device for both roles
        if self.drive_device_path is not None and device_path == self.drive_device_path:
            return False

        if self.arm_process:
            self.stop_arm()

        self.arm_process = self.launch_joy_node(device_path, '/joy/arm', 'arm_joy')
        if self.arm_process:
            self.arm_device_path = device_path
            time.sleep(0.3)
            return True

        self.arm_device_path = None
        return False

    def stop_arm(self):
        if self.arm_process:
            self.terminate_joy_node(self.arm_process)
            self.arm_process = None
        self.arm_device_path = None

    # ================= STATUS =================

    def is_drive_running(self):
        return self.drive_process is not None and self.drive_process.poll() is None

    def is_arm_running(self):
        return self.arm_process is not None and self.arm_process.poll() is None

    # ================= CLEANUP =================

    def cleanup(self):
        self.stop_drive()
        self.stop_arm()