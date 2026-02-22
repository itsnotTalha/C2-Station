import os
import signal
import subprocess
import time

class JoyManager:
    def __init__(self):
        self.drive_process = None
        self.arm_process = None

    def launch_joy_node(self, device_path, topic_remap):
        """
        Launch joy_node as subprocess.
        Args:
            device_path: /dev/input/jsX
            topic_remap: e.g., '/joy/drive' or '/joy/arm'
        Returns:
            subprocess.Popen handle or None
        """
        try:
            cmd = [
                'ros2', 'run', 'joy', 'joy_node',
                '--ros-args',
                '-p', f'dev:={device_path}',
                '-r', f'joy:={topic_remap}'
            ]
            # Launch as background process, suppress output
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group for clean termination
            )
            return process
        except Exception:
            return None

    def terminate_joy_node(self, process):
        """Terminate a joy_node subprocess cleanly"""
        if process is None:
            return
        try:
            # Send SIGTERM to the process group
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=2.0)
        except Exception:
            try:
                # Force kill if graceful termination fails
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=1.0)
            except Exception:
                pass

    def start_drive(self, device_path):
        if self.drive_process:
            self.stop_drive()
        self.drive_process = self.launch_joy_node(device_path, '/joy/drive')
        return self.drive_process is not None

    def stop_drive(self):
        if self.drive_process:
            self.terminate_joy_node(self.drive_process)
            self.drive_process = None

    def start_arm(self, device_path):
        if self.arm_process:
            self.stop_arm()
        self.arm_process = self.launch_joy_node(device_path, '/joy/arm')
        return self.arm_process is not None

    def stop_arm(self):
        if self.arm_process:
            self.terminate_joy_node(self.arm_process)
            self.arm_process = None
    
    def cleanup(self):
        self.stop_drive()
        self.stop_arm()
