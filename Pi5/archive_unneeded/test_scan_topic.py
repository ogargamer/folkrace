import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """
source /opt/ros/jazzy/setup.bash
python3 - << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanListener(Node):
    def __init__(self):
        super().__init__('test_scan_listener')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.count = 0
    def cb(self, msg):
        self.count += 1
        valid_ranges = [r for r in msg.ranges if 0.15 <= r <= 12.0]
        print(f"[{self.count}] LaserScan Received! Frame: {msg.header.frame_id}, Total Valid Points: {len(valid_ranges)}, Min: {min(valid_ranges) if valid_ranges else 0:.2f}m, Max: {max(valid_ranges) if valid_ranges else 0:.2f}m")
        if self.count >= 5:
            rclpy.shutdown()

rclpy.init()
node = ScanListener()
try:
    rclpy.spin(node)
except Exception:
    pass
EOF
"""

print("Starting scan listener on /scan...")
stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
for line in iter(stdout.readline, ""):
    print(line, end="")
ssh.close()
