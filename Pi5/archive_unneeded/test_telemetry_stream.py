import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """python3 - << 'EOF'
import websockets
import asyncio
import json

async def test():
    async with websockets.connect('ws://localhost:8888/ws') as ws:
        print("Connected to WebSocket! Waiting for Telemetry event...")
        for _ in range(15):
            msg = await ws.recv()
            data = json.loads(msg)
            if data.get('type') == 'telemetry':
                print(">> TELEMETRY RECEIVED:")
                print("  LiDAR:", data['lidar'])
                print("  IMU:  ", data['imu'])
                break

asyncio.run(test())
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
ssh.close()
