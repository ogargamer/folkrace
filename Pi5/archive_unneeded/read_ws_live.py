import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=5)

cmd = """python3 - << 'EOF'
import asyncio
import websockets
import json

async def read_stream():
    async with websockets.connect("ws://localhost:8000/ws") as ws:
        print("Connected to BNO08X Mission Dashboard WebSocket!")
        print("Reading 20 stationary samples:")
        print("-" * 80)
        for i in range(20):
            msg = await ws.recv()
            d = json.loads(msg)
            print(f"[{i+1:02d}] ROLL:{d['roll']:+6.2f}° | PITCH:{d['pitch']:+6.2f}° | YAW:{d['yaw']:+6.2f}° | Quat:({d['qw']:+.3f},{d['qx']:+.3f},{d['qy']:+.3f},{d['qz']:+.3f}) | Accel:({d['ax']:+5.2f},{d['ay']:+5.2f},{d['az']:+5.2f}) | Gyro:({d['gx']:+4.1f},{d['gy']:+4.1f},{d['gz']:+4.1f})")

asyncio.run(read_stream())
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
for line in iter(stdout.readline, ""):
    print(line, end="")

ssh.close()
