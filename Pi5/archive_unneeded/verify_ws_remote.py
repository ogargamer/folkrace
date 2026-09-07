import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=5)

cmd = """python3 - << 'EOF'
import asyncio
import websockets
import json

async def test():
    async with websockets.connect("ws://localhost:8000/ws") as ws:
        print("Connected to WebSocket on localhost:8000/ws!")
        for i in range(5):
            msg = await ws.recv()
            d = json.loads(msg)
            print(f"  [{i+1}] Roll: {d['roll']:+6.2f}° | Pitch: {d['pitch']:+6.2f}° | Yaw: {d['yaw']:+6.2f}° | Accel: ({d['ax']:+5.2f}, {d['ay']:+5.2f}, {d['az']:+5.2f}) | Quat: ({d['qw']}, {d['qx']}, {d['qy']}, {d['qz']})")
        print(">> Live WebSocket Stream Verified 100%! <<")

asyncio.run(test())
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
print(stderr.read().decode())
ssh.close()
