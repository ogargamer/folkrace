import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """python3 - << 'EOF'
import websockets
import asyncio
import json

async def test():
    async with websockets.connect('ws://localhost:8000/ws') as ws:
        print('Connected to ws://localhost:8000/ws! Reading 3 frames:')
        for i in range(3):
            msg = await ws.recv()
            d = json.loads(msg)
            print(f"Frame {i+1}: Roll={d['roll']}°, Pitch={d['pitch']}°, Yaw={d['yaw']}°, Accel=({d['ax']}, {d['ay']}, {d['az']})")

asyncio.run(test())
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
print(stderr.read().decode())
ssh.close()
