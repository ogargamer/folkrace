import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """python3 - << 'EOF'
import websockets
import asyncio
import json
import time

async def stream():
    async with websockets.connect('ws://localhost:8888/ws') as ws:
        print("Connected to WebSocket! Reading 10 continuous events...")
        for i in range(10):
            msg = await ws.recv()
            d = json.loads(msg)
            t = d.get('type')
            if t == 'map_b64':
                print(f"[{i+1}] MAP: {d['width']}x{d['height']} cells ({len(d['grid_b64'])} base64 chars)")
            elif t == 'pose':
                print(f"[{i+1}] POSE: X={d['x']:.2f}m, Y={d['y']:.2f}m, Yaw={d['yaw']*57.3:.1f}°")
            elif t == 'scan':
                print(f"[{i+1}] SCAN: {len(d['points'])} laser points")

asyncio.run(stream())
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
ssh.close()
