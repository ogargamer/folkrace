import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """python3 - << 'EOF'
import websockets
import asyncio
import time

async def test():
    async with websockets.connect('ws://localhost:8000/ws') as ws:
        print("Connected! Measuring 10 packet arrival intervals:")
        last_t = time.time()
        for i in range(10):
            msg = await ws.recv()
            now = time.time()
            dt_ms = (now - last_t) * 1000.0
            last_t = now
            if i > 0:
                print(f"Packet {i:02d}: Interval = {dt_ms:5.1f} ms  (Rate = {1000.0/dt_ms:5.1f} Hz)")

asyncio.run(test())
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
print(stderr.read().decode())
ssh.close()
