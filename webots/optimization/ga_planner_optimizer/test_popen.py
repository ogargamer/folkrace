import os
import sys
import subprocess
import socket
import time

webots_bin = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
world_path = r"C:\Users\nurmi\Documents\Da Winna\webots/optimization/ga_planner_optimizer\worlds\Cyan_Champion_GA.wbt"
env = os.environ.copy()
env["GA_WORKER_PORT"] = "10200"

cmd = [webots_bin, "--port=10200", "--mode=fast", "--no-rendering", "--batch", "--minimize", "--stdout", "--stderr", world_path]

print("Launching Webots:", cmd)
proc = subprocess.Popen(cmd, env=env)

print("Waiting for port 10200 to accept connections...", flush=True)
t0 = time.time()
connected = False
while time.time() - t0 < 30.0:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect(("127.0.0.1", 10200))
        req = '{"cmd": "PING"}\n'
        s.sendall(req.encode("utf-8"))
        data = s.recv(1024)
        s.close()
        print(f"Connected! Response: {data.decode('utf-8').strip()}")
        connected = True
        break
    except Exception as e:
        time.sleep(0.5)

if connected:
    print("Socket ping test SUCCEEDED!")
    # Test a 3.0 second evaluation
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(30.0)
    s.connect(("127.0.0.1", 10200))
    eval_req = '{"cmd": "EVAL", "planner": "OPTION_1_BVP", "genes": {}, "duration_s": 3.0}\n'
    print("Sending 3.0s eval request...", flush=True)
    t_req = time.time()
    s.sendall(eval_req.encode("utf-8"))
    res_data = s.recv(4096)
    s.close()
    print(f"Eval completed in {time.time() - t_req:.2f}s! Response: {res_data.decode('utf-8').strip()}")
else:
    print("Socket ping test FAILED!")

try:
    proc.terminate()
except Exception:
    pass
