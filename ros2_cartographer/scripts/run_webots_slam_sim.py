import os
import sys
import time
import subprocess

webots_bin = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
world_path = os.path.abspath(r"my_project\worlds\Cyan_Champion_Track_Static.wbt")

env = os.environ.copy()
env["WEBOTS_RUN_SECONDS"] = "130.0"
env["ENABLE_MOVING_OBSTACLE"] = "0"
env["WEBOTS_HOME"] = r"C:\Program Files\Webots"

cmd = [
    webots_bin,
    "--mode=fast",
    "--batch",
    "--minimize",
    world_path
]

print(f"[WEBOTS RUNNER] Launching {world_path} in fast mode for 8.0s...")
process = subprocess.Popen(
    cmd,
    env=env,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    encoding="utf-8",
    errors="replace"
)

for line in process.stdout:
    print(line, end="", flush=True)

process.wait()
print(f"[WEBOTS RUNNER] Webots completed with return code: {process.returncode}")
