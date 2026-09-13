import os
import subprocess
import sys

def compile_dll():
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    source_file = os.path.join(curr_dir, "smart_hybrid_costmap_front.cpp")
    dll_output = os.path.join(curr_dir, "smart_hybrid_costmap_front.dll")

    # Locate g++ compiler (w64devkit or system PATH)
    webots_dir = os.path.abspath(os.path.join(curr_dir, *[".."] * 4))
    candidates = [
        os.path.join(webots_dir, "optimization", "ga_planner_optimizer", "w64devkit", "bin"),
        os.path.join(webots_dir, "optimization", "ga_optimizer", "w64devkit", "bin"),
        r"C:\Users\nurmi\Documents\Folkrace\ga_optimizer\w64devkit\bin",
        r"C:\Users\nurmi\Documents\Folkrace\tools\w64devkit\bin",
    ]
    compiler = "g++"
    env = os.environ.copy()
    for cand in candidates:
        gpp = os.path.join(cand, "g++.exe")
        if os.path.exists(gpp):
            env["PATH"] = cand + os.pathsep + env.get("PATH", "")
            compiler = gpp
            break

    cmd = [
        compiler,
        "-O3",
        "-shared",
        "-fPIC",
        "-mavx2",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-o", dll_output,
        source_file,
    ]

    print(f"[BUILD] Compiling C++ DLL with:\n{' '.join(cmd)}")
    result = subprocess.run(cmd, env=env, capture_output=True, text=True)

    if result.returncode != 0:
        print("[BUILD ERROR] Compilation failed:")
        print(result.stderr)
        sys.exit(1)
    else:
        print(f"[BUILD SUCCESS] Created {dll_output} ({os.path.getsize(dll_output)} bytes)")

if __name__ == "__main__":
    compile_dll()
