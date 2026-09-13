import os
import subprocess
import sys

def compile_dll():
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    source_file = os.path.join(curr_dir, "curvilinear_sl_mpc.cpp")
    dll_output = os.path.join(curr_dir, "curvilinear_sl_mpc.dll")
    osqp_dir = os.path.join(curr_dir, "osqp_codegen_sl_mpc")
    osqp_lib = os.path.join(osqp_dir, "libosqp.a")

    # Locate w64devkit bin directory
    webots_dir = os.path.abspath(os.path.join(curr_dir, *[".."] * 3))
    w64devkit_bin = os.path.join(webots_dir, "optimization", "ga_optimizer", "w64devkit", "bin")
    w64devkit_gpp = os.path.join(w64devkit_bin, "g++.exe")

    env = os.environ.copy()
    if os.path.exists(w64devkit_bin):
        env["PATH"] = w64devkit_bin + os.pathsep + env.get("PATH", "")
        compiler = w64devkit_gpp
    else:
        compiler = "g++"

    inc_dirs = [
        f"-I{curr_dir}",
        f"-I{osqp_dir}",
        f"-I{os.path.join(osqp_dir, 'inc', 'public')}",
        f"-I{os.path.join(osqp_dir, 'inc', 'private')}",
    ]

    # Windows lock handling: atomically replace existing DLL backup so compiler can output to fresh file
    old_dll = dll_output + ".old"
    if os.path.exists(dll_output):
        try:
            os.replace(dll_output, old_dll)
        except Exception as e:
            print(f"[Notice] Could not replace existing DLL: {e}")

    cmd = [
        compiler,
        "-O3",
        "-shared",
        "-fPIC",
        "-mavx2",
        "-std=c++17",
        "-Wall",
        "-Wextra",
    ] + inc_dirs + [
        "-o", dll_output,
        source_file,
        osqp_lib,
    ]

    print(f"[BUILD] Compiling C++ Curvilinear SL-MPC DLL with:\n{' '.join(cmd)}")
    result = subprocess.run(cmd, env=env, capture_output=True, text=True)

    if result.returncode != 0:
        print("[BUILD ERROR] Compilation failed:")
        print(result.stderr)
        sys.exit(1)
    else:
        print(f"[BUILD SUCCESS] Created {dll_output} ({os.path.getsize(dll_output)} bytes)")

if __name__ == "__main__":
    compile_dll()
