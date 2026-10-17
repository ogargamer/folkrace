"""
================================================================================
  UNIFIED C++ DLL BUILD TOOL FOR AUTONOMOUS TOF CONTROLLER
  Compiles:
    1. smart_hybrid_costmap_front.dll (AVX2 Fast Circle Triangulation Costmap)
    2. curvilinear_sl_mpc.dll (AVX2 OSQP Curvilinear Slack-Constrained SL-MPC)
================================================================================
"""

import os
import shutil
import subprocess
import sys
import time
from typing import Optional, Tuple


def find_compiler() -> Tuple[str, dict]:
    """Locates g++ with AVX2 support from PATH or known w64devkit directories."""
    cpp_dir = os.path.dirname(os.path.abspath(__file__))
    controller_dir = os.path.dirname(cpp_dir)
    workspace_root = os.path.abspath(os.path.join(controller_dir, *[".."] * 4))
    # If workspace_root does not contain .git, step up one more level
    if not os.path.exists(os.path.join(workspace_root, ".git")):
        workspace_root = os.path.dirname(workspace_root)

    candidates = [
        # Check system PATH first
        shutil.which("g++"),
        # Check workspace tools
        os.path.join(workspace_root, "webots", "optimization", "ga_planner_optimizer", "w64devkit", "bin", "g++.exe"),
        os.path.join(workspace_root, "webots", "tools", "w64devkit", "bin", "g++.exe"),
        os.path.join(workspace_root, "tools", "w64devkit", "bin", "g++.exe"),
        # Direct relative path from cpp_dir
        os.path.abspath(os.path.join(cpp_dir, "..", "..", "..", "..", "optimization", "ga_planner_optimizer", "w64devkit", "bin", "g++.exe")),
    ]

    env = os.environ.copy()
    for cand in candidates:
        if cand and os.path.exists(cand):
            bin_dir = os.path.dirname(cand)
            env["PATH"] = bin_dir + os.pathsep + env.get("PATH", "")
            return cand, env

    return "g++", env


def compile_costmap(compiler: str, env: dict, cpp_dir: str, output_dir: str) -> bool:
    source_file = os.path.join(cpp_dir, "smart_hybrid_costmap_front.cpp")
    dll_output = os.path.join(output_dir, "smart_hybrid_costmap_front.dll")

    old_dll = dll_output + ".old"
    if os.path.exists(dll_output):
        try:
            os.replace(dll_output, old_dll)
        except Exception as e:
            print(f"[Costmap] Notice: Could not rotate existing DLL: {e}")

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

    print(f"[BUILD 1/2] Compiling Smart Hybrid Costmap DLL -> {os.path.basename(dll_output)}")
    t0 = time.perf_counter()
    result = subprocess.run(cmd, env=env, capture_output=True, text=True)
    dt = (time.perf_counter() - t0) * 1000.0

    if result.returncode != 0:
        print("[BUILD ERROR] Costmap compilation failed:")
        print(result.stderr)
        return False

    if os.path.exists(old_dll):
        try:
            os.remove(old_dll)
        except Exception:
            pass

    size_kb = os.path.getsize(dll_output) / 1024.0
    print(f"  -> SUCCESS ({size_kb:.1f} KB in {dt:.0f} ms)")
    return True


def compile_sl_mpc(compiler: str, env: dict, cpp_dir: str, output_dir: str) -> bool:
    source_file = os.path.join(cpp_dir, "curvilinear_sl_mpc.cpp")
    dll_output = os.path.join(output_dir, "curvilinear_sl_mpc.dll")
    osqp_dir = os.path.join(cpp_dir, "osqp_codegen_sl_mpc")
    osqp_lib = os.path.join(osqp_dir, "libosqp.a")

    if not os.path.exists(osqp_lib):
        print(f"[BUILD ERROR] libosqp.a not found at {osqp_lib}")
        return False

    inc_dirs = [
        f"-I{cpp_dir}",
        f"-I{osqp_dir}",
        f"-I{os.path.join(osqp_dir, 'inc', 'public')}",
        f"-I{os.path.join(osqp_dir, 'inc', 'private')}",
    ]

    old_dll = dll_output + ".old"
    if os.path.exists(dll_output):
        try:
            os.replace(dll_output, old_dll)
        except Exception as e:
            print(f"[SL-MPC] Notice: Could not rotate existing DLL: {e}")

    cmd = [
        compiler,
        "-O3",
        "-shared",
        "-fPIC",
        "-mavx2",
        "-fno-math-errno",
        "-std=c++17",
        "-Wall",
        "-Wextra",
    ] + inc_dirs + [
        "-o", dll_output,
        source_file,
        osqp_lib,
    ]

    print(f"[BUILD 2/2] Compiling Curvilinear SL-MPC DLL -> {os.path.basename(dll_output)}")
    t0 = time.perf_counter()
    result = subprocess.run(cmd, env=env, capture_output=True, text=True)
    dt = (time.perf_counter() - t0) * 1000.0

    if result.returncode != 0:
        print("[BUILD ERROR] SL-MPC compilation failed:")
        print(result.stderr)
        return False

    if os.path.exists(old_dll):
        try:
            os.remove(old_dll)
        except Exception:
            pass

    size_kb = os.path.getsize(dll_output) / 1024.0
    print(f"  -> SUCCESS ({size_kb:.1f} KB in {dt:.0f} ms)")
    return True


def main():
    cpp_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.dirname(cpp_dir)

    compiler, env = find_compiler()
    print(f"[Compiler] Using: {compiler}")

    build_costmap = "--sl-mpc-only" not in sys.argv
    build_mpc = "--costmap-only" not in sys.argv

    success = True
    if build_costmap:
        if not compile_costmap(compiler, env, cpp_dir, output_dir):
            success = False

    if build_mpc:
        if not compile_sl_mpc(compiler, env, cpp_dir, output_dir):
            success = False

    if not success:
        sys.exit(1)
    print("\n[ALL BUILDS COMPLETED SUCCESSFULLY]")


if __name__ == "__main__":
    main()
