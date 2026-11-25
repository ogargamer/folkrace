#!/usr/bin/env python3
"""
================================================================================
  AUTOMATED HEADLESS WEBOTS 3D WRONG-WAY & RECOVERY TEST RUNNER
================================================================================
  Launches headless Webots against test_wrong_way.wbt and track_racing.wbt
  using WEBOTS_CONTROLLER_MODE=wrong_way_eval, collects structured JSON results,
  and prints a pass/fail summary table.
================================================================================
"""

import argparse
import json
import os
import subprocess
import sys
import time
from typing import Optional
import uuid

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBOTS_DIR = os.path.join(REPO_ROOT, "webots")
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
GA_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

WORLDS = {
    "test_wrong_way": os.path.join(MY_PROJECT_DIR, "worlds", "test_wrong_way.wbt"),
    "track_racing": os.path.join(MY_PROJECT_DIR, "worlds", "track_racing.wbt"),
}


def run_world_eval(
    world_key: str,
    scenarios: str,
    render: bool,
    timeout_s: float,
    params_json: str = "",
    port: Optional[int] = None,
) -> dict:
    world_path = WORLDS[world_key]
    if not os.path.isfile(world_path):
        raise FileNotFoundError(f"World not found: {world_path}")
    if not os.path.isfile(WEBOTS_BIN):
        raise FileNotFoundError(f"Webots executable not found: {WEBOTS_BIN}")

    scratch_dir = os.path.join(REPO_ROOT, "scratch")
    os.makedirs(scratch_dir, exist_ok=True)
    out_json = os.path.join(scratch_dir, f"aww_eval_{world_key}_{uuid.uuid4().hex[:8]}.json")

    env = os.environ.copy()
    webots_home = r"C:\Program Files\Webots"
    webots_bin_dir = os.path.dirname(WEBOTS_BIN)
    py_dir = os.path.dirname(sys.executable)
    py_scripts = os.path.join(py_dir, "Scripts")
    env["WEBOTS_HOME"] = webots_home
    env["PATH"] = webots_bin_dir + os.pathsep + py_dir + os.pathsep + py_scripts + os.pathsep + env.get("PATH", "")
    env["PYTHONPATH"] = os.pathsep.join([
        PLANNERS_DIR,
        os.path.join(GA_DIR, "data"),
        GA_DIR,
        MY_PROJECT_DIR,
        REPO_ROOT,
    ])
    env["WEBOTS_CONTROLLER_MODE"] = "wrong_way_eval"
    env["AWW_EVAL_WORLD"] = world_key
    env["AWW_EVAL_SCENARIOS"] = scenarios
    env["AWW_EVAL_OUT"] = out_json
    if params_json:
        env["AWW_EVAL_PARAMS_JSON"] = params_json
    env["PYTHONUNBUFFERED"] = "1"

    port_arg = [f"--port={port}"] if port is not None else []
    if render:
        cmd = [WEBOTS_BIN, *port_arg, "--mode=realtime", "--batch", "--stdout", "--stderr", world_path]
        startupinfo = None
        creationflags = 0
    else:
        cmd = [
            WEBOTS_BIN,
            *port_arg,
            "--mode=fast",
            "--no-rendering",
            "--batch",
            "--minimize",
            "--stdout",
            "--stderr",
            world_path,
        ]
        startupinfo = None
        creationflags = 0
        if sys.platform == "win32":
            startupinfo = subprocess.STARTUPINFO()
            startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            startupinfo.wShowWindow = 0
            creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0x08000000)

    t0 = time.perf_counter()
    proc = subprocess.Popen(
        cmd,
        env=env,
        cwd=MY_PROJECT_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        startupinfo=startupinfo,
        creationflags=creationflags,
    )
    try:
        stdout_txt, _ = proc.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        if sys.platform == "win32":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            proc.kill()
        stdout_txt, _ = proc.communicate()
        return {
            "world": world_key,
            "all_passed": False,
            "results": [{"name": f"{world_key}_timeout", "passed": False, "error": f"Timed out after {timeout_s}s"}],
            "stdout": stdout_txt[-1500:],
        }
    finally:
        if os.path.exists(out_json):
            try:
                with open(out_json, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception as e:
                data = {"world": world_key, "all_passed": False, "results": [], "error": str(e)}
            if not data.get("all_passed", False) and stdout_txt:
                data["stdout"] = stdout_txt[-2500:]
            try:
                os.remove(out_json)
            except OSError:
                pass
        else:
            data = {
                "world": world_key,
                "all_passed": False,
                "results": [{"name": f"{world_key}_no_output", "passed": False, "rc": proc.returncode}],
                "stdout": stdout_txt[-1500:] if stdout_txt else "",
            }

    data["elapsed_s"] = round(time.perf_counter() - t0, 2)
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description="Run automated Webots 3D wrong-way & recovery tests.")
    parser.add_argument("--world", choices=["test_wrong_way", "track_racing", "all"], default="all")
    parser.add_argument("--scenario", default="all", help="Comma-separated scenario names or 'all'")
    parser.add_argument("--render", action="store_true", help="Launch Webots in real-time rendered mode")
    parser.add_argument("--timeout", type=float, default=240.0, help="Per-world wall-clock timeout in seconds")
    parser.add_argument("--params-json", default="", help="Optional JSON string of parameter overrides")
    parser.add_argument("--port", type=int, default=None, help="Optional Webots port")
    args = parser.parse_args()

    world_list = ["test_wrong_way", "track_racing"] if args.world == "all" else [args.world]

    print("=" * 80)
    print("  HEADLESS WEBOTS 3D WRONG-WAY & RECOVERY TEST SUITE")
    print("=" * 80)

    overall_ok = True
    for wk in world_list:
        print(f"\n--- World: {wk} ---")
        res = run_world_eval(wk, args.scenario, args.render, args.timeout, params_json=args.params_json, port=args.port)
        for r in res.get("results", []):
            tag = "PASS" if r.get("passed") else "FAIL"
            detail = {k: v for k, v in r.items() if k not in ("name", "passed")}
            print(f"  [{tag}] {r.get('name', '?'):30s} | {detail}")
        if not res.get("all_passed", False):
            overall_ok = False
            if res.get("stdout"):
                print("  [Webots tail stdout]:\n" + res["stdout"])
        print(f"  World elapsed: {res.get('elapsed_s', '?')}s")

    print("\n" + "=" * 80)
    print(f"  OVERALL RESULT: {'ALL PASSED' if overall_ok else 'FAILED'}")
    print("=" * 80)
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
