"""
================================================================================
  GENETIC ALGORITHM SIMULATION EVALUATOR FOR FOLKRACE MPPI
================================================================================
  - Persistent Webots Supervisor Worker Pool for Ultra-High Throughput Optimization
  - Sub-5ms Instantaneous Trial Resets via Local TCP Sockets
  - 60 Hz Sensor Sampling & 16ms Physics Alignment
  - Automatic Process Recovery & Graceful Shutdown
================================================================================
"""

import os
import sys
import json
import time
import math
import socket
import queue
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, List, Tuple, Optional, Any, Union

import numpy as np

# Ensure controllers are on sys.path
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FOLKRACE_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "folkrace_drive")
MPPI_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "mppi_controller")

for p in [WORKSPACE_ROOT, FOLKRACE_DIR, MPPI_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from mppi_folkrace_controller import MPPIGenome

# Webots binary and world defaults
DEFAULT_WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
DEFAULT_WORLD_PATH = os.path.join(WORKSPACE_ROOT, "my_project", "worlds", "Cyan_Champion_Track_GA.wbt")


def compute_fitness(metrics: Dict[str, Any]) -> float:
    """
    Computes a scalar optimization objective from Webots trial metrics.
    Higher score is strictly better.
    """
    dist_m = float(metrics.get("distance_m", 0.0))
    mean_speed = float(metrics.get("mean_speed_mps", 0.0))
    laps = int(metrics.get("laps_completed", 0))
    collisions = int(metrics.get("collision_count", 0))
    wall_cost = float(metrics.get("mean_wall_proximity_cost", 0.0))
    steering_var = float(metrics.get("steering_variance", 0.0))
    stuck = bool(metrics.get("stuck_abort", False))

    if collisions > 0 or stuck:
        # Crashed or stalled run
        fitness = -50000.0 + (dist_m * 10.0) - (collisions * 1000.0)
    else:
        # Clean surviving run
        dist_reward = dist_m * 200.0
        speed_reward = mean_speed * 100.0
        lap_reward = laps * 2000.0
        wall_penalty = min(1500.0, wall_cost * 0.002)
        jerk_penalty = steering_var * 10.0
        fitness = dist_reward + speed_reward + lap_reward - wall_penalty - jerk_penalty

    return float(fitness)


class PersistentWebotsWorker:
    """
    Manages a single long-lived Webots process.
    Re-uses the same simulation instance across multiple candidate evaluations
    via local TCP socket communication and Webots Supervisor pose reset.
    """

    def __init__(
        self,
        worker_id: int,
        base_port: int = 10100,
        webots_bin: str = DEFAULT_WEBOTS_BIN,
        world_path: str = DEFAULT_WORLD_PATH,
    ):
        self.worker_id = worker_id
        self.port = base_port + worker_id
        self.webots_bin = webots_bin
        self.world_path = world_path
        self.proc: Optional[subprocess.Popen] = None
        self.start_worker()

    def start_worker(self):
        env = os.environ.copy()
        env["MPPI_WORKER_PORT"] = str(self.port)
        env["MPPI_PORT_OFFSET"] = str(self.worker_id * 20)
        env["QUIT_ON_CRASH"] = "0"
        env["FAST_SIMULATION"] = "1"

        cmd = [
            self.webots_bin,
            "--mode=fast",
            "--no-rendering",
            "--batch",
            "--minimize",
            "--stdout",
            "--stderr",
            self.world_path,
        ]

        self.proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Poll socket until worker server is up
        t0 = time.time()
        connected = False
        while time.time() - t0 < 30.0:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect(("127.0.0.1", self.port))
                s.close()
                connected = True
                break
            except Exception:
                time.sleep(0.15)

        if not connected:
            print(f"[Worker {self.worker_id}] Warning: Timed out waiting for port {self.port}.", flush=True)

    def evaluate(
        self,
        genome_arr: np.ndarray,
        sim_duration_s: float = 20.0,
        timeout_wall_s: float = 90.0,
    ) -> Tuple[float, Dict[str, Any]]:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout_wall_s)
            s.connect(("127.0.0.1", self.port))
            req = {
                "type": "EVAL",
                "genome": genome_arr.tolist(),
                "duration_s": sim_duration_s,
            }
            req_data = (json.dumps(req) + "\n").encode("utf-8")
            s.sendall(req_data)

            raw_res = ""
            while "\n" not in raw_res:
                chunk = s.recv(16384).decode("utf-8")
                if not chunk:
                    break
                raw_res += chunk
            s.close()

            if raw_res:
                metrics = json.loads(raw_res.strip())
                fitness = compute_fitness(metrics)
                return fitness, metrics
        except Exception as e:
            print(f"[Worker {self.worker_id}] Socket error on port {self.port}: {e}. Restarting instance...", flush=True)
            self.restart()

        fallback_metrics = {
            "status": "WORKER_ERROR",
            "distance_m": 0.0,
            "mean_speed_mps": 0.0,
            "max_speed_mps": 0.0,
            "laps_completed": 0,
            "lap_times": [],
            "best_lap_time_s": 999.0,
            "collision_count": 1,
            "steering_variance": 0.0,
            "mean_wall_proximity_cost": 0.0,
            "sim_duration_s": 0.0,
            "stuck_abort": True,
        }
        return -50000.0, fallback_metrics

    def restart(self):
        self.shutdown()
        time.sleep(0.5)
        self.start_worker()

    def shutdown(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect(("127.0.0.1", self.port))
            s.sendall(b'{"type":"SHUTDOWN"}\n')
            s.close()
        except Exception:
            pass

        if self.proc:
            try:
                self.proc.wait(timeout=2.0)
            except Exception:
                self.proc.kill()
            self.proc = None


class PersistentWorkerPool:
    """
    Thread-safe persistent worker pool managing N Webots instances concurrently.
    """

    def __init__(self, num_workers: int = 10, base_port: int = 10100):
        self.num_workers = num_workers
        self.base_port = base_port
        self.workers: List[PersistentWebotsWorker] = []
        self.available_queue: queue.Queue = queue.Queue()
        self.init_workers()

    def init_workers(self):
        print(f"[Worker Pool] Launching {self.num_workers} persistent headless Webots instances in parallel...", flush=True)
        t0 = time.perf_counter()
        with ThreadPoolExecutor(max_workers=self.num_workers) as executor:
            futures = [
                executor.submit(PersistentWebotsWorker, i, self.base_port)
                for i in range(self.num_workers)
            ]
            for f in futures:
                self.workers.append(f.result())

        for w in self.workers:
            self.available_queue.put(w)
        dt = time.perf_counter() - t0
        print(f"[Worker Pool] All {self.num_workers} persistent workers online and ready in {dt:.1f}s!", flush=True)

    def evaluate_genome(self, genome_arr: np.ndarray, sim_duration_s: float = 20.0) -> Tuple[float, Dict[str, Any]]:
        worker = self.available_queue.get()
        try:
            return worker.evaluate(genome_arr, sim_duration_s=sim_duration_s)
        finally:
            self.available_queue.put(worker)

    def evaluate_batch(
        self,
        population: List[np.ndarray],
        sim_duration_s: float = 20.0,
    ) -> List[Tuple[float, Dict[str, Any]]]:
        results: List[Optional[Tuple[float, Dict[str, Any]]]] = [None] * len(population)
        with ThreadPoolExecutor(max_workers=self.num_workers) as executor:
            future_to_idx = {
                executor.submit(self.evaluate_genome, ind, sim_duration_s): i
                for i, ind in enumerate(population)
            }
            for future in as_completed(future_to_idx):
                idx = future_to_idx[future]
                results[idx] = future.result()
        return [r for r in results if r is not None]

    def shutdown(self):
        print(f"[Worker Pool] Shutting down {len(self.workers)} persistent workers...", flush=True)
        with ThreadPoolExecutor(max_workers=len(self.workers)) as executor:
            futures = [executor.submit(w.shutdown) for w in self.workers]
            for f in futures:
                try:
                    f.result()
                except Exception:
                    pass
        self.workers.clear()


def evaluate_genome(
    genome_or_array: Union[MPPIGenome, np.ndarray, list],
    worker_id: int = 0,
    sim_duration_s: float = 20.0,
    timeout_wall_s: float = 45.0,
    webots_bin: str = DEFAULT_WEBOTS_BIN,
    world_path: str = DEFAULT_WORLD_PATH,
    verbose: bool = False,
) -> Tuple[float, Dict[str, Any]]:
    """Single-shot legacy fallback evaluation for smoke tests."""
    if hasattr(genome_or_array, "to_array"):
        genome_arr = genome_or_array.to_array()
    elif isinstance(genome_or_array, (np.ndarray, list)):
        genome_arr = np.array(genome_or_array, dtype=np.float64)
    else:
        raise ValueError(f"Unsupported genome type: {type(genome_or_array)}")

    port_offset = worker_id * 20
    eval_file = os.path.join(WORKSPACE_ROOT, f"eval_res_worker_{worker_id}.json")
    if os.path.exists(eval_file):
        try:
            os.remove(eval_file)
        except Exception:
            pass

    env = os.environ.copy()
    env["WEBOTS_RUN_SECONDS"] = str(float(sim_duration_s))
    env["MPPI_PORT_OFFSET"] = str(port_offset)
    env["MPPI_EVAL_FILE"] = eval_file
    env["MPPI_GENOME_JSON"] = json.dumps(genome_arr.tolist())
    env["QUIT_ON_CRASH"] = "1"
    env["FAST_SIMULATION"] = "1"

    cmd = [
        webots_bin,
        "--mode=fast",
        "--no-rendering",
        "--batch",
        "--minimize",
        "--stdout",
        "--stderr",
        world_path,
    ]

    t0_wall = time.perf_counter()
    metrics = None
    raw_output_lines = []

    try:
        proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

        for line in proc.stdout:
            raw_output_lines.append(line)
            if verbose:
                print(f"[Worker {worker_id}] {line.rstrip()}", flush=True)

            tag = None
            if "[EVAL_RESULT_JSON]" in line:
                tag = "[EVAL_RESULT_JSON]"
            elif "[EVAL_METRICS_JSON]" in line:
                tag = "[EVAL_METRICS_JSON]"

            if tag is not None:
                try:
                    json_str = line.split(tag)[1].strip()
                    metrics = json.loads(json_str)
                except Exception as e:
                    if verbose:
                        print(f"[Worker {worker_id}] JSON parse error: {e}", flush=True)

        try:
            proc.wait(timeout=max(5.0, timeout_wall_s - (time.perf_counter() - t0_wall)))
        except subprocess.TimeoutExpired:
            proc.kill()

    except Exception as e:
        if verbose:
            print(f"[Worker {worker_id}] Exception spawning Webots: {e}", flush=True)

    if metrics is None and os.path.exists(eval_file):
        try:
            with open(eval_file, "r", encoding="utf-8") as f:
                metrics = json.load(f)
        except Exception:
            pass

    if metrics is None:
        metrics = {
            "status": "PROCESS_ERROR",
            "distance_m": 0.0,
            "mean_speed_mps": 0.0,
            "max_speed_mps": 0.0,
            "laps_completed": 0,
            "lap_times": [],
            "best_lap_time_s": 999.0,
            "collision_count": 1,
            "steering_variance": 0.0,
            "mean_wall_proximity_cost": 0.0,
            "sim_duration_s": round(time.perf_counter() - t0_wall, 2),
            "stuck_abort": True,
        }

    fitness = compute_fitness(metrics)
    return fitness, metrics
