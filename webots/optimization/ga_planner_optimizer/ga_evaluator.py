"""
================================================================================
  HEADLESS WEBOTS EVALUATOR & PARALLEL WORKER POOL
================================================================================
  Spawns and manages long-lived headless Webots processes across CPU cores.
  Communicates with supervisor workers via local TCP sockets for sub-second trial
  dispatch, collecting 20-second distance metrics and scalar fitness.
================================================================================
"""

import os
import sys
import json
import time
import socket
import subprocess
import queue
from typing import Dict, List, Optional, Tuple, Any
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))

def _find_world_path(start_dir: str) -> str:
    p = start_dir
    for _ in range(6):
        cand = os.path.join(p, "webots", "simulation", "my_project", "worlds", "track_racing.wbt")
        if os.path.exists(cand):
            return cand
        cand2 = os.path.join(p, "simulation", "my_project", "worlds", "track_racing.wbt")
        if os.path.exists(cand2):
            return cand2
        parent = os.path.dirname(p)
        if parent == p:
            break
        p = parent
    return os.path.join(start_dir, "track_racing.wbt")

DEFAULT_WORLD_PATH = _find_world_path(CURR_DIR)
OPTIMIZER_PROJECT_DIR = os.path.dirname(os.path.dirname(DEFAULT_WORLD_PATH)) if os.path.exists(DEFAULT_WORLD_PATH) else CURR_DIR
MY_PROJECT_DIR = OPTIMIZER_PROJECT_DIR
WEBOTS_DIR = os.path.dirname(os.path.dirname(MY_PROJECT_DIR))
WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)

for p in [OPTIMIZER_PROJECT_DIR, CURR_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from ga_planner_config import (
    NUM_CPU_CORES,
    WEBOTS_BINARY,
    BASE_WORKER_PORT,
    POPULATION_SIZE,
    NUM_GENERATIONS,
    EVALUATION_DURATION_S,
    TIMEOUT_PER_TRIAL_WALL_S,
    ACTIVE_PLANNER,
    compute_scalar_fitness,
    get_genome_class,
    BasePlannerGenome,
)


class HeadlessWebotsWorker:
    """Manages a single long-lived headless Webots process on a dedicated port."""

    def __init__(
        self,
        worker_id: int,
        base_port: int = BASE_WORKER_PORT,
        webots_bin: str = WEBOTS_BINARY,
        world_path: str = DEFAULT_WORLD_PATH,
        rendered: bool = False,
        controller: Optional[any] = None,
    ):
        self.worker_id = worker_id
        self.port = base_port + worker_id
        self.webots_bin = webots_bin
        self.world_path = world_path
        self.rendered = rendered
        self.controller = controller
        self.proc: Optional[subprocess.Popen] = None
        self.log_file = None
        self._is_stopped = False
        self.start_process()

    def _kill_proc(self):
        """Kills active process tree and closes log file without marking worker permanently stopped."""
        if self.proc is not None:
            pid = self.proc.pid
            # Try graceful TCP shutdown first
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect(("127.0.0.1", self.port))
                s.sendall((json.dumps({"cmd": "SHUTDOWN"}) + "\n").encode("utf-8"))
                s.close()
            except Exception:
                pass

            # Force-kill the entire process tree
            if sys.platform == "win32":
                try:
                    subprocess.run(
                        ["taskkill", "/F", "/T", "/PID", str(pid)],
                        capture_output=True,
                        timeout=5,
                    )
                except Exception:
                    pass
            else:
                try:
                    import signal as _signal
                    os.killpg(os.getpgid(pid), _signal.SIGKILL)
                except Exception:
                    pass

            try:
                self.proc.wait(timeout=2.0)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self.proc = None

        if self.log_file is not None and not self.log_file.closed:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None

    def start_process(self):
        """Launches Webots in fast headless batch mode or rendered real-time preview mode."""
        if self._is_stopped:
            return
        if self.controller and (getattr(self.controller, "abort_requested", False) or getattr(self.controller, "stop_requested", False)):
            self._is_stopped = True
            return

        self._kill_proc()

        env = os.environ.copy()
        webots_bin_dir = os.path.dirname(self.webots_bin)
        webots_home = r"C:\Program Files\Webots"
        py_dir = os.path.dirname(sys.executable)
        py_scripts = os.path.join(py_dir, "Scripts")
        env["WEBOTS_HOME"] = webots_home
        env["PATH"] = webots_bin_dir + os.pathsep + py_dir + os.pathsep + py_scripts + os.pathsep + env.get("PATH", "")
        env["PYTHONPATH"] = os.pathsep.join([
            CURR_DIR,
            OPTIMIZER_PROJECT_DIR,
            os.path.join(OPTIMIZER_PROJECT_DIR, "controllers", "tof_path_planner"),
            os.path.join(CURR_DIR, "data"),
            WORKSPACE_ROOT,
        ])
        env["GA_WORKER_PORT"] = str(self.port)
        env["GA_WORKER_ID"] = str(self.worker_id)
        env["WEBOTS_CONTROLLER_MODE"] = "ga_eval"
        env["PYTHONUNBUFFERED"] = "1"

        if self.rendered:
            # Rendered real-time visual mode for human inspection
            cmd = [
                self.webots_bin,
                f"--port={self.port}",
                "--mode=realtime",
                "--batch",
                self.world_path,
            ]
            startupinfo = None
            creationflags = 0
        else:
            # Headless multi-core fast batch mode
            cmd = [
                self.webots_bin,
                f"--port={self.port}",
                "--mode=fast",
                "--no-rendering",
                "--batch",
                "--minimize",
                "--stdout",
                "--stderr",
                self.world_path,
            ]
            startupinfo = None
            creationflags = 0
            if sys.platform == "win32":
                startupinfo = subprocess.STARTUPINFO()
                startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                startupinfo.wShowWindow = 0  # SW_HIDE
                creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0x08000000)

        if self._is_stopped or (self.controller and getattr(self.controller, "abort_requested", False)):
            return

        try:
            log_dir = os.path.join(CURR_DIR, "logs")
            os.makedirs(log_dir, exist_ok=True)
            self.log_path = os.path.join(log_dir, f"worker_{self.worker_id}.log")
            self.log_file = open(self.log_path, "w", encoding="utf-8")

            self.proc = subprocess.Popen(
                cmd,
                env=env,
                cwd=OPTIMIZER_PROJECT_DIR,
                startupinfo=startupinfo,
                creationflags=creationflags,
                stdout=self.log_file,
                stderr=subprocess.STDOUT,
            )
        except Exception as e:
            print(f"[Worker {self.worker_id}] Error launching Webots executable: {e}", flush=True)
            return

        # Poll socket until worker is up (up to 45 seconds)
        t0 = time.time()
        connected = False
        while time.time() - t0 < 45.0:
            if self._is_stopped or (self.controller and getattr(self.controller, "abort_requested", False)):
                self.stop_process()
                return
            if self.ping():
                connected = True
                break
            time.sleep(0.3)

        if not connected and not self._is_stopped:
            print(f"[Worker {self.worker_id}] Warning: Timed out connecting to port {self.port}.", flush=True)
        elif connected:
            print(f"  [Worker {self.worker_id}] Connected and ready on Port {self.port}.", flush=True)

    def ping(self) -> bool:
        """Checks if Webots supervisor worker server is listening."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect(("127.0.0.1", self.port))
            req = json.dumps({"cmd": "PING"}) + "\n"
            s.sendall(req.encode("utf-8"))
            data = s.recv(1024)
            s.close()
            return b"PONG" in data
        except Exception:
            return False

    def evaluate(
        self,
        planner_name: str,
        gene_dict: Dict[str, Any],
        duration_s: float = EVALUATION_DURATION_S,
    ) -> Tuple[float, Dict[str, Any]]:
        """Sends candidate genome to Webots supervisor and receives 20s run metrics."""
        fallback_metrics = {
            "distance_m": 0.0,
            "mean_speed_mps": 0.0,
            "laps_completed": 0,
            "crashed": True,
            "stalled": False,
            "aborted": True,
            "fitness": -5000.0,
        }

        if self._is_stopped or (self.controller and getattr(self.controller, "abort_requested", False)):
            return -5000.0, fallback_metrics

        for attempt in range(2):
            if self._is_stopped or (self.controller and getattr(self.controller, "abort_requested", False)):
                return -5000.0, fallback_metrics

            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(TIMEOUT_PER_TRIAL_WALL_S)
                s.connect(("127.0.0.1", self.port))

                req = json.dumps({
                    "cmd": "EVAL",
                    "planner": planner_name,
                    "genes": gene_dict,
                    "duration_s": duration_s,
                }) + "\n"

                s.sendall(req.encode("utf-8"))

                # Read response until newline
                buf = ""
                while "\n" not in buf:
                    if self._is_stopped or (self.controller and getattr(self.controller, "abort_requested", False)):
                        try:
                            s.close()
                        except Exception:
                            pass
                        return -5000.0, fallback_metrics
                    chunk = s.recv(4096).decode("utf-8", errors="ignore")
                    if not chunk:
                        break
                    buf += chunk
                s.close()

                resp = json.loads(buf.strip())
                if resp.get("status") == "OK":
                    metrics = resp.get("metrics", {})
                    fitness = compute_scalar_fitness(metrics, gene_dict=gene_dict)
                    metrics["fitness"] = fitness
                    return fitness, metrics
            except Exception as e:
                if self._is_stopped or (self.controller and getattr(self.controller, "abort_requested", False)):
                    break
                print(f"[Worker {self.worker_id}] Attempt {attempt + 1} communication failure: {e}", flush=True)
                time.sleep(1.0)
                if not self._is_stopped and not (self.controller and getattr(self.controller, "abort_requested", False)):
                    self.start_process()

        return -5000.0, fallback_metrics

    def stop_process(self):
        """Terminates Webots process, kills child processes, and permanently marks as stopped."""
        self._is_stopped = True
        self._kill_proc()


class ParallelWorkerPool:
    """Manages pool of headless Webots workers for multi-core parallel GA evaluations."""

    def __init__(
        self,
        num_workers: int = NUM_CPU_CORES,
        base_port: int = BASE_WORKER_PORT,
        webots_bin: str = WEBOTS_BINARY,
        world_path: str = DEFAULT_WORLD_PATH,
        controller: Optional[any] = None,
    ):
        self.num_workers = num_workers
        self.controller = controller
        self.workers: List[HeadlessWebotsWorker] = []
        self._shutdown_called = False

        # Wire pool into controller immediately so abort() can shut down workers even during init
        if self.controller is not None:
            self.controller._pool = self

        if self.controller and getattr(self.controller, "abort_requested", False):
            self._shutdown_called = True
            return

        print("=" * 80, flush=True)
        print(f"  INITIALIZING PARALLEL GA WORKER POOL ({num_workers} Webots Instances in Parallel)", flush=True)
        print("=" * 80, flush=True)

        def _init_worker(w_idx: int) -> Optional[HeadlessWebotsWorker]:
            if self._shutdown_called or (self.controller and getattr(self.controller, "abort_requested", False)):
                return None
            w = HeadlessWebotsWorker(
                worker_id=w_idx,
                base_port=base_port,
                webots_bin=webots_bin,
                world_path=world_path,
                controller=self.controller,
            )
            if self._shutdown_called or (self.controller and getattr(self.controller, "abort_requested", False)):
                w.stop_process()
                return None
            return w

        with ThreadPoolExecutor(max_workers=num_workers) as executor:
            futures = [executor.submit(_init_worker, i) for i in range(num_workers)]
            for f in futures:
                if self._shutdown_called or (self.controller and getattr(self.controller, "abort_requested", False)):
                    break
                try:
                    w = f.result()
                    if w is not None:
                        self.workers.append(w)
                except Exception as e:
                    print(f"[WorkerPool Init Error] {e}", flush=True)

        if self._shutdown_called or (self.controller and getattr(self.controller, "abort_requested", False)):
            self.shutdown()

    def evaluate_population(
        self,
        planner_name: str,
        population_genomes: List[BasePlannerGenome],
        duration_s: float = EVALUATION_DURATION_S,
        controller: Optional[any] = None,
    ) -> List[Tuple[float, Dict[str, Any]]]:
        """Evaluates an entire generation of candidate genomes in parallel."""
        ctrl = controller or self.controller
        if ctrl and getattr(ctrl, "abort_requested", False):
            return []
        if self._shutdown_called or not self.workers:
            return []

        results: List[Optional[Tuple[float, Dict[str, Any]]]] = [None] * len(population_genomes)

        worker_q = queue.Queue()
        for w in self.workers:
            worker_q.put(w)

        def eval_task(cand_idx: int, genome: BasePlannerGenome):
            if ctrl and getattr(ctrl, "abort_requested", False):
                return cand_idx, -5000.0, {"crashed": True, "aborted": True}
            if self._shutdown_called or not self.workers:
                return cand_idx, -5000.0, {"crashed": True, "aborted": True}
            worker = worker_q.get()
            try:
                fitness, metrics = worker.evaluate(planner_name, genome.to_dict(), duration_s)
                return cand_idx, fitness, metrics
            finally:
                worker_q.put(worker)

        with ThreadPoolExecutor(max_workers=len(self.workers)) as executor:
            futures = []
            for idx, genome in enumerate(population_genomes):
                if ctrl and getattr(ctrl, "abort_requested", False):
                    break
                futures.append(executor.submit(eval_task, idx, genome))

            for future in as_completed(futures):
                if ctrl and getattr(ctrl, "abort_requested", False):
                    for f in futures:
                        f.cancel()
                    break
                try:
                    cand_idx, fitness, metrics = future.result()
                    results[cand_idx] = (fitness, metrics)
                except Exception:
                    pass

        return [r for r in results if r is not None]

    def shutdown(self):
        """Stops all Webots worker processes."""
        self._shutdown_called = True
        print("[ParallelWorkerPool] Shutting down all Webots instances...", flush=True)
        for w in list(self.workers):
            if w is not None:
                try:
                    w.stop_process()
                except Exception:
                    pass
        self.workers.clear()


def run_rendered_preview(
    planner_name: str,
    gene_dict: Optional[Dict[str, Any]] = None,
    duration_s: float = 20.0,
    port: int = 10299,
    controller: Optional[any] = None,
) -> Tuple[float, Dict[str, Any]]:
    """
    Executes a single rendered, real-time 20-second run of the specified planner in Webots GUI.
    Allows user to observe vehicle behavior and abort if anything is wrong.
    """
    if gene_dict is None:
        genome_cls = get_genome_class(planner_name)
        gene_dict = genome_cls().to_dict()

    if controller and (getattr(controller, "abort_requested", False) or getattr(controller, "stop_requested", False)):
        return -5000.0, {"crashed": True, "aborted": True}

    print(f"\n[Preview] Starting 20s Rendered Real-Time Preview for {planner_name} on Port {port}...", flush=True)
    worker = HeadlessWebotsWorker(
        worker_id=99,
        base_port=port,
        rendered=True,
        controller=controller,
    )

    if controller is not None:
        controller._preview_worker = worker

    try:
        if controller and (getattr(controller, "abort_requested", False) or getattr(controller, "stop_requested", False)):
            return -5000.0, {"crashed": True, "aborted": True}

        fitness, metrics = worker.evaluate(
            planner_name=planner_name,
            gene_dict=gene_dict,
            duration_s=duration_s,
        )
        return fitness, metrics
    finally:
        if controller is not None:
            controller._preview_worker = None
        worker.stop_process()

