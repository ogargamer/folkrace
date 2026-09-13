import sys
import os
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
GA_DIR = os.path.join(THIS_DIR, "..")
sys.path.insert(0, GA_DIR)

from ga_optimizer import GAController
from ga_evaluator import HeadlessWebotsWorker, ParallelWorkerPool
from ga_planner_config import get_genome_class

def test_abort_guards():
    print("Testing abort guards...", flush=True)
    ctrl = GAController()
    ctrl.abort()
    assert ctrl.abort_requested is True

    # Test that creating a worker when controller is already aborted does NOT start a process
    worker = HeadlessWebotsWorker(
        worker_id=88,
        base_port=10288,
        controller=ctrl,
    )
    assert worker._is_stopped is True
    assert worker.proc is None, "Worker should not have started a process!"

    # Test start_process() is a no-op
    worker.start_process()
    assert worker.proc is None, "start_process() should be blocked after abort!"

    # Test evaluate() returns immediately with aborted
    genome_cls = get_genome_class("OPTION_3_SEPTIC")
    fit, metrics = worker.evaluate("OPTION_3_SEPTIC", genome_cls().to_dict(), 5.0)
    assert metrics.get("aborted") is True
    assert worker.proc is None

    # Test ParallelWorkerPool when aborted
    pool = ParallelWorkerPool(num_workers=2, base_port=10280, controller=ctrl)
    assert pool._shutdown_called is True
    assert len(pool.workers) == 0

    results = pool.evaluate_population("OPTION_3_SEPTIC", [genome_cls()], 5.0, controller=ctrl)
    assert len(results) == 0

    print("ALL ABORT GUARDS PASSED SUCCESSFULLY!", flush=True)

if __name__ == "__main__":
    test_abort_guards()
