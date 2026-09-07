import os
import sys
import numpy as np

GA_DIR = r"c:\Users\nurmi\Documents\Da Winna\ga_planner_optimizer"
if GA_DIR not in sys.path:
    sys.path.insert(0, GA_DIR)

from controllers.ga_supervisor_worker.ga_supervisor_worker import WebotsTrackProgressTracker as WorkerTracker, START_POS as WORKER_START, START_ROT as WORKER_ROT
from controllers.champion_live_preview.champion_live_preview import WebotsTrackProgressTracker as PreviewTracker, START_POS as PREVIEW_START, START_ROT as PREVIEW_ROT

def verify_track_progression():
    traj_path = os.path.join(GA_DIR, "data", "cyan_champion_trajectory.csv")
    traj = np.loadtxt(traj_path, delimiter=",", skiprows=1)
    
    print(f"Point 0 in CSV: x={traj[0,0]}, y={traj[0,1]}, yaw={traj[0,3]}")
    print(f"Worker START_POS: {WORKER_START} | PREVIEW START_POS: {PREVIEW_START}")
    print(f"Worker START_ROT: {WORKER_ROT} | PREVIEW START_ROT: {PREVIEW_ROT}")
    
    assert abs(WORKER_START[0] - traj[0,0]) < 1e-4
    assert abs(WORKER_START[1] - traj[0,1]) < 1e-4
    assert abs(PREVIEW_START[0] - traj[0,0]) < 1e-4
    assert abs(PREVIEW_START[1] - traj[0,1]) < 1e-4
    
    for TrackerCls, name in [(WorkerTracker, "SupervisorWorker"), (PreviewTracker, "ChampionPreview")]:
        tracker = TrackerCls(traj_path)
        tracker.reset(WORKER_START)
        assert tracker.last_idx == 0
        assert tracker.max_track_progress == 0.0
        
        # 1. Drive forward 200 points
        for i in range(1, 200):
            tracker.update(tracker.traj_pts[i])
        prog_200 = tracker.max_track_progress
        assert prog_200 > 2.5, f"Expected >2.5m, got {prog_200}"
        
        # 2. Drive backward 100 points
        for i in range(198, 99, -1):
            tracker.update(tracker.traj_pts[i])
        assert tracker.max_track_progress == prog_200, f"Backward movement altered progress in {name}!"
        
        # 3. Drive forward back to 200
        for i in range(101, 200):
            tracker.update(tracker.traj_pts[i])
        assert tracker.max_track_progress == prog_200, f"Retracing old ground altered progress in {name}!"
        
        # 4. Side-to-side wiggling at point 200
        pt200 = tracker.traj_pts[200]
        pt201 = tracker.traj_pts[201]
        v = pt201 - pt200
        n = np.array([-v[1], v[0]]) / np.linalg.norm(v)
        for offset in [-0.20, 0.20, -0.15, 0.15]:
            tracker.update(pt200 + offset * n)
        assert abs(tracker.max_track_progress - prog_200) < 0.05, f"Side wiggling exploited {name}!"
        
        # 5. Lap wrap test: simulate entire lap
        tracker.reset(WORKER_START)
        for i in range(1, len(tracker.traj_pts)):
            tracker.update(tracker.traj_pts[i])
        tracker.update(tracker.traj_pts[0])  # lap boundary
        tracker.update(tracker.traj_pts[10])
        assert tracker.current_lap == 1, f"Expected 1 completed lap, got {tracker.current_lap}"
        assert tracker.max_track_progress > tracker.total_length, f"Expected progress > total_length"
        print(f"[{name}] Completed 1 lap test cleanly: dist={tracker.max_track_progress:.2f}m, laps={tracker.current_lap}")

    # Verify .wbt world files
    for w in ["Cyan_Champion_GA.wbt", "Cyan_Champion_Live_Preview.wbt"]:
        w_path = os.path.join(GA_DIR, "worlds", w)
        with open(w_path, "r", encoding="utf-8") as f:
            text = f.read()
        assert "translation 0.0819 0.11927 0.028" in text, f"Robot start position mismatch in {w}"
        assert "rotation 0 0 1 -1.802096" in text, f"Robot start rotation mismatch in {w}"
        print(f"[{w}] Verified robot translation and rotation match racing line point 0!")

    print("\n[ALL TESTS PASSED] True track progression and racing line starting alignment successfully verified!")

if __name__ == "__main__":
    verify_track_progression()
