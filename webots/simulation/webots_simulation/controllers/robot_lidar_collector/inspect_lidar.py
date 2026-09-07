import os, sys, numpy as np
if "WEBOTS_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))
from controller import Supervisor

def inspect_lidar():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    lidar = robot.getDevice("ms200k_lidar")
    lidar.enable(int(1000 / 12))
    lidar.enablePointCloud()
    
    # Run 2 steps to get 1 full scan
    for _ in range(10):
        robot.step(timestep)
        
    cloud = lidar.getPointCloud()
    if cloud and len(cloud) > 0:
        pts = np.array([[p.x, p.y, p.z] for p in cloud])
        print("Total rays in cloud:", len(pts))
        print("Ray 0 local (x, y, z):", pts[0])
        print("Ray 112 local (x, y, z):", pts[len(pts)//4])
        print("Ray 225 local (x, y, z):", pts[len(pts)//2])
        print("Ray 337 local (x, y, z):", pts[3*len(pts)//4])
        
        # Check angle of Ray 0:
        # Is X forward or Z forward?
        ang0 = np.arctan2(pts[0, 2], pts[0, 0])
        print("Ray 0 angle (rad):", ang0, "(deg):", np.degrees(ang0))
        ang_mid = np.arctan2(pts[len(pts)//2, 2], pts[len(pts)//2, 0])
        print("Ray 225 angle (rad):", ang_mid, "(deg):", np.degrees(ang_mid))

    robot.simulationQuit(0)

if __name__ == "__main__":
    inspect_lidar()
