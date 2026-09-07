import os, sys, numpy as np
os.environ["WEBOTS_HOME"] = r"C:\Program Files\Webots"
sys.path.append(r"C:\Program Files\Webots\lib\controller\python")
from controller import Supervisor

def test():
    # Let's check rotation matrix convention in Webots
    # When rotation is [0, 1, 0, theta], what is R?
    theta = 0.5 # rad (~28.6 deg)
    # Standard Right-Hand Rule around +Y (NUE: X=right, Y=up, Z=depth):
    # Rotating +Z towards +X or +X towards +Z?
    # In NUE right hand rule: thumb along +Y (up), fingers curl from +Z to +X
    # So X' = X*cos(theta) + Z*sin(theta)
    #    Z' = -X*sin(theta) + Z*cos(theta)
    print("Testing complete.")

test()
