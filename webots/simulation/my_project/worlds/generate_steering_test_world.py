import os
import re

world_src = os.path.join("webots/simulation/my_project", "worlds", "Cyan_Champion_ToF_Track.wbt")
world_dst = os.path.join("webots/simulation/my_project", "worlds", "steering_test_world.wbt")

with open(world_src, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Header: up to RectangleArena closing bracket (line 67)
header_lines = lines[0:67]

# Robot lines: from Robot { to DEF MOVING_OBSTACLE
start_robot = -1
end_robot = -1
for i, l in enumerate(lines):
    if l.startswith("Robot {"):
        start_robot = i
        break

for i in range(start_robot, len(lines)):
    if lines[i].startswith("DEF MOVING_OBSTACLE"):
        end_robot = i
        break

robot_lines = lines[start_robot:end_robot]
robot_str = "".join(robot_lines)

# Set starting position to 0 0 0.028 rotation 0 0 1 0 (+Y forward)
robot_str = re.sub(r"translation 0\.6 0 0\.028", "translation 0 0 0.028", robot_str, count=1)
robot_str = re.sub(r"rotation 0 0 1 -1\.57079632679", "rotation 0 0 1 0", robot_str, count=1)

# Set DEF for Robot
robot_str = robot_str.replace("Robot {", "DEF FOLKRACE_ROBOT Robot {", 1)

# Set controller to steering_angle_tester
robot_str = re.sub(r'controller "tof_path_planner"', 'controller "steering_angle_tester"', robot_str, count=1)

# Add PositionSensors to servos
# Servo right device
servo_right_pattern = r'(name "servo right"[\s\S]*?maxTorque 0\.2\s*\})'
servo_right_replacement = r'\1\n        PositionSensor {\n          name "servo right sensor"\n        }'
robot_str = re.sub(servo_right_pattern, servo_right_replacement, robot_str, count=1)

# Servo left device
servo_left_pattern = r'(name "servo left"[\s\S]*?maxTorque 0\.2\s*\})'
servo_left_replacement = r'\1\n        PositionSensor {\n          name "servo left sensor"\n        }'
robot_str = re.sub(servo_left_pattern, servo_left_replacement, robot_str, count=1)

# Enable torque feedback on both servos if possible
# Give DEF names to wheels if needed
robot_str = robot_str.replace('endPoint Solid {\n        translation -0.05 -0.07', 'endPoint DEF WHEEL_RL Solid {\n        translation -0.05 -0.07')
robot_str = robot_str.replace('endPoint Solid {\n        translation 0.05 -0.07', 'endPoint DEF WHEEL_RR Solid {\n        translation 0.05 -0.07')
robot_str = robot_str.replace('endPoint Solid {\n        translation 0.0411 0.0625', 'endPoint DEF WHEEL_FR Solid {\n        translation 0.0411 0.0625')
robot_str = robot_str.replace('endPoint Solid {\n        translation -0.0411 0.0625', 'endPoint DEF WHEEL_FL Solid {\n        translation -0.0411 0.0625')

# Add Viewpoint with a good overhead perspective
# Find Viewpoint in header and update
header_str = "".join(header_lines)
# Make arena larger: 20 x 20
header_str = header_str.replace("floorSize 16 16", "floorSize 20 20")

# Adjust Viewpoint to look directly down at the robot
vp_old = r'Viewpoint \{[\s\S]*?position 0\.6236307978610706 -3\.758504738281843 3\.589131455956603\s*\}'
vp_new = """Viewpoint {
  orientation 0 0 1 0
  position 0 0 4.5
}"""
header_str = re.sub(vp_old, vp_new, header_str, count=1)

world_content = header_str + "\n\n" + robot_str

with open(world_dst, "w", encoding="utf-8") as f:
    f.write(world_content)

print("Generated", world_dst, "successfully!")
