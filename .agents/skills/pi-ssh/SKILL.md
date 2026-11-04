---
name: pi-ssh
description: SSH connection, command execution, and code deployment for Raspberry Pi 4 (pi4).
---

# Raspberry Pi 4 SSH

- **SSH Target**: `robot` (`admin@robot`, uses `~/.ssh/id_ed25519` for login, sudo password: `l`)
- **Local Source**: `pi4/`
- **Remote Target**: `/home/admin/folkrace/`

## Quick Commands
- **Run command**: `ssh robot "<command>"`
- **Sync pi4 files**: `scp -r pi4/* admin@robot:/home/admin/folkrace/`
- **Sync planner stack**: `scp webots/simulation/my_project/controllers/tof_path_planner/planner_config.py webots/simulation/my_project/controllers/tof_path_planner/planner_sl_mpc.py webots/simulation/my_project/controllers/tof_path_planner/cpp_smart_hybrid_costmap.py webots/simulation/my_project/controllers/tof_path_planner/tof_gap_scout.py webots/simulation/my_project/controllers/tof_path_planner/ramp_detector.py admin@robot:/home/admin/folkrace/`
- **Check USB/Serial**: `ssh robot "ls -l /dev/ttyUSB* /dev/ttyACM*"`
