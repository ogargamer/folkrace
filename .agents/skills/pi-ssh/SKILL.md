---
name: pi-ssh
description: SSH connection, command execution, and code deployment for Raspberry Pi 4 (pi4).
---

# Raspberry Pi 4 SSH

- **SSH Target (Old Wi-Fi)**: `robot` (`admin@robot`, `~/.ssh/config` hardcodes `192.168.20.120`, uses `~/.ssh/id_ed25519`, sudo password: `l`)
- **SSH Target (Phone Hotspot)**: Dynamic IP (e.g., `10.88.202.59`). Because `~/.ssh/config` hardcodes `192.168.20.120` for `robot` and `robot.local` mDNS may not resolve on hotspots, connect with `ssh admin@<ip>` or `ssh -o HostName=<ip> robot`.
- **Finding Hotspot IP (No Subnet Scan)**: Check the local ARP table (`arp -a`) for the Pi's MAC OUI (`dc-a6-32-...`, e.g. `dc-a6-32-f0-ce-c9`).
- **Local Source**: `pi4/`
- **Remote Target**: `/home/admin/folkrace/`

## Connection Guidelines
- Use a short timeout (`-o ConnectTimeout=2`) when checking Pi availability.
- If the Pi does not respond or times out, **stop immediately**: do NOT retry in a loop and do NOT scan the subnet. Inform the user right away that the Pi is unreachable (e.g. insufficient USB power/brownout, power off, or network loss).

## Quick Commands
- **Run command**: `ssh robot "<command>"`
- **Sync pi4 files**: `scp -r pi4/* admin@robot:/home/admin/folkrace/`
- **Sync planner stack**: `scp webots/simulation/my_project/controllers/tof_path_planner/planner_config.py webots/simulation/my_project/controllers/tof_path_planner/planner_sl_mpc.py webots/simulation/my_project/controllers/tof_path_planner/cpp_smart_hybrid_costmap.py webots/simulation/my_project/controllers/tof_path_planner/tof_gap_scout.py webots/simulation/my_project/controllers/tof_path_planner/ramp_detector.py admin@robot:/home/admin/folkrace/`
- **Check USB/Serial**: `ssh robot "ls -l /dev/ttyUSB* /dev/ttyACM*"`
