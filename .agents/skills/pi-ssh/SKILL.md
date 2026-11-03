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
- **Sync files**: `scp -r pi4/* admin@robot:/home/admin/folkrace/`
- **Check USB/Serial**: `ssh robot "ls -l /dev/ttyUSB* /dev/ttyACM*"`
