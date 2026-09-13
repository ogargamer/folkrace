---
trigger: always_on
description: Workspace file hygiene, anti-bloat, and domain boundary rules.
---

# File Hygiene & Anti-Bloat Rules

- Scratch/temporary scripts belong ONLY in `scratch/`. Delete one-off test scripts after running.
- NEVER create `*_v2`, `*_old`, `*_test2`, `*.bak`, or `*.dll.old`. Edit in place; use git for history.
- NEVER dump raw datasets, CSVs, or `.npz` into source folders. Use `data/` or `scratch/`.
- `launchers/`: Single source of truth for runnable scripts/GUIs. Do not scatter runners elsewhere.
- `controllers/`: Real Webots controllers only, 1:1 matching world files. No library/helper folders.
- `worlds/`: Only active worlds matching controllers. No orphan `.wbproj` or scratch worlds.
- Hardware: Firmware and test benches strictly belong in `esp32/`, `stm32/`, or `raspberry_pi/`.
