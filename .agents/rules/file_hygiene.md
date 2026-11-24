---
trigger: always_on
description: Workspace file hygiene and directory boundaries.
---

# File Hygiene

- **Edit in place:** Never create `*_v2`, `*_old`, `*_test2`, `*.bak`, `*.dll.old`, or extra root folders/docs (`docs/`). Put agent guides in `.agents/skills/`.
- **Temp & data:** Put one-off scripts in `scratch/` (delete after use) and datasets/CSVs/`.npz`/`.db` in `data/` or `scratch/`, never source folders.
- **Directory map:**
  - `1_launchers/`: Human-facing GUIs & simulation launchers only.
  - `2_pi4/`, `3_esp32/`, `4_stm32/`: Target runtime/firmware and hardware benches.
  - `5_webots/`: Simulation (`controllers/` 1:1 with `worlds/`, no helper clutter in `controllers/`), GA, assets.
  - `brain/`: Shared planner, `vehicle_config.json`, and synced `vehicle_params.py` (sync via `python tools/sync_params.py`).
  - `tools/`: CLI/AI utilities and unit tests.
