---
name: ai-flight-debug
description: Analyze Folkrace Pi 4 SQLite flight recordings (.db) for crashes, anomalies, planner/ToF/MCU diagnostics, A/B run comparisons, and ML/AI data export.
---

# AI Flight Debugging Suite

Use `launchers/20_ai_debug_suite.py` instead of writing ad-hoc SQLite/zlib scripts (supports both v1 and v2 `.db` files in `Downloads/` and `data/recordings/`).

## Workflow
1. Run `digest` first as a triage index (diagnostics, KPIs, top episodes, deduplicated MCU logs).
2. Drill into incidents with `episodes`, then `window` starting **1–2s before** the episode to see upstream cause, then `frame` for full sensors + 2D ASCII ToF/MPC radar.
3. Use `compare` for A/B run regressions, `query` for custom filters, `tag` to label frame ranges in the `.db`, and `export` for `report.md`/`digest.json`/`frames.csv`/`arrays.npz`.

## Commands
```powershell
python launchers/20_ai_debug_suite.py list
python launchers/20_ai_debug_suite.py digest [db]                 # omit [db] for latest
python launchers/20_ai_debug_suite.py episodes [db] [--category SHIELD|STALL|LINK_DROP]
python launchers/20_ai_debug_suite.py window [db] <start> <end> [--by-time] [--stride N]
python launchers/20_ai_debug_suite.py frame [db] <frame_id>       # full state + ASCII radar
python launchers/20_ai_debug_suite.py query [db] --shield --max-tof 0.25 --limit 20
python launchers/20_ai_debug_suite.py compare <db_a> <db_b>
python launchers/20_ai_debug_suite.py export [db] [--formats md,json,csv,npz]
python launchers/20_ai_debug_suite.py tag [db] <start> <end> <tag> --note "..."
python launchers/20_ai_debug_suite.py pull --digest               # download from Pi 4 + digest
```

## Folkrace Telemetry & Verification Guardrails
- **Cross-eyed ToF pods**: `tof_front_left_*` (ch 0..7) points right; `tof_front_right_*` (ch 8..15) points left (`0.0` = invalid/offline, `>=1.79` = open/saturated). ASCII radar is ~`5x9.5 cm` quantized and omits `candidate_lines`/costmap inflation; use `frame --json` for exact coordinates.
- **Do not trust `digest` blindly**:
  - Check **wheel tracking MAE (`cmd - act`)**, not just raw RPM lag (track turn direction creates Ackermann speed differences).
  - Check **`override_active`** and **`mcu_logs`** before calling `LINK_DROP`/`EMERGENCY` a crash—manual GUI/bench stops also trigger them.
  - Check `pitch_deg` directly if `is_on_ramp` stays high (an IMU pitch offset `>=5 deg` bypasses the flat-floor warning).
  - Silent tuning issues (slow lap, mild oscillation, poor line choice) may trigger zero anomalies—verify with `window` and `compare`.
