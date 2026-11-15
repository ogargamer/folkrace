#!/usr/bin/env python3
"""Run a high-rate STM32 FOC trace while the ESP32 commands the motors."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import time
from pathlib import Path

import serial


TRACE_HEADER = struct.Struct("<2sHHHII")
TRACE_SAMPLE = struct.Struct("<HHhhhhhh")
TRACE_SYNC = b"FT"
MOTOR_POLE_PAIRS = 11
RAW_COUNTS = 16384


def read_trace(port: serial.Serial) -> tuple[dict[str, int], list[tuple[int, ...]]]:
    data = bytearray()
    deadline = time.monotonic() + 8.0
    expected = None
    while time.monotonic() < deadline:
        chunk = port.read(4096)
        if chunk:
            data.extend(chunk)
        start = data.find(TRACE_SYNC)
        if start >= 0 and len(data) >= start + TRACE_HEADER.size:
            header = TRACE_HEADER.unpack_from(data, start)
            count = header[3]
            expected = start + TRACE_HEADER.size + count * TRACE_SAMPLE.size
            if len(data) >= expected:
                payload = data[start + TRACE_HEADER.size : expected]
                samples = [
                    TRACE_SAMPLE.unpack_from(payload, offset)
                    for offset in range(0, len(payload), TRACE_SAMPLE.size)
                ]
                return (
                    {
                        "version": header[1],
                        "sample_hz": header[2],
                        "sample_count": count,
                        "overwritten": header[4],
                    },
                    samples,
                )
    raise TimeoutError(f"trace dump incomplete; expected_end={expected}, received={len(data)}")


def unwrap_delta(delta: int) -> int:
    half = RAW_COUNTS // 2
    return ((delta + half) % RAW_COUNTS) - half


def motor_analysis(samples: list[tuple[int, ...]], motor_index: int) -> dict[str, object]:
    raw_index = motor_index
    target_index = 2 + motor_index
    actual_index = 4 + motor_index
    uq_index = 6 + motor_index
    raw = [sample[raw_index] for sample in samples]
    deltas = [unwrap_delta(b - a) for a, b in zip(raw, raw[1:])]
    electrical_steps = [
        delta * (2.0 * math.pi * MOTOR_POLE_PAIRS / RAW_COUNTS) for delta in deltas
    ]

    actual = [sample[actual_index] for sample in samples]
    target = [sample[target_index] for sample in samples]
    uq = [sample[uq_index] for sample in samples]
    actual_steps = [abs(b - a) for a, b in zip(actual, actual[1:])]
    phase_jumps = [abs(step) for step in electrical_steps]
    phase_tracking_errors = [
        measured - (
            target[index]
            * (2.0 * math.pi / 60.0)
            * MOTOR_POLE_PAIRS
            / 500.0
        )
        for index, measured in enumerate(electrical_steps)
    ]
    return {
        "target_rpm_avg": round(statistics.mean(target), 1),
        "actual_rpm_avg": round(statistics.mean(actual), 1),
        "actual_rpm_minmax": [min(actual), max(actual)],
        "uq_mv_max_abs": max(abs(value) for value in uq),
        "max_raw_delta_counts": max(abs(value) for value in deltas),
        "max_electrical_step_rad": round(max(phase_jumps), 3),
        "electrical_steps_over_pi": sum(value > math.pi for value in phase_jumps),
        "max_phase_tracking_error_rad": round(max(abs(value) for value in phase_tracking_errors), 3),
        "phase_tracking_errors_over_1rad": sum(
            abs(value) > 1.0 for value in phase_tracking_errors
        ),
        "actual_steps_over_40rpm": sum(value > 40 for value in actual_steps),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stm-port", default="COM13")
    parser.add_argument("--esp-port", default="COM5")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--speed", type=float, default=-0.8)
    parser.add_argument("--steer-angle", type=int, default=90)
    parser.add_argument("--output", type=Path, default=Path("stm32_foc_trace.bin"))
    args = parser.parse_args()

    with serial.Serial(args.stm_port, 921600, timeout=0.01) as stm, serial.Serial(
        args.esp_port, 921600, timeout=0.005
    ) as esp:
        stm.reset_input_buffer()
        esp.reset_input_buffer()
        stm.write(b"TRACE_START\n")
        stm.flush()
        time.sleep(0.1)
        try:
            for command in (
                b"MOTORS\n",
                f"A{args.steer_angle}\n".encode(),
                f"V {args.speed}\n".encode(),
            ):
                esp.write(command)
                esp.flush()
                time.sleep(0.15)
            time.sleep(args.duration)
        finally:
            esp.write(b"ESTOP\n")
            esp.flush()
            time.sleep(0.5)
            stm.write(b"TRACE_STOP\n")
            stm.flush()
            time.sleep(0.1)
            stm.reset_input_buffer()
            stm.write(b"TRACE_DUMP\n")
            stm.flush()
        header, samples = read_trace(stm)

    raw = b"".join(TRACE_SAMPLE.pack(*sample) for sample in samples)
    args.output.write_bytes(TRACE_HEADER.pack(TRACE_SYNC, header["version"], header["sample_hz"], header["sample_count"], header["overwritten"], 0) + raw)
    result = {
        "trace_file": str(args.output),
        "header": header,
        "motor1": motor_analysis(samples, 0),
        "motor2": motor_analysis(samples, 1),
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
