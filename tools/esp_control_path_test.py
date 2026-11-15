#!/usr/bin/env python3
"""Exercise and diagnose the direct ESP32 control path.

The test captures the ESP32's 100 Hz chassis telemetry directly, rather than
through the Pi dashboard. It sends a centered, constant-speed command by
default, records sequence/timestamp continuity, and stops the motors in a
finally block.
"""

from __future__ import annotations

import argparse
import json
import statistics
import struct
import time
from dataclasses import dataclass
from typing import Optional

import serial


BAUD = 921_600
CHASSIS_FORMAT = "<2sBBII8hH10hBbBB"
CHASSIS_SIZE = struct.calcsize(CHASSIS_FORMAT)
SYNC = b"\xAA\x55"


def checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


@dataclass
class Sample:
    sequence: int
    timestamp_us: int
    v_cmd: float
    v_est: float
    command_rpm: tuple[int, int, int, int]
    actual_rpm: tuple[int, int, int, int]
    flags: int


def decode_frames(buffer: bytearray) -> list[Sample]:
    samples: list[Sample] = []
    while True:
        start = buffer.find(SYNC)
        if start < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            break
        if start:
            del buffer[:start]
        if len(buffer) < CHASSIS_SIZE:
            break

        frame = bytes(buffer[:CHASSIS_SIZE])
        if checksum(frame[:-1]) != frame[-1]:
            del buffer[:2]
            continue

        unpacked = struct.unpack(CHASSIS_FORMAT, frame)
        samples.append(
            Sample(
                sequence=unpacked[3],
                timestamp_us=unpacked[4],
                v_cmd=unpacked[14] / 1000.0,
                v_est=unpacked[15] / 1000.0,
                command_rpm=tuple(unpacked[16:20]),
                actual_rpm=tuple(unpacked[20:24]),
                flags=unpacked[26],
            )
        )
        del buffer[:CHASSIS_SIZE]
    return samples


def read_samples(port: serial.Serial, duration: float) -> list[Sample]:
    deadline = time.monotonic() + duration
    buffer = bytearray()
    samples: list[Sample] = []
    while time.monotonic() < deadline:
        data = port.read(4096)
        if data:
            buffer.extend(data)
            samples.extend(decode_frames(buffer))
    return samples


def percentile(values: list[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    return statistics.quantiles(values, n=100)[int(fraction * 100) - 1] if len(values) > 1 else values[0]


def summarize(samples: list[Sample]) -> dict[str, object]:
    if not samples:
        return {"frames": 0}

    sequence_gaps = [
        (current.sequence - previous.sequence) & 0xFFFFFFFF
        for previous, current in zip(samples, samples[1:])
    ]
    timestamp_steps = [
        current.timestamp_us - previous.timestamp_us
        for previous, current in zip(samples, samples[1:])
    ]
    rear_actual = [sample.actual_rpm[2:4] for sample in samples]
    rear_command = [sample.command_rpm[2:4] for sample in samples]
    actual_steps = [
        max(abs(current[0] - previous[0]), abs(current[1] - previous[1]))
        for previous, current in zip(rear_actual, rear_actual[1:])
    ]
    settled_actual = rear_actual[len(rear_actual) // 5 :]
    settled_command = rear_command[len(rear_command) // 5 :]

    return {
        "frames": len(samples),
        "sequence_gap_counts": {
            str(gap): sequence_gaps.count(gap) for gap in sorted(set(sequence_gaps))
        },
        "missing_sequences": sum(max(0, gap - 1) for gap in sequence_gaps),
        "timestamp_us_p50": percentile([float(step) for step in timestamp_steps], 0.5),
        "timestamp_us_max": max(timestamp_steps) if timestamp_steps else None,
        "rear_command_avg": [
            round(statistics.mean(pair[index] for pair in settled_command), 1)
            for index in (0, 1)
        ],
        "rear_actual_avg": [
            round(statistics.mean(pair[index] for pair in settled_actual), 1)
            for index in (0, 1)
        ],
        "rear_actual_minmax": [
            [min(pair[index] for pair in settled_actual), max(pair[index] for pair in settled_actual)]
            for index in (0, 1)
        ],
        "max_actual_step_rpm": max(actual_steps) if actual_steps else 0,
        "steps_over_40rpm": sum(step > 40 for step in actual_steps),
        "settled_zero_samples": sum(
            left == 0 and right == 0 for left, right in settled_actual
        ),
        "rear_link_online_samples": sum(bool(sample.flags & 0x10) for sample in samples),
        "last_sample": samples[-1].__dict__,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--speed", type=float, default=-0.8)
    parser.add_argument("--steer-angle", type=int, default=90)
    args = parser.parse_args()

    with serial.Serial(args.port, BAUD, timeout=0.005) as port:
        port.reset_input_buffer()
        try:
            for command in (
                b"MOTORS\n",
                f"A{args.steer_angle}\n".encode(),
                f"V {args.speed}\n".encode(),
            ):
                port.write(command)
                port.flush()
                time.sleep(0.15)
            samples = read_samples(port, args.duration)
        finally:
            port.write(b"ESTOP\n")
            port.flush()
            time.sleep(0.5)

    print(json.dumps(summarize(samples), indent=2))


if __name__ == "__main__":
    main()
