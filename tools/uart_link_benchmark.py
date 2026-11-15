#!/usr/bin/env python3
"""Motor-free performance benchmark for the ESP32 <-> STM32 UART protocol.

The default benchmark uses synthetic byte streams and does not open a serial
port or command a motor. It compares the old unbounded parser behavior with
the bounded/latest-command-wins behavior used by the firmware.

Examples:
    python tools/uart_link_benchmark.py
    python tools/uart_link_benchmark.py --scenario steady --duration 5
    python tools/uart_link_benchmark.py --live-port COM7 --duration 10
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from typing import Optional


BAUD = 921_600
COMMAND_HZ = 400
CONTROL_PERIOD_US = 2_500
COMMAND_SIZE = 12
FEEDBACK_SIZE = 13
DEFAULT_BYTE_BUDGET = 64


def xor_checksum(data: bytes) -> int:
    result = 0
    for byte in data:
        result ^= byte
    return result


def make_command(sequence: int) -> bytes:
    payload = struct.pack("<Bff", 0, float(sequence), float(-sequence))
    return b"\xAA\x55" + payload + bytes((xor_checksum(payload),))


def make_feedback(sequence: int) -> bytes:
    payload = struct.pack("<BffB", 0, float(sequence), float(-sequence), 0)
    return b"\xAA\x55" + payload + bytes((xor_checksum(payload),))


@dataclass
class Metrics:
    received_packets: int = 0
    applied_packets: int = 0
    replies: int = 0
    checksum_errors: int = 0
    service_calls: int = 0
    max_bytes_per_call: int = 0
    max_packets_per_call: int = 0
    max_backlog: int = 0
    total_bytes_processed: int = 0


class CommandParserModel:
    """Reference model of the STM32 command receiver."""

    def __init__(self, bounded: bool, byte_budget: int = DEFAULT_BYTE_BUDGET) -> None:
        self.bounded = bounded
        self.byte_budget = byte_budget
        self.buffer = bytearray()
        self.metrics = Metrics()
        self.applied_sequences: list[int] = []

    def feed(self, data: bytes) -> None:
        self.buffer.extend(data)

    def service(self) -> None:
        before = len(self.buffer)
        limit = self.byte_budget if self.bounded else len(self.buffer)
        consumed = 0
        packets = 0
        latest_sequence: Optional[int] = None

        while self.buffer and consumed < limit:
            if len(self.buffer) < 2:
                break
            if self.buffer[:2] != b"\xAA\x55":
                del self.buffer[0]
                consumed += 1
                continue
            if len(self.buffer) < COMMAND_SIZE or consumed + COMMAND_SIZE > limit:
                break

            frame = bytes(self.buffer[:COMMAND_SIZE])
            del self.buffer[:COMMAND_SIZE]
            consumed += COMMAND_SIZE
            payload = frame[2:-1]
            if xor_checksum(payload) != frame[-1]:
                self.metrics.checksum_errors += 1
                continue

            sequence = int(struct.unpack("<f", payload[1:5])[0])
            self.metrics.received_packets += 1
            packets += 1
            latest_sequence = sequence

        if latest_sequence is not None:
            if self.bounded:
                self.applied_sequences.append(latest_sequence)
                self.metrics.applied_packets += 1
                self.metrics.replies += 1
            else:
                self.metrics.applied_packets += packets
                self.metrics.replies += packets
                self.applied_sequences.extend(
                    range(latest_sequence - packets + 1, latest_sequence + 1)
                )

        self.metrics.service_calls += 1
        self.metrics.total_bytes_processed += consumed
        self.metrics.max_bytes_per_call = max(self.metrics.max_bytes_per_call, consumed)
        self.metrics.max_packets_per_call = max(self.metrics.max_packets_per_call, packets)
        self.metrics.max_backlog = max(self.metrics.max_backlog, len(self.buffer))
        if before == len(self.buffer) and self.buffer and consumed == 0:
            raise RuntimeError("parser made no progress on a complete synthetic frame")


def run_synthetic(duration: float, burst_packets: int, bounded: bool) -> Metrics:
    parser = CommandParserModel(bounded=bounded)
    ticks = int(duration * COMMAND_HZ)
    sequence = 0
    for _ in range(ticks):
        for _ in range(burst_packets):
            parser.feed(make_command(sequence))
            sequence += 1
        parser.service()
    return parser.metrics


def format_metrics(name: str, metrics: Metrics) -> str:
    return (
        f"{name}: received={metrics.received_packets} "
        f"applied={metrics.applied_packets} replies={metrics.replies} "
        f"max_service_bytes={metrics.max_bytes_per_call} "
        f"max_service_packets={metrics.max_packets_per_call} "
        f"max_backlog_bytes={metrics.max_backlog} "
        f"checksum_errors={metrics.checksum_errors}"
    )


def run_live(port: str, duration: float) -> None:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("Live mode requires pyserial; synthetic mode does not.") from exc

    ser = serial.Serial(port=port, baudrate=BAUD, timeout=0)
    deadline = time.monotonic() + duration
    received = 0
    bytes_read = 0
    max_backlog = 0
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        max_backlog = max(max_backlog, waiting)
        data = ser.read(waiting or 1)
        bytes_read += len(data)
        received += data.count(b"\xAA\x55")
        time.sleep(0.001)
    ser.close()
    print(
        f"live: port={port} duration_s={duration:.2f} bytes={bytes_read} "
        f"header_candidates={received} max_os_backlog_bytes={max_backlog}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--scenario", choices=("steady", "burst"), default="burst")
    parser.add_argument("--live-port", help="Observe a live feedback UART without transmitting.")
    args = parser.parse_args()

    if args.live_port:
        run_live(args.live_port, args.duration)
        return

    burst_packets = 1 if args.scenario == "steady" else 8
    before = run_synthetic(args.duration, burst_packets, bounded=False)
    after = run_synthetic(args.duration, burst_packets, bounded=True)
    print(
        f"synthetic: duration_s={args.duration:.2f} scenario={args.scenario} "
        f"burst_packets={burst_packets} byte_budget={DEFAULT_BYTE_BUDGET}"
    )
    print(format_metrics("before", before))
    print(format_metrics("after ", after))


if __name__ == "__main__":
    main()
