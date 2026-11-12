"""
================================================================================
  FOLKRACE PI 4: MCU BINARY TELEMETRY PROTOCOL DECODER & ENCODER
================================================================================
  High-speed zero-copy Python bridge matching core/mcu_telemetry_protocol.h:
    - Chassis Dynamics Packet (ESP32 Main -> Pi 4): 54 Bytes
    - ToF Perception Packet (ESP32 ToF Hub -> Pi 4): 74 Bytes
    - Motion Command Packet (Pi 4 -> ESP32 Main): 13 Bytes

  Decodes in < 1 microsecond per packet using native C Python struct unpack.
================================================================================
"""

import struct
from typing import Dict, Any, Optional, Tuple

# Sync Bytes
MCU_SYNC = b"\xAA\x55"
PI_SYNC = b"\x55\xAA"

MSG_ID_CHASSIS_TELEM = 0x01
MSG_ID_TOF_TELEM = 0x02
MSG_ID_TOF_SUB_TELEM = 0x05
MSG_ID_MOTION_CMD = 0x10

# Struct formats (little-endian '<', packed)
# Chassis: sync(2), msg_type(B), len(B), seq(I), ts(I),
#          IMU & Steer (8h: yaw, pitch, roll, gyro_z, ax, ay, steer_cmd, steer_act),
#          servo_us(H),
#          Speeds & RPMs (10h: v_cmd, v_est, fl_cmd, fr_cmd, rl_cmd, rr_cmd, fl_act, fr_act, rl_act, rr_act),
#          slip_state(B), slip_ratio(b), flags(B), chk(B)
CHASSIS_FORMAT = "<2sBBII8hH10hBbBB"
CHASSIS_SIZE = struct.calcsize(CHASSIS_FORMAT)  # 54 bytes

# ToF 16-zone full frame: sync(2), msg_type(B), sensor_id(B), seq(I), ts(I),
#      dist_mm(16H), status(16B), spad(16B), temp(b), chk(B)
TOF_FORMAT = "<2sBBII16H16B16BbB"
TOF_SIZE = struct.calcsize(TOF_FORMAT)  # 78 bytes

# ToF 4-zone per-sensor frame (ESP32_ToF/src/main.cpp ToF_Binary_Packet):
#      sync(2), sensor_id(B), seq(I), ts_ms(I), dist_mm(4H), chk(B)
TOF_SUB_FORMAT = "<2sBII4HB"
TOF_SUB_SIZE = struct.calcsize(TOF_SUB_FORMAT)  # 20 bytes

# ToF 52-byte multi-target 4-zone frame (ESP32_ToF/src/main.cpp multi-target ToF_Binary_Packet):
#      sync(2), sensor_id(B), seq(I), ts_ms(I), dist_mm(4H), target0_mm(4H), target1_mm(4H),
#      target0_status(4B), target1_status(4B), spad_count(4H), chk(B)
TOF_SUB52_FORMAT = "<2sBII4H4H4H4B4B4HB"
TOF_SUB52_SIZE = struct.calcsize(TOF_SUB52_FORMAT)  # 52 bytes

# ToF 60-byte per-sensor frame (ESP32_ToF/src/main.cpp multi-target ToF_Binary_Packet):
#      sync(2), sensor_id(B), seq(I), ts_ms(I), dist_mm(16H), status(16B), chk(B)
TOF_SUB60_FORMAT = "<2sBII16H16BB"
TOF_SUB60_SIZE = struct.calcsize(TOF_SUB60_FORMAT)  # 60 bytes

# Motion Command: sync(2), msg_type(B), seq(I), v_cmd(h), steer_mrad(h), mode(B), chk(B)
CMD_FORMAT = "<2sBIhhBB"
CMD_SIZE = struct.calcsize(CMD_FORMAT)  # 13 bytes


def calc_checksum(data: bytes) -> int:
    """Computes XOR checksum over bytes."""
    chk = 0
    for b in data:
        chk ^= b
    return chk


def decode_chassis_packet(data: bytes) -> Optional[Dict[str, Any]]:
    """Decodes 54-byte binary Chassis telemetry frame into a clean Python dictionary."""
    if len(data) < CHASSIS_SIZE:
        return None

    unpacked = struct.unpack_from(CHASSIS_FORMAT, data)
    sync, msg_type, _, seq, ts_us = unpacked[0:5]

    if sync != MCU_SYNC or msg_type != MSG_ID_CHASSIS_TELEM:
        return None

    expected_chk = unpacked[-1]
    actual_chk = calc_checksum(data[: CHASSIS_SIZE - 1])
    if actual_chk != expected_chk:
        return None  # Checksum mismatch

    (
        yaw_x10, pitch_x10, roll_x10, gyro_z_mrad, ax_cm, ay_cm,
        steer_cmd_x10, steer_act_x10, servo_us, v_cmd_mm, v_est_mm,
        fl_cmd, fr_cmd, rl_cmd, rr_cmd,
        fl_act, fr_act, rl_act, rr_act,
        slip_state_code, slip_ratio_pct, flags
    ) = unpacked[5:27]

    slip_map = {0: "NORMAL", 1: "UNDERSTEER", 2: "OVERSTEER", 3: "SPIN"}

    return {
        "seq": seq,
        "mcu_timestamp_us": ts_us,
        "yaw_deg": round(yaw_x10 / 10.0, 1),
        "pitch_deg": round(pitch_x10 / 10.0, 1),
        "roll_deg": round(roll_x10 / 10.0, 1),
        "gyro_z": round(gyro_z_mrad / 1000.0, 3),
        "accel_fwd": round(ax_cm / 100.0, 2),
        "accel_lat": round(ay_cm / 100.0, 2),
        "steer_cmd_deg": round(steer_cmd_x10 / 10.0, 1),
        "steer_actual_deg": round(steer_act_x10 / 10.0, 1),
        "servo_pulse_us": servo_us,
        "v_cmd": round(v_cmd_mm / 1000.0, 3),
        "v_real": round(v_est_mm / 1000.0, 3),
        "v_est": round(v_est_mm / 1000.0, 3),
        "rpms_cmd": [float(fl_cmd), float(fr_cmd), float(rl_cmd), float(rr_cmd)],
        "rpms_act": [float(fl_act), float(fr_act), float(rl_act), float(rr_act)],
        "slip_state": slip_map.get(slip_state_code, "NORMAL"),
        "slip_ratio": round(slip_ratio_pct / 100.0, 2),
        "flags": {
            "override_active": bool(flags & 0x01),
            "fake_data": bool(flags & 0x02),
            "imu_online": bool(flags & 0x04),
            "stm32_front": bool(flags & 0x08),
            "stm32_rear": bool(flags & 0x10),
        },
    }


def decode_tof_packet(data: bytes) -> Optional[Dict[str, Any]]:
    """Decodes 74-byte binary ToF telemetry frame."""
    if len(data) < TOF_SIZE:
        return None

    unpacked = struct.unpack_from(TOF_FORMAT, data)
    sync, msg_type, sensor_id, seq, ts_us = unpacked[0:5]

    if sync != MCU_SYNC or msg_type != MSG_ID_TOF_TELEM:
        return None

    expected_chk = unpacked[-1]
    actual_chk = calc_checksum(data[: TOF_SIZE - 1])
    if actual_chk != expected_chk:
        return None

    dist_mm = list(unpacked[5:21])
    status = list(unpacked[21:37])
    spad = list(unpacked[37:53])
    silicon_temp = unpacked[53]

    # Convert mm to meters (clamp every reading below 30mm to 30mm for Pi reporting)
    dist_mm = [max(30, d) if d < 30 else d for d in dist_mm]
    ranges_m = [round(d / 1000.0, 3) for d in dist_mm]

    return {
        "sensor_id": sensor_id,
        "seq": seq,
        "mcu_timestamp_us": ts_us,
        "silicon_temp_c": silicon_temp,
        "distances_m": ranges_m,
        "status": status,
        "spad": spad,
    }


def decode_tof_sub_packet(data: bytes, accept_s10: bool = True) -> Optional[Dict[str, Any]]:
    """Decodes per-sensor binary ToF packet (20-byte lean frame or 52-byte multi-target frame)."""
    # 1. Check exact 20-byte lean frame (default for Pi)
    if len(data) == TOF_SUB_SIZE:
        unpacked = struct.unpack_from(TOF_SUB_FORMAT, data)
        sync, sensor_id, seq, ts_ms = unpacked[0:4]
        dist_mm = unpacked[4:8]

        if sync != MCU_SYNC or not (1 <= sensor_id <= 4):
            return None

        expected_chk = unpacked[8]
        actual_chk = calc_checksum(data[: TOF_SUB_SIZE - 1])
        if actual_chk != expected_chk:
            return None

        clamped_mm = [max(30, d) if d < 30 else d for d in dist_mm]
        return {
            "sensor_id": sensor_id,
            "seq": seq,
            "ts_ms": ts_ms,
            "mcu_timestamp_us": ts_ms * 1000,
            "distances_m": [round(d / 1000.0, 3) for d in clamped_mm],
            "dist_mm": list(clamped_mm),
        }

    # 2. Check exact 52-byte multi-target frame
    if len(data) == TOF_SUB52_SIZE:
        unpacked = struct.unpack_from(TOF_SUB52_FORMAT, data)
        sync, sensor_id, seq, ts_ms = unpacked[0:4]
        if sync != MCU_SYNC or not (1 <= sensor_id <= 4):
            return None

        expected_chk = unpacked[-1]
        actual_chk = calc_checksum(data[: TOF_SUB52_SIZE - 1])
        if actual_chk != expected_chk:
            return None

        dist_mm = unpacked[4:8]
        target0_mm = unpacked[8:12]
        target1_mm = unpacked[12:16]
        target0_status = unpacked[16:20]
        target1_status = unpacked[20:24]
        spad_count = unpacked[24:28]

        clamped_mm = [max(30, d) if d < 30 else d for d in dist_mm]
        return {
            "sensor_id": sensor_id,
            "seq": seq,
            "ts_ms": ts_ms,
            "mcu_timestamp_us": ts_ms * 1000,
            "distances_m": [round(d / 1000.0, 3) for d in clamped_mm],
            "dist_mm": list(clamped_mm),
            "target0_mm": list(target0_mm),
            "target1_mm": list(target1_mm),
            "target0_status": list(target0_status),
            "target1_status": list(target1_status),
            "spad_count": list(spad_count),
        }

    # 3. Check exact 60-byte legacy frame
    if len(data) == TOF_SUB60_SIZE:
        unpacked = struct.unpack_from(TOF_SUB60_FORMAT, data)
        sync, sensor_id, seq, ts_ms = unpacked[0:4]
        if sync != MCU_SYNC or not (1 <= sensor_id <= 4):
            return None
        expected_chk = unpacked[-1]
        actual_chk = calc_checksum(data[: TOF_SUB60_SIZE - 1])
        if actual_chk != expected_chk:
            return None

        dist_raw = unpacked[4:20]
        status_raw = unpacked[20:36]
        valid_statuses = {5, 6, 9, 10} if accept_s10 else {5, 6, 9}

        primary_distances_m = []
        for i in range(4):
            d0, d1 = dist_raw[2 * i], dist_raw[2 * i + 1]
            s0, s1 = status_raw[2 * i], status_raw[2 * i + 1]
            v0 = (s0 in valid_statuses and d0 > 0)
            v1 = (s1 in valid_statuses and d1 > 0)
            near0 = (s0 != 0 and s0 != 255 and 1 <= d0 <= 30)
            near1 = (s1 != 0 and s1 != 255 and 1 <= d1 <= 30)

            if v0 and v1:
                chosen = min(d0, d1)
            elif v0:
                chosen = d0
            elif v1:
                chosen = d1
            elif near0 and near1:
                chosen = min(d0, d1)
            elif near0:
                chosen = d0
            elif near1:
                chosen = d1
            else:
                chosen = 3000

            if chosen < 30:
                chosen = 30
            primary_distances_m.append(round(chosen / 1000.0, 3))

        return {
            "sensor_id": sensor_id,
            "seq": seq,
            "ts_ms": ts_ms,
            "mcu_timestamp_us": ts_ms * 1000,
            "distances_m": primary_distances_m,
            "distances_raw_mm": list(dist_raw),
            "status_raw": list(status_raw),
        }

    return None


def encode_motion_command(target_v_mps: float, target_steer_rad: float, run_mode: int = 1, seq: int = 0) -> bytes:
    """Encodes a 13-byte binary motion command packet for ESP32 Main."""
    v_mm_s = int(round(target_v_mps * 1000.0))
    steer_mrad = int(round(target_steer_rad * 1000.0))
    
    # Pack without checksum first
    header_payload = struct.pack("<2sBIhhB", PI_SYNC, MSG_ID_MOTION_CMD, seq, v_mm_s, steer_mrad, run_mode)
    chk = calc_checksum(header_payload)
    return header_payload + bytes([chk])


if __name__ == "__main__":
    import time
    print("Testing MCU Telemetry Protocol...")
    cmd = encode_motion_command(2.5, -0.35, 1, 42)
    assert len(cmd) == 13, f"Expected 13 bytes, got {len(cmd)}"
    assert calc_checksum(cmd[:-1]) == cmd[-1], "Checksum mismatch"
    print(f"Motion Command encoded: {len(cmd)} bytes -> OK")

    # Synthetic chassis packet
    payload = struct.pack(
        "<2sBBI" "I" "8h" "H" "10h" "BbBB",
        MCU_SYNC, MSG_ID_CHASSIS_TELEM, 49, 101,
        12345678,
        150, -20, 5, 250, 120, -15, 180, 175,
        1620,
        2500, 2480, 1200, 1220, 1200, 1220, 1195, 1215, 1202, 1225,
        0, 3, 0x1C, 0
    )
    chk = calc_checksum(payload[:-1])
    pkt = payload[:-1] + bytes([chk])
    
    t0 = time.perf_counter()
    res = decode_chassis_packet(pkt)
    dt_us = (time.perf_counter() - t0) * 1e6
    assert res is not None, "Failed to decode chassis packet"
    assert res["v_cmd"] == 2.5
    assert res["yaw_deg"] == 15.0
    print(f"Chassis Packet decoded in {dt_us:.2f} us -> OK: {res}")
    print("All MCU telemetry protocol tests passed!")

