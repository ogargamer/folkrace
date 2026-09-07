import time
import smbus2
import struct
import math

def analyze_raw():
    print("==================================================================")
    print("  BNO08X STATIONARY RAW PACKET ANALYSIS ON RASPBERRY PI 5")
    print("==================================================================")

    bus = smbus2.SMBus(1)
    addr = 0x4B

    # Read 10 raw reports from Channel 3
    print("Capturing 15 raw reports from I2C address 0x4B:")
    print("-" * 80)

    for sample_idx in range(15):
        try:
            msg = smbus2.i2c_msg.read(addr, 64)
            bus.i2c_rdwr(msg)
            raw = list(msg)
            plen = (raw[0] | (raw[1] << 8)) & ~0x8000
            chan = raw[2]
            seq = raw[3]

            if plen > 4:
                payload = raw[4:plen]
                print(f"\n[Sample {sample_idx+1:02d}] Chan:{chan} | TotalLen:{plen} | Seq:{seq}")
                print("  Raw Header:", [f"0x{x:02X}" for x in raw[:4]])
                print("  Payload Bytes (len=" + str(len(payload)) + "):", [f"0x{x:02X}" for x in payload])

                # Let's inspect report IDs inside payload
                # Each SH-2 report starts with:
                # Byte 0: Report ID
                # Byte 1: Sequence Number
                # Byte 2: Status
                # Byte 3: Delay
                # Followed by sensor-specific data!
                # Wait! What if there is a Timebase Base Timestamp report (0xFB) at the start?
                # If byte 0 is 0xFB (Base Timestamp):
                #   Byte 0..4 = [0xFB, time0, time1, time2, time3] (5 bytes total!)
                #   Then the actual report begins at offset 5!
                # If byte 0 is 0x08 (Game Rotation Vector directly):
                #   Byte 0 = 0x08, Byte 1 = seq, Byte 2 = status, Byte 3 = delay
                #   Bytes 4..11 = [i_lsb, i_msb, j_lsb, j_msb, k_lsb, k_msb, r_lsb, r_msb]
                
                offset = 0
                while offset < len(payload):
                    rep_id = payload[offset]
                    print(f"   -> At offset {offset}: Report ID = 0x{rep_id:02X}")
                    if rep_id == 0xFB: # Timebase
                        print(f"      [Timebase Report 0xFB]: 5 bytes long")
                        offset += 5
                    elif rep_id == 0x08: # Game Rotation Vector
                        # Structure: 0x08, seq, status, delay, i(2B), j(2B), k(2B), r(2B)
                        if offset + 12 <= len(payload):
                            i_val, j_val, k_val, r_val = struct.unpack_from("<hhhh", bytes(payload), offset + 4)
                            scale = 1.0 / (1 << 14)
                            qx, qy, qz, qw = i_val * scale, j_val * scale, k_val * scale, r_val * scale
                            print(f"      [Game Rotation Vector 0x08]:")
                            print(f"         Raw Integers: i={i_val}, j={j_val}, k={k_val}, r={r_val}")
                            print(f"         Unit Quat: (qw={qw:+.4f}, qx={qx:+.4f}, qy={qy:+.4f}, qz={qz:+.4f})")
                            mag = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
                            print(f"         Quat Magnitude: {mag:.4f} (should be ~1.0000)")
                            
                            # Euler
                            sinr = 2.0 * (qw*qx + qy*qz)
                            cosr = 1.0 - 2.0 * (qx*qx + qy*qy)
                            roll = math.atan2(sinr, cosr) * 57.2958

                            sinp = 2.0 * (qw*qy - qz*qx)
                            pitch = math.asin(max(-1.0, min(1.0, sinp))) * 57.2958

                            siny = 2.0 * (qw*qz + qx*qy)
                            cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
                            yaw = math.atan2(siny, cosy) * 57.2958
                            print(f"         Euler Angles: ROLL={roll:+6.2f}°, PITCH={pitch:+6.2f}°, YAW={yaw:+6.2f}°")
                            offset += 12
                        else:
                            break
                    elif rep_id == 0x01: # Accelerometer
                        if offset + 10 <= len(payload):
                            ax_raw, ay_raw, az_raw = struct.unpack_from("<hhh", bytes(payload), offset + 4)
                            scale = 1.0 / (1 << 8)
                            print(f"      [Accelerometer 0x01]: raw=({ax_raw},{ay_raw},{az_raw}) -> ({ax_raw*scale:+.2f}, {ay_raw*scale:+.2f}, {az_raw*scale:+.2f}) m/s²")
                            offset += 10
                        else:
                            break
                    elif rep_id == 0x02: # Gyroscope
                        if offset + 10 <= len(payload):
                            gx_raw, gy_raw, gz_raw = struct.unpack_from("<hhh", bytes(payload), offset + 4)
                            scale = 1.0 / (1 << 9)
                            print(f"      [Gyroscope 0x02]: raw=({gx_raw},{gy_raw},{gz_raw}) -> ({gx_raw*scale*57.2958:+.2f}, {gy_raw*scale*57.2958:+.2f}, {gz_raw*scale*57.2958:+.2f}) deg/s")
                            offset += 10
                        else:
                            break
                    else:
                        offset += 1
        except Exception as e:
            print("[Read Error]:", e)
        time.sleep(0.05)

    bus.close()

if __name__ == "__main__":
    analyze_raw()
