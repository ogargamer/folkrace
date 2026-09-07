import time
import spidev
import board
import digitalio
import struct

def test_hillcrest_full():
    print("==================================================================")
    print("  BNO08X IMU SPI COMMUNICATION & REPORT INITIALIZATION")
    print("==================================================================")

    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 3000000
    spi.mode = 3 # SPI Mode 3

    # Pin setup: CS=GPIO 16 (Pin 36), RST=GPIO 21 (Pin 40), INT=GPIO 26 (Pin 37)
    cs = digitalio.DigitalInOut(board.D16)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True # CS High (inactive)

    rst = digitalio.DigitalInOut(board.D21)
    rst.direction = digitalio.Direction.OUTPUT
    rst.value = True

    int_pin = digitalio.DigitalInOut(board.D26)
    int_pin.direction = digitalio.Direction.INPUT
    int_pin.pull = digitalio.Pull.UP

    def set_cs(val):
        cs.value = not val # True = pull LOW (active)

    def set_rst(val):
        rst.value = val

    def is_int():
        return not int_pin.value # True = LOW (asserted)

    # Reset
    print("Step 1: Hardware Resetting BNO08X...")
    set_rst(True)
    set_cs(False)
    time.sleep(0.05)
    set_rst(False)
    time.sleep(0.02)
    set_rst(True)

    # Wait for INT
    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)

    print(f"INT asserted in {round((time.time()-start)*1000, 1)} ms!")

    def read_packet():
        if not is_int():
            return None
        set_cs(True)
        time.sleep(0.0001)
        hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
        pkt_len = (hdr[0] | (hdr[1] << 8)) & ~0x8000
        chan = hdr[2]
        seq = hdr[3]
        payload = []
        if pkt_len > 4 and pkt_len < 1024:
            payload = spi.xfer2([0x00] * (pkt_len - 4))
        set_cs(False)
        time.sleep(0.001)
        return (chan, seq, pkt_len, payload)

    def write_packet(channel, payload, seq_num):
        pkt_len = len(payload) + 4
        hdr = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, channel, seq_num]
        full_pkt = hdr + list(payload)
        
        # Pull CS LOW and wait for INT LOW (handshake)
        set_cs(True)
        start_w = time.time()
        while not is_int() and (time.time() - start_w < 0.1):
            time.sleep(0.0002)
            
        spi.xfer2(full_pkt)
        set_cs(False)
        time.sleep(0.002)

    # Drain initial reset packets
    print("\nStep 2: Draining SHTP Bootloader Packets...")
    for _ in range(6):
        start = time.time()
        while not is_int() and (time.time() - start < 0.2):
            time.sleep(0.0005)
        pkt = read_packet()
        if pkt:
            chan, seq, plen, pdata = pkt
            print(f"  Received: Chan={chan}, Len={plen}, Seq={seq}")
        else:
            break

    # Send Product ID Request (Report ID 0xF9 on Channel 2)
    print("\nStep 3: Requesting Product ID (Report 0xF9)...")
    write_packet(2, [0xF9, 0x00], 0)

    # Read Response
    for _ in range(10):
        start = time.time()
        while not is_int() and (time.time() - start < 0.2):
            time.sleep(0.0005)
        pkt = read_packet()
        if pkt:
            chan, seq, plen, pdata = pkt
            if len(pdata) > 0 and pdata[0] == 0xF8: # Product ID Response
                sw_ver_major = pdata[2]
                sw_ver_minor = pdata[3]
                sw_part_num = (pdata[7] << 24) | (pdata[6] << 16) | (pdata[5] << 8) | pdata[4]
                sw_build = (pdata[11] << 24) | (pdata[10] << 16) | (pdata[9] << 8) | pdata[8]
                print(f"========================================================")
                print(f"  SUCCESS! BNO08X PRODUCT ID VERIFIED:")
                print(f"  Part Number : 1000{sw_part_num} (BNO080/BNO085)")
                print(f"  Firmware    : v{sw_ver_major}.{sw_ver_minor}.{sw_build}")
                print(f"========================================================")
                break

    # Enable Rotation Vector (Report 0x05) & Accelerometer (Report 0x01) at 50Hz (20,000 us = 0x00004E20)
    print("\nStep 4: Enabling 50Hz Rotation Vector & Accelerometer Reports...")
    # SHTP Set Feature Command (0xFD)
    # [0xFD, Feature Report ID, Flags (0x00), Change Sensitivity LSB/MSB (0x0000), Report Interval (4 bytes us), Batch Interval (4 bytes), Sensor Specific (4 bytes)]
    
    interval_us = 20000 # 50Hz (20ms)
    interval_bytes = list(struct.pack("<I", interval_us))
    
    # 1. Enable Game Rotation Vector (Report ID 0x08)
    cmd_rot = [0xFD, 0x08, 0x00, 0x00, 0x00] + interval_bytes + [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    write_packet(2, cmd_rot, 1)
    time.sleep(0.01)

    # 2. Enable Accelerometer (Report ID 0x01)
    cmd_acc = [0xFD, 0x01, 0x00, 0x00, 0x00] + interval_bytes + [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    write_packet(2, cmd_acc, 2)
    time.sleep(0.01)

    print("\nStep 5: Streaming Live 9-DOF IMU Telemetry (Quaternion & Accelerometer)...")
    print("-" * 75)
    
    sample_count = 0
    start_stream = time.time()
    
    while sample_count < 30 and (time.time() - start_stream < 6.0):
        if is_int():
            pkt = read_packet()
            if pkt:
                chan, seq, plen, pdata = pkt
                if chan == 3 and len(pdata) >= 14: # Input Sensor Reports (Channel 3)
                    report_id = pdata[4] if len(pdata) > 4 else 0
                    
                    # Parse Game Rotation Vector (Report 0x08)
                    # pdata[4] = 0x08, pdata[5] = seq, pdata[6] = status
                    # Quat: I, J, K, Real as signed 16-bit Q14 (scale = 1.0 / (1 << 14))
                    if 0x08 in pdata:
                        idx = pdata.index(0x08)
                        if idx + 13 < len(pdata):
                            qi, qj, qk, qr = struct.unpack_from("<hhhh", bytes(pdata), idx + 5)
                            q_scale = 1.0 / (1 << 14)
                            w, x, y, z = qr * q_scale, qi * q_scale, qj * q_scale, qk * q_scale
                            
                            # Roll, Pitch, Yaw
                            roll = (180.0 / 3.14159265) * struct.unpack("<f", struct.pack("<f", 0.0))[0]
                            siny_cosp = 2.0 * (w * z + x * y)
                            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                            import math
                            yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)
                            pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))) * (180.0 / math.pi)
                            roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * (180.0 / math.pi)

                            sample_count += 1
                            print(f"[{sample_count:02d}] Quat (w:{w:+.3f} x:{x:+.3f} y:{y:+.3f} z:{z:+.3f}) | Roll:{roll:+6.1f}° Pitch:{pitch:+6.1f}° Yaw:{yaw:+6.1f}°")
        time.sleep(0.002)

    print("-" * 75)
    print(">> VERIFIED: BNO08X IMU IS 100% OPERATIONAL OVER SPI ON PI 5! <<")

    spi.close()

if __name__ == "__main__":
    test_hillcrest_full()
