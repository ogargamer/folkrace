import serial
import struct
import time
import sys

def xor_check(data):
    chk = 0
    for b in data:
        chk ^= b
    return chk

def set_speed(ser, hz):
    # Packet format:
    # 0xF5A5 (head_flag, 2 bytes: 0xA5, 0xF5)
    # cmd = 0xA1 (SET_ROTATION_SPEED)
    # cmd_type = 0xC1 (WRITE_PARAM)
    # payload_len = 2
    # data[0] = hz & 0xFF
    # data[1] = (hz >> 8) & 0xFF
    # xor_check (of header + payload = 5 + 2 = 7 bytes)
    # 0x31 0xF2 (TAIL_FLAG)
    header = struct.pack("<HBBB", 0xF5A5, 0xA1, 0xC1, 2)
    payload = struct.pack("<H", hz)
    chk = xor_check(header + payload)
    tail = bytes([chk, 0x31, 0xF2])
    pkt = header + payload + tail
    print(f"Sending SET_SPEED({hz} Hz): {pkt.hex()}")
    ser.write(pkt)
    ser.flush()

def test():
    ser = serial.Serial("/dev/ttyAMA4", 230400, timeout=1.0)
    print("Opened /dev/ttyAMA4")
    
    # Read current speed
    time.sleep(1)
    
    # Test setting to 8 Hz
    set_speed(ser, 8)
    time.sleep(2)
    
    # Test setting to 12 Hz
    set_speed(ser, 12)
    time.sleep(2)
    
    # Test setting back to 10 Hz
    set_speed(ser, 10)
    time.sleep(1)
    
    ser.close()

if __name__ == "__main__":
    test()
