import time
import spidev
import gpiod
from gpiod.line import Direction, Value, Bias

def test_raw_bno08x():
    print("Testing BNO08X Hardware Pins & SPI on Raspberry Pi 5...")
    
    # SPI0
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 3000000
    spi.mode = 3 # BNO08X SPI mode 3 (CPOL=1, CPHA=1)
    
    # Let's check with gpiod on chip 4 (RP1 GPIO controller on Pi 5)
    # GPIO 16 (CS), GPIO 26 (INT), GPIO 21 (RST)
    import subprocess
    pinctrl_out = subprocess.check_output("pinctrl get 16,21,26", shell=True).decode()
    print("Pin States:\n", pinctrl_out)
    
    # Send a hard reset sequence: Pull RST (GPIO 21) low for 10ms, then high
    subprocess.call("pinctrl set 21 op dh", shell=True) # RST HIGH
    subprocess.call("pinctrl set 16 op dh", shell=True) # CS HIGH
    subprocess.call("pinctrl set 26 ip pu", shell=True) # INT INPUT with PULL-UP
    time.sleep(0.05)
    
    print("Resetting BNO08X...")
    subprocess.call("pinctrl set 21 op dl", shell=True) # RST LOW
    time.sleep(0.02)
    subprocess.call("pinctrl set 21 op dh", shell=True) # RST HIGH
    time.sleep(0.1)
    
    print("Reading INT pin state after reset:")
    print(subprocess.check_output("pinctrl get 26", shell=True).decode())
    
    # Assert CS (LOW), read 4 bytes SHTP header, then deassert CS (HIGH)
    subprocess.call("pinctrl set 16 op dl", shell=True) # CS LOW
    time.sleep(0.001)
    hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
    subprocess.call("pinctrl set 16 op dh", shell=True) # CS HIGH
    
    print("Received 4-byte SHTP Header from BNO08X:", [hex(x) for x in hdr])
    
    pkt_len = (hdr[0] | (hdr[1] << 8)) & ~0x8000
    channel = hdr[2]
    seq = hdr[3]
    print(f"Packet Length: {pkt_len} bytes | Channel: {channel} | Sequence: {seq}")
    
    if pkt_len > 4 and pkt_len < 512:
        subprocess.call("pinctrl set 16 op dl", shell=True) # CS LOW
        time.sleep(0.001)
        payload = spi.xfer2([0x00] * pkt_len)
        subprocess.call("pinctrl set 16 op dh", shell=True) # CS HIGH
        print(f"Payload ({len(payload)} bytes): {[hex(x) for x in payload[:20]]}")
        
    spi.close()

if __name__ == "__main__":
    test_raw_bno08x()
