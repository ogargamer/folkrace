import time
import smbus2

bus = smbus2.SMBus(1)
addr = 0x4B

print("Benchmarking direct I2C SHTP read speed over 100 reads:")
t0 = time.time()
count = 0
for _ in range(100):
    try:
        msg = smbus2.i2c_msg.read(addr, 4)
        bus.i2c_rdwr(msg)
        count += 1
    except Exception:
        pass
dt = time.time() - t0
print(f"100 reads took {dt*1000.0:.2f} ms ({dt/100.0*1000.0:.3f} ms per read) -> Max I2C rate: {100.0/dt:.0f} Hz!")
bus.close()
