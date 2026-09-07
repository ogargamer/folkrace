import gpiod
import time

# Scan all lines on gpiochip4 (RP1 GPIO)
chip = gpiod.Chip('gpiochip4')
print("Reading RP1 GPIO states for UART pins:")

for pin in [4, 5, 14, 15, 18, 19]:
    try:
        line = chip.get_line(pin)
        line.request(consumer="probe", type=gpiod.LINE_REQ_DIR_IN)
        val = line.get_value()
        line.release()
        print(f"  GPIO {pin} (Physical Pin on Header): Value = {val}")
    except Exception as e:
        print(f"  GPIO {pin}: {e} (occupied by driver/overlay)")

chip.close()
