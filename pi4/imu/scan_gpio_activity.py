import time
import os

# Check pin activity using pinctrl or sysfs
print("Scanning Pi 5 GPIO header pins for activity / pulses from ESP32:")

# We can run pinctrl poll or test pins 1 to 27
os.system("pinctrl get 0-27")
