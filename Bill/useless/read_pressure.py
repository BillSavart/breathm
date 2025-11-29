from bmp280 import BMP280
from smbus import SMBus
import RPi.GPIO as GPIO
import sys
from enum import Enum
import time
import numpy as np
from scipy.signal import butter, filtfilt

def main():
    bus = SMBus(1)
    bmp280 = BMP280(i2c_dev=bus)
    bmp280.setup(mode="forced")

    while True:
        raw_pressure = bmp280.get_pressure()
        print(f"Pressure: {raw_pressure:.2f} hPa")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        GPIO.cleanup()
        sys.exit(0)
