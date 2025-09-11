from bmp280 import BMP280
from smbus import SMBus
import time

bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus)
bmp280.setup(mode="forced")

while True:
    print("Presure: ", bmp280.get_pressure(), " hPa")
    time.sleep(1)
