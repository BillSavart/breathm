import board
import busio
import adafruit_bmp280

i2c = busio.I2C(board.SCL, board.SDA)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)

bmp280.sea_level_pressure = 1013.25

print("Temp: %.2f C" % bmp280.temperature)
print("Pressure: %.2f hPa" % bmp280.pressure)
print("Alti: %.2f m" % bmp280.altitude)

