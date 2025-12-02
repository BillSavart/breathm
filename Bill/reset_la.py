import RPi.GPIO as GPIO
from time import sleep

in1 = 23
in2 = 24
en = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)

# 縮
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.HIGH)
p = GPIO.PWM(en, 800)
p.start(100)

sleep(5)  # 縮到底

# 停
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)

GPIO.cleanup()

