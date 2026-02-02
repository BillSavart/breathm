# self_check.py
import time
import sys
import RPi.GPIO as GPIO

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus
from bmp280 import BMP280

I2C_BUS_ID = 1             
BMP280_I2C_ADDR = 0x76     

IN1_PIN = 23   
IN2_PIN = 24   
ENA_PIN = 25   

EXTEND_TIME = 3  
RETRACT_TIME = 3 
TEST_SPEED = 80    

def self_check_bmp280():
    print("[SELF-CHECK] Checking BMP280...")

    try:
        bus = SMBus(I2C_BUS_ID)
        bmp280 = BMP280(i2c_dev=bus, i2c_addr=BMP280_I2C_ADDR)
        pressure = bmp280.get_pressure()
        print(f"[SELF-CHECK] BMP280 reads data successfully: {pressure:.2f} hPa")
        return True

    except Exception as e:
        print(f"[SELF-CHECK] BMP280 自檢失敗：{e}")
        return False


def setup_motor_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1_PIN, GPIO.OUT)
    GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(ENA_PIN, GPIO.OUT)

    pwm = GPIO.PWM(ENA_PIN, 800) 
    pwm.start(100)
    return pwm


def motor_stop(pwm):
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.LOW)
    pwm.ChangeDutyCycle(0)


def motor_extend(pwm, seconds):
    print("[SELF-CHECK] Motor: extending...")
    GPIO.output(IN1_PIN, GPIO.HIGH)
    GPIO.output(IN2_PIN, GPIO.LOW)
    pwm.ChangeDutyCycle(TEST_SPEED)
    time.sleep(seconds)
    motor_stop(pwm)


def motor_retract(pwm, seconds):
    print("[SELF-CHECK] Motor: Retracting...")
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.HIGH)
    pwm.ChangeDutyCycle(TEST_SPEED)
    time.sleep(seconds)
    motor_stop(pwm)


def self_check_actuator():
    print("[SELF-CHECK] Check L298N Actuator...")

    pwm = None
    try:
        pwm = setup_motor_gpio()

        motor_stop(pwm)
        time.sleep(0.5)

        motor_extend(pwm, EXTEND_TIME)
        time.sleep(0.5)

        motor_retract(pwm, RETRACT_TIME)
        time.sleep(0.5)

        print("[SELF-CHECK] Linear actuator operates successfully.")
        return True

    except Exception as e:
        print(f"[SELF-CHECK] Linear actuator fails: {e}")
        return False

    finally:
        if pwm is not None:
            pwm.stop()
        GPIO.cleanup()


def run_self_check():
    print("========== Start self-checking ==========")

    bmp_ok = self_check_bmp280()
    if not bmp_ok:
        print("[SELF-CHECK] BMP280 self-checking fails.")
    else:
        print("[SELF-CHECK] BMP280 self-checking passes.")

    motor_ok = self_check_actuator()
    if not motor_ok:
        print("[SELF-CHECK] Linear actuator self-checking fails.")
    else:
        print("[SELF-CHECK] Linear actuator self-checking passes.")

    all_ok = bmp_ok and motor_ok

    if all_ok:
        print("Everything looks good!")
    else:
        print("Something is wrong!")

    print("========== End of self-checking ==========")
    return all_ok


if __name__ == "__main__":
    ok = run_self_check()
    if not ok:
        sys.exit(1)  