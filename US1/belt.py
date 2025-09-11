import serial
import time
import struct
import sys
import RPi.GPIO as GPIO

in1 = 23
in2 = 24
en = 25
linear_actuator_max_distance = 50

# Function to calculate the checksum
def calculate_checksum(data):
    return sum(data) & 0xFF

# Initialize serial connection
ser = serial.Serial(
    port='/dev/serial0',  # Adjust according to your setup
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

def move_linear_actuator(direction):
    if (direction == 1):
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)

    elif (direction == -1):
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)

    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2 ,GPIO.LOW)

def mirror_breathing(curr_pressure, prev_pressure, position, direction):
    if (prev_pressure != 0 and curr_pressure > prev_pressure): # inhale
        if (position <= linear_actuator_max_distance):
            direction = 1

        else:
            direction = 0

    elif (prev_pressure != 0 and curr_pressure < prev_pressure): # exhale
        if (position >= 0):
            direction = -1

        else:
            direction = 0

    move_linear_actuator(direction)

    position += direction

    return position, direction

def main():
	GPIO.setmode(GPIO.BCM)
	
	# init linear actuators
	GPIO.setup(in1,GPIO.OUT)
	GPIO.setup(in2,GPIO.OUT)
	GPIO.setup(en,GPIO.OUT)
	GPIO.output(in1,GPIO.LOW)
	GPIO.output(in2,GPIO.LOW)
	p = GPIO.PWM(en,800)
	p.start(100)
	
	la_position = 0
	la_direction = 0
    
	# Command to send to the sensor
	command = bytearray([0xFF, 0xCC, 0x03, 0xA3, 0xA0])

	# Send command to the sensor
	ser.write(command)
	time.sleep(5)
	
	prev_pressure = 0
	curr_pressure = 0
	
	while True:
		# Read the response from the sensor
		response = ser.read(35)
		
		'''
		for res in response:
			print(hex(res))
			
		print("==")
		'''
		
		
		if (response[28] == 0xFF and response[29] == 0xCC):
			mbh = response[33]
			mbl = response[34]
			curr_pressure = (mbh << 8) | mbl
			print(curr_pressure)
		
		#print(res_data)
		
		la_position, la_direction = mirror_breathing(curr_pressure, prev_pressure, la_position, la_direction)
		
		prev_pressure = curr_pressure

		# Wait for 0.1 seconds before next read
		time.sleep(0.1)

	
if __name__ == "__main__":
	try:
		main()

	except KeyboardInterrupt:
		print("Keyboard Interrupt")
		command = bytearray([0xFF, 0xCC, 0x03, 0xA4, 0xA1])
		ser.write(command)
		sys.exit(0)
