import serial
import time
import RPi.GPIO as GPIO

# Initialize serial communication
ser = serial.Serial('/dev/serial0', 115200, timeout=1)  # Replace with your serial port if different

def calculate_cksum():
    # Calculate checksum (CKSUM) as needed. Here is a simple example. Adjust according to your protocol.
    return 0xA3

# Initialize communication bytes
request_bytes = bytearray([0xFF, 0xCC, 0x03, 0xA3, 0xA0])

# Function to read respiratory data
def read_respiratory_data():
    # Read response from sensor
    response = ser.read(8)  # Adjust the length if necessary
    
    if len(response) == 8 and response[:3] == bytearray([0xFF, 0xCC, 0x05]):
        cksum = response[3]
        # You might need to verify CKSUM here
        MBH = response[6]
        MBL = response[7]
        respiratory_data = (MBH << 8) | MBL
        print(respiratory_data)
    '''else:
        print("Invalid response:", response)
    '''
# Main loop to read data every 0.1 seconds
#try:
    # Send request to sensor
ser.write(request_bytes)
    
while True:
    read_respiratory_data()
    time.sleep(0.1)
'''
except KeyboardInterrupt:
    pass
finally:
    ser.close()
'''
