#!/usr/bin/env python

import sys
import time
import statistics
import RPi.GPIO as GPIO 
from bmp280 import BMP280

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus
    
global user_inhale
global user_exhale
global sampling_rate
global guiding_type


# user condition
user_inhale = 0
user_exhale = 0

global curr_pressure
global prev_pressure
global position
global direction

# linear actuator
in1 = 23
in2 = 24
en = 25
max_distance = 50


mirror_breath_diff_tolerance = 0.5
guide_breath_diff_tolerance = 0.4
pressure_diff_tolerance = 0.5
sampling_rate = 0.02
target_mirror = 4

MACHINE_STATE = {
    "MIRROR": 0,
    "GUIDE": 1
}

BREATH_STATE = {
    "INHALE": 0,
    "EXHALE": 1
}

machine_state = MACHINE_STATE["MIRROR"]

GPIO.setmode(GPIO.BCM)

fig = plt.figure(dpi=200)
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

curr_pressure = 0
prev_pressure = 0
position = 0
direction = 0

def init_bmp280():
    # Initialise the BMP280
    bus1 = SMBus(1)
    bmp280s = [BMP280(i2c_dev=SMBus(1))]

    # Set up oversampling
    for bmp280 in bmp280s:
        bmp280.setup(mode="forced") # , pressure_oversampling=16
    #  bmp280.setup(mode="forced", temperature_oversampling=2, pressure_oversampling=16, filter=1, standby=4)

    return bmp280s
    
bmp280s = init_bmp280()

def detect_breathing(bmp280s):
    curr_pressures = [0.0]
    
    for idx, bmp280 in enumerate(bmp280s):
        curr_pressures[idx] = bmp280.get_pressure() 

        curr_pressure = round(statistics.median(curr_pressures), 1)

    return curr_pressure

def mirror_breathing(bmp280s, prev_pressure, position, direction):
    curr_pressure = detect_breathing(bmp280s)

    if (prev_pressure != 0 and curr_pressure > prev_pressure): # inhale direction 1
        if (position <= max_distance):
            direction = 1
            
        else:
            direction = 0
        
    elif (prev_pressure != 0 and curr_pressure < prev_pressure): # exhale direction -1
        if (position > 1):
            direction = -1

        else:
            direction = 0

    prev_pressure = curr_pressure
    position += direction

    return curr_pressure, position, direction



def animate(i, xs, ys):
    global prev_pressure, position, direction
    
    current_time = datetime.now()
    format_time = current_time.strftime("%H%M%S")
    
    curr_pressure, position, direction = mirror_breathing(bmp280s, prev_pressure, position, direction)
    #print(curr_pressure)
    print(format_time, curr_pressure)
    
    
    xs.append(i)
    ys.append(curr_pressure)

    # Limit x and y lists to 20 items
    xs = xs[-80:]
    ys = ys[-80:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)
    ax.set_xlim(80* int(i / 80), 80* int(i / 80)+80)

    # Format plot
    #plt.ylim(1032, 1050)
    
    
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Pressure over Time')
    plt.ylabel('Pressure (Pa)')
    
    '''
    if (curr_pressure > prev_pressure):
        if (user_state is BREATH_STATE["EXHALE"]):
            curr_bpm = (curr_breath["inhale"] + curr_breath["exhale"]) / 2
            
            bpms.append(curr_bpm)
            
            if (machine_state == MACHINE_STATE["MIRROR"]):
                mirror_count += 1
            else:
                mirror_count = 0

            # current conditions for data
            print("bpms", bpms)
            print("== 1 breath completed ==")
            print("machine_state", machine_state)
            print("target_breath", target_breath)
            print("curr_breath", curr_breath)
            print("count", count)
            print("magic_numbers", magic_numbers)
            
            # updated conditions
            print("== updated conditions ==")
            print("target_breath", target_breath)
            print("magic_numbers", magic_numbers)
            
            # reset variables
            prev_breath = curr_breath
            curr_breath = {
                "inhale": 0,
                "exhale": 0
            }
            
        curr_breath["inhale"] += sampling_rate
        
        user_state = BREATH_STATE["INHALE"]  
        
        
        
        
        

    elif (curr_pressure < prev_pressure):
        # if (user_state is BREATH_STATE["INHALE"]):
        #     print("user is exhaling")

        curr_breath["exhale"] += sampling_rate
        user_state = BREATH_STATE["EXHALE"]

    else:
        if (user_state is BREATH_STATE["INHALE"]):
            curr_breath["inhale"] += sampling_rate

        else:
            curr_breath["exhale"] += sampling_rate
    '''
    prev_pressure = curr_pressure

    # time.sleep(sampling_rate)
    

'''

'''

try:
    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=sampling_rate)
    plt.show()
    '''
    while True:
        now = dt.datetime.now()
        print(now.strftime("%Y%m%d%H%M%S"))
        curr_pressure, position, direction = mirror_breathing(bmp280s, prev_pressure, position, direction)
        prev_pressure = curr_pressure
        
        print(prev_pressure)
        
        time.sleep(sampling_rate)
    '''
        
except KeyboardInterrupt:
		# print("\n")
		sys.stdout = orig_stdout
		f.close()
		GPIO.cleanup()
		sys.exit(0)
