print("Loading libraries...")
#import cv2
#import numpy as np
#import time
#import torch
#from ultralytics import YOLO
import serial

import time
from pyfirmata import Arduino, util

print("Loading libraries complete!!!")

 
# Define the port where your Arduino is connected
# Change this to the actual port where your Arduino is connected
PORT = '/dev/ttyUSB0'
 
# Initialize Arduino board
board = Arduino(PORT)
 
# print(board.get_firmata_version())
 
# Start an iterator to avoid serial buffer overflow
it = util.Iterator(board)
it.start()
 
# Create a Servo object
esc5 = board.get_pin('d:' + str(6) + ':s')
esc6 = board.get_pin('d:' + str(7) + ':s')
esc7 = board.get_pin('d:' + str(8) + ':s')
 
surge = 126
 
# Send PWM signal with duty cycle of 1500 (center position)
esc7.write(surge)
esc5.write(surge)
esc6.write(surge)
 
kp = 0.2
ki = 0.0
kd = 0.0
 
error = float(input())
integral = 0.0
prevTime = time.time()
#derivative = (delX - prevDelX) / sampleTime
# prevDelX = delX
currentTime = time.time()
integral = integral + error * (currentTime - prevTime)
prevTime = currentTime
yaw = kp * error + ki * integral
yaw = max(min(yaw, 10.0), -10.0)
pid_out = int(yaw)
print(yaw)
 
if pid_out > 0 :
    esc5.write(surge - pid_out)
    print(surge - pid_out)
else :
    esc6.write(surge + pid_out)
    
time.sleep(10)
