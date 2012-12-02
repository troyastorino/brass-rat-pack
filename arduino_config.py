# Import required libraries

import serial
import time

# Setup serial connection to arduino - baud rate = 115200

ser=serial.Serial('/dev/ttyACM0',115200, timeout=1)

def initialise_serial():
    # Wait for arduino ready and intialisation to complete
    time.sleep(1)
    ser.write('\1')
    time.sleep(3)
    ser.write('\1')
    time.sleep(5)
    # Clear serial buffer ready for sensor data
    ser.flushInput()
    ser.flushOutput()
    

initialise_serial()
       
def get_IMU_data():
    ser.write('\1')
    time.sleep(0.01)
    IMU_data_raw = ser.readline();
    IMU_data = map(float, IMU_data_raw.strip().split("\t"))
    return IMU_data
