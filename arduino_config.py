# Import required libraries

import serial
import time

# Setup serial connection to arduino - baud rate = 115200
YAW0=None
PITCH0=None
ROLL0=None
GYROX0=None
GYROY0=None
GYROZ0=None
ser=serial.Serial('/dev/ttyACM1',115200, timeout=1)
 
def get_IMU_data():
    ser.write('\1')
    time.sleep(0.01)
    IMU_data_raw = ser.readline();
    IMU_data = map(float, IMU_data_raw.strip().split("\t"))
    return IMU_data
def IMU_zero():
    GYRO_settled=False
    GYROY=get_IMU_data()[4]
    while GYROY > 30:
        GYROY=get_IMU_data()[4]
    
    ser.write('\1')
    IMU_data_zero_raw = ser.readline();
    IMU_data_zero = map(float, IMU_data_zero_raw.strip().split("\t"))
    print IMU_data_zero
    return IMU_data_zero
def initialise_serial():
    # Wait for arduino ready and intialisation to complete
    time.sleep(1)
    ser.write('\1')
    time.sleep(3)
    # Clear serial buffer ready for sensor data
    ser.flushInput()
    ser.flushOutput()
    IMU_VALS=IMU_zero()
    YAW0=IMU_VALS[0]
    PITCH0=IMU_VALS[1]
    ROLL0=IMU_VALS[2]
    GYROX0=IMU_VALS[3]
    GYROY0=IMU_VALS[4]
    GYROZ0=IMU_VALS[5]



initialise_serial()
