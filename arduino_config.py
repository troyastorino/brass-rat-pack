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
ser=serial.Serial('/dev/ttyACM0',115200, timeout=1)

def read_IMU():
    ser.write('\1')
    time.sleep(0.01)
    return ser.readline();

def parse_IMU_data(data):
    return map(float, data.strip().split("\t"))
    
def get_IMU_data():
    return parse_IMU_data(read_IMU())

def IMU_zero():
    print "Waiting for IMU values to settle"
    GYRO_settled=False
    init_data=None
    attempts = 0
    while not GYRO_settled:
        GYRO_settled=True
        attempts += 1
        time.sleep(.5)
        init_data=get_IMU_data()
        for gyro in init_data[3:]:
           if abs(gyro) > 5:
               GYRO_settled = False
        if attempts > 40:
            raise Exception('Too many attempts to wait for the gyro to settle. '+
                              'The last data read is below: ' + str(init_data))

    return init_data

def initialise_dmp():
    initialized = False
    timeouts = 0
    print "Initialising DMP"
    while not initialized:
        data = None
        try:
            ser.flushInput()
            ser.flushOutput()
            ser.write('\1')
            data = read_IMU()
            parse_IMU_data(data)
            initialized = True
        except:
            if data == '':
                timeouts += 1
            if timeouts > 10:
                raise Exception('Timed out too many times during initialisation. '+
                                'Reset the Arduino.')
    print "Successfully initialized DMP!"

def initialise_serial():
    initialise_dmp()

    # Wait for arduino ready and intialisation to complete
#    time.sleep(1)
#    ser.write('\1')
#    time.sleep(3)
#    ser.write('\1')
#    time.sleep(5)
    # Clear serial buffer ready for sensor data
    #ser.flushInput()
    #ser.flushOutput()
    IMU_VALS=IMU_zero()
    global YAW0
    global PITCH0
    global ROLL0
    global GYROX0
    global GYROY0
    global GYROZ0
    YAW0=IMU_VALS[0]
    PITCH0=IMU_VALS[1]
    ROLL0=IMU_VALS[2]
    GYROX0=IMU_VALS[3]
    GYROY0=IMU_VALS[4]
    GYROZ0=IMU_VALS[5]
    
initialise_serial()
