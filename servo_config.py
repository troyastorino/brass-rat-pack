from lib_robotis import *
from math import pi

servo_param = {
    1: {'home_encoder': 545,
        'flipped': True,
        'min_ang': -1.6,
        'max_ang': 3},

    2: {'home_encoder': 2302,
        'flipped': True,
        'max_ang': 2.8,
        'min_ang': -2.6},
    
    3: {'home_encoder': 1024}
}

dm = USB2Dynamixel_Device('/dev/ttyUSB0')

s1 = Robotis_Servo(dm, 1)
s2 = Robotis_Servo(dm, 2)
s3 = Robotis_Servo(dm, 3)

# do initialization setup.  Make sure they are in the configuration we are expecting
for servo in [s1, s2, s3]:
    servo.disable_torque_control_mode()
