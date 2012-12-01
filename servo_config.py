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
    
    3: {'home_encoder': 120}
}

dm = USB2Dynamixel_Device('/dev/ttyUSB0')

# extend Robotis_Servo class to have new methods for establishing
# torque_control_mode and torque_limit. Methods do not send unnecessary commands
# to the servo
class Extended_Robotis_Servo(Robotis_Servo):
    def __init__(self, dev_name='/dev/ttyUSB0', baudrate=400000):
        """Extended init makes sure the servos are in the state we expect
        them to be at startup"""
        super(Extended_Robotis_Servo, self).__init__(dev_name, baudrate)

        self.disable_torque_control_mode()
        self.in_torque_control_mode = False

        self.set_torque_limit(1)
        self.current_torque_limit = 1
        
    def establish_torque_limit(self, limit):
        """limit should be of the same form as that passed to
        set_torque_limit"""
        if self.current_torque_limit != limit:
            self.set_torque_limit(limit)
            self.current_torque_limit = limit

    def establish_torque_control_mode(self, state):
        """state is True if setting to torque control mode, or False if setting to
        servo mode"""
        # move out of torque control mode
        if self.in_torque_control_mode and not state:
            self.disable_torque_control_mode()
            self.in_torque_control_mode = False
        # move into torque control mode
        elif not self.in_torque_control_mode and state:
            self.establish_torque_limit(1)            
            self.enable_torque_control_mode()
            self.in_torque_control_mode = True

s1 = Extended_Robotis_Servo(dm, 1)
s2 = Extended_Robotis_Servo(dm, 2)
s3 = Extended_Robotis_Servo(dm, 3)
