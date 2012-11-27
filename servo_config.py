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

# define functions for establishing torque_control_mode and torque_limit in the
# Robotis_Servo class. Functions do not send unnecessary commands to the servo,
# and only sends necessary commands
def establish_torque_limit(self, limit):
    try:
        if self.current_torque_limit != limit:
            self.set_torque_limit(limit)
            self.current_torque_limit = limit
    # if current_torque_limit hasn't been defined yet
    except AttributeError:
        # initialize current_torque_limit to something that will not evaluate to
        # limit, to make sure the torque limit is properly set on first use
        self.current_torque_limit = not limit
        self.establish_torque_limit(limit)

def establish_torque_control_mode(self, state):
    """state is True if setting to torque control mode, or False if setting to
       servo mode
    """
    try:
        # move out of torque control mode
        if self.in_torque_control_mode and not state:
            self.disable_torque_control_mode()
            self.in_torque_control_mode = False
        # move into torque control mode
        elif not self.in_torque_control_mode and state:
            self.establish_torque_limit(1)            
            self.enable_torque_control_mode()
            self.in_torque_control_mode = True

    # if in_torque_control_mode hasn't been defined yet
    except AttributeError:
        # initialize in_torque_control_mode to the opposite of the desired state
        # to make sure that torque control mode is properly enabled or disabled
        self.in_torque_control_mode = not state
        self.establish_torque_control_mode(state)
        
# monkey patch the new functions to the Robotis_Servo class
Robotis_Servo.establish_torque_control_mode = establish_torque_control_mode
Robotis_Servo.establish_torque_limit = establish_torque_limit

s1 = Robotis_Servo(dm, 1)
s2 = Robotis_Servo(dm, 2)
s3 = Robotis_Servo(dm, 3)

# do initialization setup.  Make sure they are in the configuration we are
# expecting. Also add in state variables
for servo in [s1, s2, s3]:
    servo.establish_torque_control_mode(False)
    servo.establish_torque_limit(1)
