import state_estimation
from math import pi

from servo_config import s1, s2, s3

"""Interacts with the motors"""

# define the static keys and values for a command map
CONTROL_MODE_KEY = 'control_mode'
SERVO_CONTROL = 'servo'
TORQUE_CONTROL = 'torque'

MOTOR_1_KEY = 'motor_1'
MOTOR_2_KEY = 'motor_2'
MOTOR_3_KEY = 'motor_3'

THETA_KEY = 'theta'
ANGVEL_KEY = 'angvel'
TORQUE_LIMIT_KEY = 'torque_percentage'
TORQUE_KEY = 'torque'

NO_COMMAND = 'no_command'

# Example motor control map:

# {'motor_1': {'control_mode': 'servo',
#              'theta': 0,
#              'angvel': .2,
#              'torque_percentage: .8'},
#  'motor_2': {'control_mode': 'servo',
#              'theta': 0},
#  'motor_3': {'control_mode': 'torque',
#              'torque': 14}}
#
# You can also specify the value of a motor's command to be 'no_command'
#
# For servo control model, theta and angvel are optional; if not provided,
# torque_percentage will be set ot 1 and angvel will not be specified

## utility functions for generating motor commands
def generate_servo_command(theta, angvel=None, torque_percentage=1):
    return {CONTROL_MODE_KEY: SERVO_CONTROL,
            THETA_KEY: theta,
            ANGVEL_KEY: angvel,
            TORQUE_LIMIT_KEY: torque_percentage}

def generate_torque_command(torque):
    return {CONTROL_MODE_KEY: TORQUE_CONTROL,
            TORQUE: torque}

def generate_command_map(motor_1, motor_2, motor_3):
    return {MOTOR_1_KEY: motor_1,
            MOTOR_2_KEY: motor_2,
            MOTOR_3_KEY: motor_3}
        
def send_command(command_map, blocking=False):
    # get motor commands
    motor_1 = command_map.get(MOTOR_1_KEY)
    motor_2 = command_map.get(MOTOR_2_KEY)
    motor_3 = command_map.get(MOTOR_3_KEY)

    # make sure we have command maps for all of the motors
    if motor_1 == None or motor_2 == None or motor_3 == None:
        raise Exception('In a command map must specify commands for each motor. ' +
                        'If you don\'t want to send a command to the motor, ' +
                        'specify NO_COMMAND')

    # create a list of command-servo tuples
    cmd_list = [(motor_1, s1), (motor_2, s2), (motor_3, s3)]

    
    for i in range(len(cmd_list)):
        cmd, servo = cmd_list[i]
        if cmd == NO_COMMAND:
            pass
        else:
            # get the command mode
            mode = cmd.get(CONTROL_MODE_KEY)

            # execute control depending on the mode
            if mode == SERVO_CONTROL:
                servo.establish_torque_control_mode(False)

                # get the command keys
                theta = cmd.get(THETA_KEY)
                angvel = cmd.get(ANGVEL_KEY)
                torque_percentage = cmd.get(TORQUE_LIMIT_KEY)

                # make sure a theta is specified
                if theta == None:
                    raise Exception('Must specify a theta to move to')

                # if desired, set the torque limit
                if torque_percentage:
                    servo.establish_torque_limit(torque_percentage)
                else:
                    servo.establish_torque_limit(1)
                
                # command the servo to move
                try:
                    servo.move_angle(theta, angvel=angvel, blocking=blocking)
                except Exception as e:
                   raise Exception('Failure in moving servo '+ str(i+1) +
                                   '\n' + str(e))

            elif mode == TORQUE_CONTROL:
                # if not in torque control mode, switch into it
                servo.establish_torque_control_mode(True)

                torque = cmd.get(TORQUE_KEY)

                if torque == None:
                    raise Exception('Must specify the torque in torque control mode')

                # send the command
                servo.set_torque(torque)
            else:
                raise Exception('Must specify the control mode as either servo ' +
                                'control or torque control')
