from string import join
from motor_control import *
from datetime import datetime
from os import path
from motor_control import *
import logging

FORMAT = '%(relativeCreated)d,%(message)s'
FILENAME = datetime.now().strftime(path.join('logs','%Y_%m_%d_%H_%M_%s.log'))
COLUMNS = ['time', 'theta_1', 'theta_2', 'theta_3', 'phi', 'psi', 'theta_1_dot', 'theta_2_dot', 'theta_3_dot', 'phi_dot', 'psi_dot', 'motor_1_mode', 'motor_1_theta', 'motor_1_angvel', 'motor_1_torque_limit', 'motor_1_torque', 'motor_2_mode', 'motor_2_theta', 'motor_2_angvel', 'motor_2_torque_limit', 'motor_2_torque', 'motor_3_mode', 'motor_3_theta', 'motor_3_angvel', 'motor_3_torque_limit', 'motor_3_torque'];

with open(FILENAME, 'w') as f:
    f.write(join(COLUMNS, ',') + "\n")

logging.basicConfig(filename=FILENAME, format=FORMAT, level=logging.DEBUG)

def log(q, q_dot, motor_commands):
    # convert the generalized coordinates into strings
    q_string = join(map(str, q), ",")
    q_dot_string = join(map(str, q_dot), ",")

    # combine to one string and log
    logging.info(
        join([q_string, q_dot_string, motor_command_string(motor_commands)],
             ","))
    
def motor_command_string(motor_commands):
    vals = []
    motor_keys = [MOTOR_1_KEY, MOTOR_2_KEY, MOTOR_3_KEY]

    for key in motor_keys:
        cmd = motor_commands[key]

        if cmd == NO_COMMAND:
            vals.extend(['NaN', 'NaN', 'NaN', 'NaN'])
        else:
            mode = 0 if cmd.get(CONTROL_MODE_KEY) == SERVO_CONTROL else 1

            vals.extend([mode, cmd.get(THETA_KEY, 'NaN'),
                         cmd.get(ANGVEL_KEY, 'NaN'), cmd.get(TORQUE_LIMIT_KEY, 'NaN'),
                         cmd.get(TORQUE_KEY, 'NaN')])

    vals = map(lambda(x): 'Nan' if x == None else x, vals)
    
    return join(map(str, vals), ",")
