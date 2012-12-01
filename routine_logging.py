from string import join
from motor_control import *
from datetime import datetime
from os import path
import logging

FORMAT = '%(relativeCreated)d\t%(message)s'
FILENAME = datetime.now().strftime(path.join('logs','%Y_%m_%d_%H_%M_%s.log'))

logging.basicConfig(filename=FILENAME, format=FORMAT, level=logging.DEBUG)

COLUMNS = ['theta_1', 'theta_2', 'theta_3', 'phi', 'psi', 'theta_1_dot', 'theta_2_dot', 'theta_3_dot', 'phi_dot', 'psi_dot', 'motor_1_mode', 'motor_1_theta', 'motor_1_angvel', 'motor_1_torque_limit', 'motor_1_torque', 'motor_2_mode', 'motor_2_theta', 'motor_2_angvel', 'motor_2_torque_limit', 'motor_2_torque', 'motor_3_mode', 'motor_3_theta', 'motor_3_angvel', 'motor_3_torque_limit', 'motor_3_torque'];

logging.info(join(COLUMNS, "\t"))

def log(q, q_dot, motor_commands):
    # convert the generalized coordinates into strings
    q_string = join(map(str, q), "\t")
    q_dot_string = join(map(str, q_dot), "\t")

    # combine to one string and log
    logging.info(
        join([q_string, q_dot_string, motor_command_string(motor_commands)],
             "\t"))
    
def motor_command_string(motor_commands):
    vals = []
    motor_keys = [MOTOR_1_KEY, MOTOR_2_KEY, MOTOR_3_KEY]

    for key in motor_keys:
        cmd = motor_commands[key]

        vals.extend([cmd.get(CONTROL_MODE_KEY), cmd.get(THETA_KEY, ''),
                     cmd.get(ANGVEL_KEY, ''), cmd.get(TORQUE_LIMIT_KEY, ''),
                     cmd.get(TORQUE_KEY, '')])

    vals = map(lambda(x): '' if x == None else x, vals)
    
    return join(map(str, vals), "\t")
