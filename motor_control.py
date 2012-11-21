import state_estimation
from math import pi

from servo_config import s1, s2, s3

"""Interacts with the motors"""

def send_command(command):
    # TODO: actually send the commands to the motors. For now just set the state
    # variables to the motor commands
    s1.move_angle(command.motor_1.theta, command.motor_2.omega)
    s2.move_angle(command.motor_2.theta, command.motor_2.omega)
    s3.move_angle(command.motor_3.theta, command.motor_3.theta)
