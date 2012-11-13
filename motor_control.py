import state_estimation

"""Interacts with the motors"""

def send_command(command):
    # TODO: actually send the commands to the motors. For now just set the state
    # variables to the motor commands
    state_estimation.theta_1 = command.motor_1.theta
    state_estimation.theta_1_dot = command.motor_1.omega 
    state_estimation.theta_2 = command.motor_2.theta
    state_estimation.theta_2_dot = command.motor_2.omega 
    state_estimation.theta_3 = command.motor_3.theta
    state_estimation.theta_3_dot = command.motor_3.omega 

