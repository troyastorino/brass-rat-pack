"""The module reads from the sensors and the actuators in order to estimate
the current state.  q"""

import math
from servo_config import s1, s2, s3
import arduino_config

# TODO: actually do estimation of state variables

phi = 0 # ring to arm angle
phi_dot = 0    
psi = 0 # ring to platform angle
psi_dot = 0

def current_state_estimate():
    """Returns the current best estimate of q, q_dot"""
    # read data from arduino
    IMU_data_import = arduino_config.get_IMU_data()
    Yaw1 = IMU_data_import[0]
    Pitch1 = IMU_data_import[1]
    Roll1 = IMU_data_import[2]
    Gyrox1 = IMU_data_import[3]
    Gyroy1 = IMU_data_import[4]
    Gyroz1 = IMU_data_import[5]
    Yaw2 = IMU_data_import[6]
    Pitch2 = IMU_data_import[7]
    Roll2 = IMU_data_import[8]
    Gyrox2 = IMU_data_import[9]    
    Gyroy2 = IMU_data_import[10]
    Gyroz2 = IMU_data_import[11]

    # read angular positions
    theta_1 = s1.read_angle()
    theta_2 = s2.read_angle()
    theta_3 = s3.read_angle()

    # read angular velocities
    theta_1_dot = s1.read_angvel()
    theta_2_dot = s2.read_angvel()
    theta_3_dot = s3.read_angvel()

    # Package q and q_dot as tuples
    q = theta_1, theta_2, theta_3, phi, psi
    q_dot = theta_1_dot, theta_2_dot, theta_3_dot, phi_dot, psi_dot

    # Return a tuple of q, q_dot
    return q, q_dot
