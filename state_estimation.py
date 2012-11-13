"""The module reads from the sensors and the actuators in order to estimate
the current state.  q"""

import math

# TODO: actually do estimation of state variables

theta_1 = 0 # shoulder rotation (in same plane as hip)
theta_1_dot = 0 
theta_2 = 0 # hip rotation
theta_2_dot = 0    
theta_3 = -math.pi/2 # shoulder separation
theta_3_dot = 0    
phi = 0 # ring to arm angle
phi_dot = 0    
psi = 0 # ring to platform angle
psi_dot = 0

def current_state_estimate():
    """Returns the current best estimate of q, q_dot"""

    # Package q and q_dot as tuples
    q = theta_1, theta_2, theta_3, phi, psi
    q_dot = theta_1_dot, theta_2_dot, theta_3_dot, phi_dot, psi_dot

    # Return a tuple of q, q_dot
    return q, q_dot
