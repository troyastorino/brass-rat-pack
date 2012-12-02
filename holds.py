from routine import Controller
from motor_control import generate_command_map, generate_servo_command, generate_torque_command
from common import total_energy,minTE,upright_min_TE
from math import sqrt, cos, sin, pi

import math

class HoldController(Controller):
    def is_stable(self, q, q_dot):
        """Given the generalized state of the system, returns a boolean indicating
        whether the system has stablized into the hold.  This must return True before
        system will continue to the next move."""
        raise NotImplementedError("Must implement is_stable for hold controllers.")
    
class Hold(object):
    """A hold class"""
    def __init__(self, name, controller):
        """name is a name for the hold, controller is a HoldController object"""
        self.name = name
        self.controller = controller

class SimpleHoldController(HoldController):
    """Simple illustration of a controller class"""
    def __init__(self, theta_1, theta_2, theta_3):
        """theta_1, theta_1, theta_3 are the target angles for the motors"""
        self.theta_1 = theta_1
        self.theta_2 = theta_2
        self.theta_3 = theta_3
        
    def control(self, q, q_dot):
        return generate_command_map(
            generate_servo_command(self.theta_1, torque_percentage = 1),
            generate_servo_command(self.theta_2, torque_percentage = 1),
            generate_servo_command(self.theta_3, torque_percentage = 1))
        
    def is_stable(self, q, q_dot):
        eps = 0.2
        
        # make sure all of the generalized coordinates are no longer moving
        for q_i_dot in q_dot:
            if abs(q_i_dot) > eps:
                return False

        # if we are not moving, make sure we are at the target position
        if abs(q[0] - self.theta_1) < eps and abs(q[1] - self.theta_2) < eps and abs(q[2] - self.theta_3) < eps:
            return True
        else:
            return False
     
# class EnergyController(HoldController):
#     """Simple illustration of a controller class"""
#     def __init__(self,TEtarget):
#         """theta_1, theta_1, theta_3 are the target angles for the motors"""
#         self.s1 = 1
#         self.s2 = 0
#         self.dtheta_1 = 0.01
#         self.dtheta_2 = 0.01
#         self.max_theta_1 = pi/2
#         self.max_theta_2 = pi/2


#         self.pdTE = 0.001
#         self.ndTE = 0.001
#         self.dE = 0.001
#         self.TEtarget = TEtarget
#         self.just_initialised = True
        
         
#     def control(self, q, q_dot):
#         if self.just_initialised:
#             self.TEbp = total_energy(q,q_dot)
#             self.just_initialised = False
                    
#         self.TE = total_energy(q,q_dot)

        
#         if (self.TE > self.TEbp+self.pdTE):
#                self.s1 = -self.s1
#                self.s2 = -self.s2
#                self.TEbp=self.TEbp+pdTE
        
            
#         if(self.TE< self.TEbp-self.ndTE):
#             self.TEbp = self.TEbp-self.ndTE;
               
#         if(q[0]*self.s1 < self.max_theta_1):
#             theta_1 = q[0]+self.s1*self.dtheta_1

#         if(q[1]*self.s2 < self.max_theta_2):
#             theta_2 = q[1]+self.s2*self.dtheta_2
               
#         return generate_command_map(
#             generate_servo_command(theta_1, torque_percentage = 0.7),
#             generate_servo_command(theta_2, torque_percentage = 0.7),
#             generate_servo_command(q[2], torque_percentage = 0.7))
        
#     def is_stable(self, q, q_dot):  
#           if (self.TE < self.TEtarget + self.dE):
#                return True
#           else:
#                return False

### Hold instances below here
DEAD_HANG = Hold("Dead Hang", SimpleHoldController(0, 0, 0))

IRON_CROSS = Hold("Iron Cross", SimpleHoldController(0, 0, 1.8))

# DAMPING = Hold("Damping Hold",EnergyController(minTE))
# SWING = Hold("Swing",EnergyController(upright_min_TE))
