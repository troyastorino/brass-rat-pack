from routine import Controller
from motor_control import generate_command_map, generate_servo_command, generate_torque_command

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
            generate_servo_command(self.theta_1, torque_percentage = 0.7),
            generate_servo_command(self.theta_2, torque_percentage = 0.7),
            generate_servo_command(self.theta_3, torque_percentage = 0.7))
        
    def is_stable(self, q, q_dot):
        eps = 0.01
        
        # make sure all of the generalized coordinates are no longer moving
        for q_i_dot in q_dot:
            if abs(q_i_dot) > eps:
                return False

        # if we are not moving, make sure we are at the target position
        if abs(q[0] - self.theta_1) < eps and abs(q[1] - self.theta_2) < eps and abs(q[2] - self.theta_3) < eps:
            return True
        else:
            return False

        
class DampingController(HoldController):
    """Simple illustration of a controller class"""
    def __init__(self, theta_1, theta_2, theta_3):
        """theta_1, theta_1, theta_3 are the target angles for the motors"""
        self.theta_1 = theta_1
        self.theta_2 = theta_2
        self.theta_3 = theta_3
        self.s1 = 1
        self.s2 = 0
        self.TEbp = TEbp
        self.dtheta_1 = 0.00001
        self.dtheta_2 = 0.00001


        self.pdTE = 0.001
        self.ndTE = 0.001
        self.dE = 0.001
        self.TEtarget = minPE+self.dE
        self.TE = Calculate_Total_Energy
        
         
    def control(self, q, q_dot):
        self.TE = Calculate_Total_Energy 

        if (self.TE < self.TEtarget):
               return "target reached"
    
            
        if (self.TE > self.TEbp+self.pdTE):
               self.s1 = -self.s1
               self.s2 = -self.s2
               self.TEbp=self.TEbp+pdTE
        
            
        if(self.TE< self.TEbp-self.ndTE):
               self.TEbp = self.TEbp-self.ndTE;

        self.theta_1 = self.theta_1+self.s1*self.dtheta_1
        self.theta_2 = self.theta_2+self.s2*self.dtheta_2
               
        return generate_command_map(
            generate_servo_command(self.theta_1, torque_percentage = 0.7),
            generate_servo_command(self.theta_2, torque_percentage = 0.7),
            generate_servo_command(self.theta_3, torque_percentage = 0.7))
        
    def is_stable(self, q, q_dot):
        eps = 0.01
        
        # make sure all of the generalized coordinates are no longer moving
        for q_i_dot in q_dot:
            if abs(q_i_dot) > eps:
                return False

        # if we are not moving, make sure we are at the target position
        if abs(q[0] - self.theta_1) < eps and abs(q[1] - self.theta_2) < eps and abs(q[2] - self.theta_3) < eps:
            return True
        else:
            return False

### Hold instances below here
DEAD_HANG = Hold("Dead Hang", SimpleHoldController(0, 0, -math.pi/2))

IRON_CROSS = Hold("Iron Cross", SimpleHoldController(0, 0, 0))

                 
