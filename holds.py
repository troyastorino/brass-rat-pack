from routine import Controller, MotorCommand, Command

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
        motor_1_command = MotorCommand(self.theta_1, 0, 0.7)
        motor_2_command = MotorCommand(self.theta_2, 0, 0.7)
        motor_3_command = MotorCommand(self.theta_3 , 0, 0.7)        
        return Command(motor_1_command, motor_2_command, motor_3_command)
        
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

                 
