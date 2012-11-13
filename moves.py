from routine import Controller, MotorCommand, Command
from holds import *

class MoveController(Controller):
    def __init__(self, hold_from, hold_to):
        """hold_from and hold_to are the Holds that this controller moves between"""
        self.hold_from = hold_from
        self.hold_to = hold_to

    def is_finished(self, q, q_dot):
        """Given the generalized state of the system, returns a boolean indicating
        move has completed.  Must return True before the system continues to the
        hold controller."""
        raise NotImplementedError("Must implement move_finished")
    
class Move(object):
    """An class to define a transition between two holds"""
    def __init__(self, name, controller):
        self.name = name
        self.controller = controller
        
class DeadHangToIronCrossController(MoveController):
    delta_theta = 0.01
    
    def __init__(self):
        super(DeadHangToIronCrossController, self).__init__(DEAD_HANG, IRON_CROSS)

        # internal variable for keeping track of whether the move is done
        self.done = False

    def control(self, q, q_dot):
        if (q[2] < self.hold_to.controller.theta_3):
            # increase theta_3 by a small amount
            theta_3 = q[2] + self.delta_theta
            return Command(
                MotorCommand(self.hold_to.controller.theta_1, 0, 0.7),
                MotorCommand(self.hold_to.controller.theta_2, 0, 0.7),
                MotorCommand(theta_3, 0, 0.7))
        else:
            self.done = True
            return self.hold_to.controller.control(q, q_dot)

    def is_finished(self, q, q_dot):
        # normally, might calculate based on q and q_dot, but here just use an
        # internal variable
        return self.done

### Move instances below here
DEAD_HANG_TO_IRON_CROSS = Move("Dead Hang to Iron Cross", DeadHangToIronCrossController())