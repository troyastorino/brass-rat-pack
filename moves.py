from routine import Controller
from motor_control import generate_command_map, generate_servo_command, generate_torque_command
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

class AngleSetMoveController(MoveController):
    angle_precision = .2
    angvel_precision = .01
    
    def __init__(self, hold_from, hold_to):
        super(AngleSetMoveController, self).__init__(hold_from, hold_to)

        self.command_sent = False
    
    def control(self, q, q_dot):
        if not self.command_sent:
            return generate_command_map(
                generate_servo_command(self.hold_to.controller.theta_1,
                                       torque_percentage = 0.7),
                generate_servo_command(self.hold_to.controller.theta_2,
                                       torque_percentage = 0.7),
                generate_servo_command(self.hold_to.controller.theta_3,
                                       torque_percentage = 0.7))
        else:
            return generate_command_map(None, None, None)

    def is_finished(self, q, q_dot):
        angle_tuples = [(q[0], self.hold_to.controller.theta_1),
                        (q[1], self.hold_to.controller.theta_2),
                        (q[2], self.hold_to.controller.theta_3)]

        # check if angles are at the goal
        for angle, goal in angle_tuples:
            if abs(angle - goal) > self.angle_precision:
                return False

        # check if velocity is zero
        for vel in [q_dot[0], q_dot[1], q_dot[2]]:
            if abs(vel) > self.angvel_precision:
                return False

        # if here, we have completed
        return True
    
### Move instances below here
DEAD_HANG_TO_IRON_CROSS = Move("Dead Hang to Iron Cross", AngleSetMoveController(DEAD_HANG, IRON_CROSS))
