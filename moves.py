from routine import Controller
from motor_control import *
from holds import *
from common import at_angles
from math import pi

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
    def __init__(self, hold_from, hold_to):
        super(AngleSetMoveController, self).__init__(hold_from, hold_to)

        self.command_sent = False
    
    def control(self, q, q_dot):
        if not self.command_sent:
            return generate_command_map(
                generate_servo_command(self.hold_to.controller.theta_1,
                                       angvel=.5),
                generate_servo_command(self.hold_to.controller.theta_2,
                                       angvel=.5),
                generate_servo_command(self.hold_to.controller.theta_3,
                                       angvel=.5))
        else:
            return generate_command_map(None, None, None)

    def is_finished(self, q, q_dot):
        return at_angles((self.hold_to.controller.theta_1,
                          self.hold_to.controller.theta_2,
                          self.hold_to.controller.theta_3),
                         q, q_dot, .2, .01)

class SwingMoveController(MoveController):
    @staticmethod
    def gen_state_map(command_map, state_transition_fn):
        return {'command': command_map,
                'transition_fn': state_transition_fn}
    
    def __init__(self):
        super(SwingMoveController, self).__init__(DEAD_HANG, DEAD_HANG)

        self.state = 0
        
        state_0_thetas = [pi/4, pi/2, 0]
        state_0_angvel = 1
        state_1_thetas = [pi/4, pi/2, pi/2]
        state_1_angvel = 1
        state_2_thetas = [-pi/4, -pi/2, pi]
        state_2_angvel = None # keeps the angvel at max
        state_3_thetas = [0, 0, 0]
        state_3_angvel = 1 # also at max angvel

        def state_0_transition(q, q_dot):
            return at_angles(state_0_thetas, q, q_dot, .2, .1)

        def state_1_transition(q, q_dot):
            return at_angles(state_1_thetas, q, q_dot, .2, .1)

        def state_2_transition(q, q_dot):
            return at_angles([0,0,0], q, q_dot, .5, 10)

        def state_3_transition(q, q_dot):
            return at_angles(state_3_thetas, q, q_dot, .1, .01)
        
        # map of the command to send at the beginning of each state. For each
        # state, have a tuple with the command map, 
        self.state_maps = {
            0: self.gen_state_map(generate_command_map(
                    generate_servo_command(state_0_thetas[0], angvel=state_0_angvel),
                    generate_servo_command(state_0_thetas[1], angvel=state_0_angvel),
                    generate_servo_command(state_0_thetas[2], angvel=state_0_angvel)),
                             state_0_transition),
            1: self.gen_state_map(generate_command_map(
                    generate_servo_command(state_1_thetas[0], angvel=state_1_angvel),
                    generate_servo_command(state_1_thetas[1], angvel=state_1_angvel),
                    generate_servo_command(state_1_thetas[2], angvel=state_1_angvel)),
                             state_1_transition),
            2: self.gen_state_map(generate_command_map(
                    generate_servo_command(state_2_thetas[0], angvel=state_2_angvel),
                    generate_servo_command(state_2_thetas[1], angvel=state_2_angvel),
                    generate_servo_command(state_2_thetas[2], angvel=state_2_angvel)),
                             state_2_transition),
            3: self.gen_state_map(generate_command_map(
                    generate_servo_command(state_3_thetas[0], angvel=state_3_angvel),
                    generate_servo_command(state_3_thetas[1], angvel=state_3_angvel),
                    generate_servo_command(state_3_thetas[2], angvel=state_3_angvel)),
                             state_3_transition),
            4: self.gen_state_map(generate_command_map(
                    generate_servo_command(state_3_thetas[0], angvel=None),
                    generate_servo_command(state_3_thetas[1], angvel=None),
                    generate_servo_command(state_3_thetas[2], angvel=None)),
                             None)}
        
    def control(self, q, q_dot):
        # see if transitioning to the next state
        if self.state_maps[self.state]['transition_fn'](q, q_dot):
            self.state += 1
        
        command = self.state_maps[self.state]['command']
        if command[MOTOR_1_KEY] != None:
            self.state_maps[self.state]['command'] = generate_command_map(
                NO_COMMAND, NO_COMMAND, NO_COMMAND)

        return command

    def is_finished(self, q, q_dot):
        return self.state == 4
            
### Move instances below here
DEAD_HANG_TO_IRON_CROSS = Move("Dead Hang to Iron Cross", AngleSetMoveController(DEAD_HANG, IRON_CROSS))

IRON_CROSS_TO_DEAD_HANG = Move("Iron Cross to Dead Hang", AngleSetMoveController(IRON_CROSS, DEAD_HANG))

SWING = Move("Swing", SwingMoveController())
