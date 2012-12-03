from routine import Controller
from motor_control import *
from holds import *
from common import at_angles
from math import pi
from os.path import join

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
    def __init__(self, hold_from, hold_to, angvel):
        super(AngleSetMoveController, self).__init__(hold_from, hold_to)

        self.angvel = angvel
        self.command_sent = False
    
    def control(self, q, q_dot, time):
        if not self.command_sent:
            return generate_command_map(
                generate_servo_command(self.hold_to.controller.theta_1,
                                       angvel=self.angvel),
                generate_servo_command(self.hold_to.controller.theta_2,
                                       angvel=self.angvel),
                generate_servo_command(self.hold_to.controller.theta_3,
                                       angvel=self.angvel))
        else:
            return generate_command_map(None, None, None)

    def is_finished(self, q, q_dot):
        return at_angles((self.hold_to.controller.theta_1,
                          self.hold_to.controller.theta_2,
                          self.hold_to.controller.theta_3),
                         q, q_dot, .2, .01)

class StaticWaypointController(MoveController):
    def read_float(self, s):
        """Converts a string to a float, except on error, returns none"""
        try:
            return float(s)
        except:
            return None
    
    def command_map_from_waypoint_vals(self, line_vals):
        """Takes the values read from a line and converts it into a command map"""
        commands = []
        for i in range(3):
            theta = line_vals[2*i]

            if theta == None:
                commands.append(NO_COMMAND)
            else:
                commands.append(generate_servo_command(theta, line_vals[2*i+1]))
                    
        return generate_command_map(commands[0], commands[1], commands[2])
    
    def command_map_from_waypoint(self, line):
        """Expects a list of value strings in the waypoint file, in the order
        [theta_1, angvel_1, theta_2, angvel_2, theta_3, angvel_1]"""
        line_vals = map(lambda(x): self.read_float(x.strip()), line)
        return self.command_map_from_waypoint_vals(line_vals)
    
    def read_waypoints_file(self, waypoints_file):
        """Assumes waypoints_file is inside the folders waypoint_files.
        Returns a list of commands"""
        with open(join('waypoint_files', waypoints_file), 'r') as f:
            lines = map(lambda(line): line.strip().split(','), f.readlines())
            return map(self.command_map_from_waypoint, lines)
        
    def __init__(self, hold_from, hold_to, waypoints_file):
        """Assumes waypoints_file is inside the folders waypoint_files"""
        super(StaticWaypointController, self).__init__(hold_from, hold_to)

        self.commands = self.read_waypoints_file(waypoints_file)

        self.state = 0
        self.state_command_sent = False

    def finished_state(self, q, q_dot, time):
        command = self.commands[self.state]

        angles = []
        for cmd in [command[MOTOR_1_KEY], command[MOTOR_2_KEY], command[MOTOR_3_KEY]]:
            try:
                angles.append(cmd[THETA_KEY])
            except:
                angles.append(None)

        return at_angles(angles, q, q_dot)

    def move_to_next_state(self, q, q_dot, time):
        self.state_command_sent = False
        self.state += 1

    def control(self, q, q_dot, time):
        if self.finished_state(q, q_dot, time):
            self.move_to_next_state(q, q_dot, time)
            
        if (not self.state_command_sent) and self.state < len(self.commands):
            self.state_command_sent = True
            return self.commands[self.state]
        else:
            return generate_command_map(NO_COMMAND, NO_COMMAND, NO_COMMAND)

    def is_finished(self, q, q_dot):
        return self.state >= len(self.commands)

class TimedWaypointController(StaticWaypointController):
    """Same as the static waypoint controller, but can specify a time from the
    last waypoint to move to the next waypoint instead of just waiting for the
    servo to reach the waypoint"""
    def command_map_from_waypoint_vals(self, line_vals):
        try:
            self.time_delays.append(line_vals[6])
        except:
            self.time_delays.append(None)

        return super(TimedWaypointController, self).command_map_from_waypoint_vals(
            line_vals[0:6])
    
    def __init__(self, hold_from, hold_to, waypoints_file):
        self.time_delays = []
        self.state_start_time = None
        super(TimedWaypointController, self).__init__(hold_from, hold_to,
                                                      waypoints_file)

    def finished_state(self, q, q_dot, time):
        if self.time_delays[self.state] != None:
            return time > self.time_delays[self.state] + self.state_start_time
        else:
            return super(TimedWaypointController, self).finished_state(q, q_dot, time)

    def move_to_next_state(self, q, q_dot, time):
        super(TimedWaypointController, self).move_to_next_state(q, q_dot, time)
        self.state_start_time = time

    def control(self, q, q_dot, time):
        if self.state_start_time == None:
            self.state_start_time = time

        return super(TimedWaypointController, self).control(q, q_dot, time)

class Pause(MoveController):
    """ Controller to pause at a certain hold"""
    def __init__(self, hold, pause_length):
        """ Length of pause in seconds"""
        super(Pause, self).__init__(hold, hold)

        self.pause = pause_length
        self.start_time = None
        self.finished = False
        
    def control(self, q, q_dot, time):
        if self.start_time == None:
            self.start_time = time
            
        if time > self.start_time + self.pause:
            self.finished = True
        
        return generate_empty_command_map()

    def is_finished(self, q, q_dot):
        return self.finished

### Move instances below here
DEAD_HANG_TO_IRON_CROSS = Move("Dead Hang to Iron Cross", AngleSetMoveController(DEAD_HANG, IRON_CROSS, 10))

IRON_CROSS_TO_DEAD_HANG = Move("Iron Cross to Dead Hang", AngleSetMoveController(IRON_CROSS, DEAD_HANG, 1))

DEAD_HANG_TO_PIKE = Move("Dead hang to pike", AngleSetMoveController(DEAD_HANG, PIKE_POS, 1))

PIKE_TO_REVERSE_PIKE = Move("Pike to reverse pike", AngleSetMoveController(PIKE_POS, REVERSE_PIKE, 4))

REVERSE_PIKE_TO_DEAD_HANG = Move("Reverse pike to dead hang", AngleSetMoveController(REVERSE_PIKE, DEAD_HANG, 4))

TEST_SWING = Move("Test swing", StaticWaypointController(DEAD_HANG, DEAD_HANG, 'swing2.points'))

TIMED_SWING = Move("timed swing", TimedWaypointController(DEAD_HANG, DEAD_HANG, 'swing2timed.points'))

DEAD_HANG_PAUSE = Move("Dead hang pause", Pause(DEAD_HANG, 5))
