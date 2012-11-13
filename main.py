from routine import *
from holds import *
from moves import *
from state_estimation import current_state_estimate
from motor_control import send_command

routine = Routine(DEAD_HANG, [DEAD_HANG_TO_IRON_CROSS])

if __name__ == "__main__":
    print "Starting routine in the hold", routine.init_hold.name
    
    while True:
        # get the state estimate
        (q, q_dot) = current_state_estimate()

        # get the motor command
        motor_command = routine.get_command(q, q_dot)
        
        # send a command to the motors
        send_command(motor_command)
