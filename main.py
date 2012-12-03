from routine import *
from holds import *
from moves import *
from routine_logging import log
from state_estimation import current_state_estimate
from motor_control import send_command

routine = Routine(DEAD_HANG, [TIMED_SWING, DEAD_HANG_TO_IRON_CROSS, IRON_CROSS_TO_DEAD_HANG, DEAD_HANG_PAUSE, DEAD_HANG_TO_PIKE, PIKE_TO_REVERSE_PIKE, REVERSE_PIKE_TO_DEAD_HANG, TEST_SWING])
# routine = Routine(DEAD_HANG, [TEST_SWING])

if __name__ == "__main__":
    print "Starting routine in the hold", routine.init_hold.name

    while True:
        # get the state estimate
        (q, q_dot, time) = current_state_estimate()

        # get the motor commands
        motor_command = routine.get_command(q, q_dot, time)

        # send a command to the motors
        send_command(motor_command)

        # log the state and the command
        log(q, q_dot, time, motor_command)
