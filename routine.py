import sys

class Controller(object):
    """Abstract controller class"""
    def control(self, q, q_dot):
        """Given the generalized state for the system, returns a command_map to be
           sent to the motors."""
        raise NotImplementedError("Must define a control method")
                   
class Routine(object):
    """The routine object encapsulates the initial hold, and the sequence of moves that make up the transition"""
    def __init__(self, init_hold, moves):
        """init_hold is the Hold that the robot starts the routine in.
           moves is the list of Moves that makes ups the routine."""
        self.init_hold = init_hold

        ## Check to see if the list of moves is consistent
        inconsistent = False
        if moves[0].controller.hold_from != init_hold:
            inconsistent = True
        else:
            for i in range(1, len(moves)):
                if moves[i].controller.hold_from != moves[i-1].controller.hold_to:
                    inconsistent = True
        if inconsistent:
            raise Error("The list of moves must be consistent.")

        # If here, the list of moves must be consistent, so we can finish the
        # initialization of the routine object
        self.moves = moves

        # index of the current move. Starts at -1 to indicate that it is at the
        # init_hold
        self.current_move_index = -1

        # indicates whether we have finished the move and are now in the hold phase
        self.in_hold = True

    def _get_hold_command(self, hold, q, q_dot):
        """Private method for getting the command while in a hold"""

        # check if we are stable in the hold
        if hold.controller.is_stable(q, q_dot):
            # if we are stable, we can continue in the routine. Check to
            # see if that was the last move
            if self.current_move_index == len(self.moves) - 1:
                print "Finished routine!"
                sys.exit(0)
            else:
                # it wasn't the last move, so continue to the next move
                self.current_move_index += 1
                self.in_hold = False

                print "Starting move", self.moves[self.current_move_index].name
                
                # return the command from the move
                return self.get_command(q, q_dot)
        else:
            # if we aren't stable, use the hold controller
            return hold.controller.control(q, q_dot)

    def get_command(self, q, q_dot):
        i = self.current_move_index

        if i == -1:
            # if i is -1, it means we are in the init hold
            return self._get_hold_command(self.init_hold, q, q_dot)
        else:
            # get the current move from the routine
            current_move = self.moves[i]

            if self.in_hold:
                # if we are in the hold component of the move, get the command
                # from the _get_hold_command method
                return self._get_hold_command(current_move.controller.hold_to, q, q_dot)
            else:
                # otherwise, check if the move has finished
                if current_move.controller.is_finished(q, q_dot):
                    # if it has, move to the hold
                    self.in_hold = True
                    hold = current_move.controller.hold_to
                    print "Completed move, stabilizing in hold", hold.name
                    # return the command from the hold
                    return self._get_hold_command(hold, q, q_dot)
                else:
                    # if it hasn't, use the controller for the current move
                    return current_move.controller.control(q, q_dot)
