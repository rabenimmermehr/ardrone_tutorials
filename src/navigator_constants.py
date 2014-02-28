# Necessary to build the empty steering command
from ardrone_autonomy.msg import matrix33

class NavigatorConstants(object):

    def __init__(self):


        # The command to turn the drone while looking for a tag is sent for this duration
        # Unit is Hz, so it turns only for 1/10th of a second
        self.FIND_TAG_TURN_RATE = 2

        # The velocity with which the drone turns, between 0 and 1, 1 is
        # what a keyboard press sends
        self.FIND_TAG_TURN_VELOCITY = 0.5

        # The command to approach the tag is sent for this duration
        # Unit is Hz, so it moves for only 1/10th of a second
        self.TAG_APPROACH_RATE = 2

        # The size of the angle the drone will make sideways, between 0 and 1, 1 is
        # what a keyboard press sends
        self.TAG_CENTER_VELOCITY = 0.2

        # The velocity with which the drone approaches a tag, between 0 and 1,
        # 1 is equivalent to a keypress
        self.TAG_APPROACH_VELOCITY = 0.5

        # A matrix with an empty command, this is sent whenever the drone needs to stop
        # moving
        self.STOP_MOVING = matrix33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
