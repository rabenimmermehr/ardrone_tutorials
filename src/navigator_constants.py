# Necessary to build the empty steering command
from ardrone_autonomy.msg import matrix33

class NavigatorConstants(object):

    def __init__(self):


        # The command to turn the drone while looking for a tag is sent for this duration
        # Unit is Hz
        self.FIND_TAG_TURN_RATE = 2

        # The velocity with which the drone turns, between 0 and 1, 1 is
        # what a keyboard press sends
        self.FIND_TAG_TURN_VELOCITY = 0.6

        # The command to approach the tag is sent for this duration
        # Unit is Hz, so it moves for only 1/10th of a second
        self.TAG_APPROACH_RATE = 2

        # The velocity with which the drone will move laterally, between 0 and 1, 1 is
        # what a keyboard press sends
        self.TAG_CENTER_VELOCITY = 0.5

        # The velocity with which the drone approaches a tag, between 0 and 1,
        # 1 is equivalent to a keypress
        self.TAG_APPROACH_VELOCITY = 0.6

        # A matrix with an empty command, this is sent whenever the drone needs to stop
        # moving
        self.STOP_MOVING = matrix33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # The code for the 3 striped tag (Green Orange Green):
        self.frontTagType = 0

        # The code for the oriented A4 tag:
        self.bottomTagType = 131072

        # The distance, up to which the tag should be approached, measured in cm
        self.desiredDistance = 180.0

        # The distance, from which point on the controller will regulate the approach speed, measured in cm
        self.controllerDistance = 250.0

        # The distance, from which on the sidewards movements to center the tag will be reduced linearly
        # (See LandingNavigator.CenterTag())
        self.reduceDistance = 250.0

        # A factor to further reduce sidewards movements
        self.reduceFactor = 1