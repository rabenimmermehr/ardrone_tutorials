# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy


# Imprt the messages
from ardrone_autonomy.msg import matrix33 # for steering commands
from ardrone_autonomy.msg import Navdata # for receiving navdatas from the drone, especially the tags

# Some constants
FIND_TAG_TURN_RATE = 10 # The command to turn the drone while looking for a tag is sent for this duration
                        # Unit is Hz, so it turns only for 1/10th of a second

MATRIX_TURN_VALUE = 0.5 # The intensitiy at which rate the drone turns, between 0 and 1, 1 is
                        # what a keyboard press sends

class Landing_Navigaor(object):

    def __init__(self):

        # Store the last navdata
        self.lastNavdata = None

        # Subscribe to the /ardrone/navdata topic, so we get updated about the tags
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)

        # Allow the navigator to send commands to the controller
        self.pubSteering = rospy.Publisher('ardrone/steering_commands', matrix33)

    # Handles the incoming data
    def ReceiveNavdata(self, navdata):
        # !!!! TODO Check for possible collisions, when both the subscriber and the code try to access the variable
        self.lastNavdata = navdata

    # Gets the x-position of the tag, returns a matrix with appropriate steering commands
    def GetTurnValue(self, xc):

        # !!!!! TODO Implement controller here, so that the returned matrix stands in relation
        #  to the position of the tag


        # Turns the drone, because no tag was found
        if xc == -1:
            return matrix33(0.0, 0.0, 0.0, MATRIX_TURN_VALUE, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Tag is too far left
        elif xc >= 0 and xc <= 460:
            return matrix33(0.0, 0.0, 0.0, MATRIX_TURN_VALUE, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Tag is too far right
        elif xc >=  540 and xc <= 1000:
            return matrix33(0.0, 0.0, 0.0, -MATRIX_TURN_VALUE, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Tag is centered
        elif 460 < xc and xc < 540:
            return true

    # Keeps the drone in one position, but rotates it, until a tag is found and centered in the field of view
    def FindTag(self):

        # Necessary to wait for the drone to turn
        r = rospy.Rate(FIND_TAG_TURN_RATE)

        # The command to stop the drone turning
        stopTurning = matrix33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Navdata is available and no tag is visible
        if self.lastNavdata != None and self.lastNavdata.tags_count == 0:
            print "No tag found, turning"
            # Turn the drone
            self.pubSteering.publish(self.GetTurnValue(-1))
            # Wait a moment
            r.sleep()
            # Stop turning
            self.pubSteering.publish(stopTurning)
            return False

        # So, now we found a tag, lets try to center it in the field of view
        elif self.lastNavdata != None and self.lastNavdata.tags_count == 1:

            turnCommand = self.GetTurnValue(self.lastNavdata.tags_xc[0])

            # In case "turnCommand" is true, the tag is centered
            if turnCommand == True:
                return True
            # In case it isn't centerd, turn the drone accordingly
            else:
                self.pubSteering.publish(turnCommand)
                # Wait a moment
                r.sleep()
                # Stop turning
                self.pubSteering.publish(stopTurning)
                return False

        # In case I forgot any cases / No navdata available
        return False

    # Tells the drone to approach the tag
    def approachTag(self):
        # Necessary to wait for the drone to move
        r = rospy.Rate(FIND_TAG_TURN_RATE)

        # The command to stop the drone moving
        stopMoving = matrix33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


# Set up a ROS node
rospy.init_node('landing_navigator')

# Create the navigator
navigator = Landing_Navigaor()

centeredTag = False

# Check at a rate of 2 Hz
r = rospy.Rate(2)

try:
    while True:
        # Find and center the tag:
        while not centeredTag:
            centeredTag = navigator.FindTag()
            r.sleep()

        print "Tag is centered"
        centeredTag = False
        raw_input("Press Enter to look again")
except:
    print "Stopping"
    sys.exit(1)
