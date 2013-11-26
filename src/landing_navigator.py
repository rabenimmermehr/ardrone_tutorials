# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy


# Imprt the messages
from ardrone_autonomy.msg import matrix33 # for steering commands
from ardrone_autonomy.msg import Navdata # for receiving navdatas from the drone, especially the tags

# Some constants
FIND_TAG_TURN_RATE = 10 # The command to turn the drone while looking for a tag is sent for this duration

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
        # !!!! TODO Check for probable collisions
        self.lastNavdata = navdata


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
            self.pubSteering.publish(matrix33(0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
            # Wait a moment
            r.sleep()
            # Stop turning
            self.pubSteering.publish(stopTurning)
            return False

        # So, now we found a tag, lets try to center it in the field of view
        elif self.lastNavdata != None and self.lastNavdata.tags_count == 1:

            # Tag is too far to the left
            if self.lastNavdata.tags_xc[0] <= 400:
                print "Tag is too far left"
                # Turn the drone
                self.pubSteering.publish(matrix33(0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
                # Wait a moment
                r.sleep()
                # Stop turning
                self.pubSteering.publish(stopTurning)
                return False

            # Tag is too far to the right
            elif self.lastNavdata.tags_xc[0] >= 600:
                print "Tag is too far right"
                # Turn the drone
                self.pubSteering.publish(matrix33(0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
                # Wait a moment
                r.sleep()
                # Stop turning
                self.pubSteering.publish(stopTurning)
                return False

            # Tag is centered
            else:
                return True

        # In case I forgot any cases / No navdata available
        return False

# Set up a ROS node
rospy.init_node('landing_navigator')

# Create the navigator
navigator = Landing_Navigaor()

centeredTag = False

r = rospy.Rate(2)

while True:
    # Find and center the tag:
    while not centeredTag:
        centeredTag = navigator.FindTag()
        r.sleep()

    print "Tag is centered"
    centeredTag = False
    raw_input("Press Enter to look again")
