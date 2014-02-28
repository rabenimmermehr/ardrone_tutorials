# Import stuff to kill sript when necessary via ctrl+c
import signal
import sys

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages
from ardrone_autonomy.msg import matrix33 # for steering commands
from ardrone_autonomy.msg import Navdata # for receiving navdatas from the drone, especially the tags

# Import the class containing all the constants
from navigator_constants import NavigatorConstants

# Import stuff for threads
from threading import Thread
from time import sleep


# Listener, to make the script stoppable via ctrl + c
def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class Landing_Navigator(object):

    def __init__(self):

        # Store the last navdata
        self.lastNavdata = None

        # Subscribe to the /ardrone/navdata topic, so we get updated about the tags
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)

        # Allow the navigator to send commands to the controller
        self.pubSteering = rospy.Publisher('ardrone/steering_commands', matrix33)

        # Get those constants
        self.constants = NavigatorConstants()

    def ReceiveNavdata(self, navdata):
        ''' Recieves the incoming navdata and stores it'''

        # !!!! TODO Check for possible collisions, when both the subscriber and the code try to access the variable
        self.lastNavdata = navdata


    def GetTurnValue(self, xc):
        '''Gets the x-position of the tag and calculates the appropriate steering command
           @param xc The x position of the tag in the range from 0 - 1000, from left to right
           @return A matrix33 with appropriate steering commands or True, if the tag is centered

        '''

        # !!!!! TODO Implement controller here, so that the returned matrix is in relation
        #    to the position of the tag

        # Turns the drone, because no tag was found
        if xc == -1:
            return matrix33(0.0, 0.0, 0.0, self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Tag is too far left
        elif xc >= 0 and xc <= 460:
            return matrix33(0.0, 0.0, 0.0, self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Tag is too far right
        elif xc >=  540 and xc <= 1000:
            return matrix33(0.0, 0.0, 0.0, -self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Tag is centered
        elif 460 < xc and xc < 540:
            return True

    def FindTag(self):
        ''' Keeps the drone in one position, but rotates it
            for a short moment,trying to find and center a tag
            The direction depends on if a tag is visible.
            @return False if the tag wasn't centered, True if it was
        '''

        # Necessary to wait for the drone to turn
        r = rospy.Rate(self.constants.FIND_TAG_TURN_RATE)

        # Navdata is available and no tag is visible
        if self.lastNavdata != None and self.lastNavdata.tags_count == 0:
            print "No tag found, turning"
            # Turn the drone
            self.pubSteering.publish(self.GetTurnValue(-1))
            # Wait a moment
            r.sleep()
            # Stop turning
            self.pubSteering.publish(self.constants.STOP_MOVING)
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
                self.pubSteering.publish(self.constants.STOP_MOVING)
                return False

        # In case I forgot any cases / No navdata available
        return False


    def CenterTag(self):
        ''' Moves the drone laterally in order to center it
            Throws an exception if no tag was found
            @return True tag is centered
        '''
        # Create a copy of the current Navdata, so it doesn't get updated
        # while running this method
        myNavdata = self.lastNavdata

        # No tag? Raise error
        if myNavdata.tags_count == 0:
            raise Exception('No Tag available')


        # Create the timer
        r = rospy.Rate(self.constants.TAG_APPROACH_RATE)

        # Tag, but not centered? Move laterally

        # Too far left
        if(myNavdata.tags_xc[0] <= 480 and myNavdata.tags_xc[0] >= 0 ):

            steering_matrix = matrix33(0.0, self.constants.TAG_CENTER_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            # Send the command
            self.pubSteering.publish(steering_matrix)
            # Wait for a moment
            r.sleep()
            # Stop the drone
            self.pubSteering.publish(self.constants.STOP_MOVING)

            return False

        # Too far right
        elif(myNavdata.tags_xc[0] >= 520 and myNavdata.tags_xc[0] <= 1000):

            steering_matrix = matrix33(0.0, -self.constants.TAG_CENTER_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            # Send the command
            self.pubSteering.publish(steering_matrix)
            # Wait for a moment
            r.sleep()
            # Stop the drone
            self.pubSteering.publish(self.constants.STOP_MOVING)

            return False

        # Centered
        else:
            return True

    # Tells the drone to approach the tag
    def ApproachTag(self):
        ''' Tells the drone to approach the tag up to distance of 1.3 m
            @return True if it is sufficiently close, False otherwise
        '''

        # Create a copy of the current Navdata, so it doesn't get updated
        # while running this method
        myNavdata = self.lastNavdata

        # No tag? Raise error
        if myNavdata.tags_count == 0:
            raise Exception('No Tag available')



        # Necessary to wait for the drone to move
        r = rospy.Rate(self.constants.TAG_APPROACH_RATE)

        # Tag is visible and centered
        if (myNavdata.tags_count == 1 and self.GetTurnValue(myNavdata.tags_xc[0]) == True):


            # If the distance is bigger than 30 cm !!!! TODO !!!! Check if 0.30 actually is 30cm
            if myNavdata.tags_distance[0] >= 1.30:
                steering_matrix = matrix33(self.constants.TAG_APPROACH_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                # Send the command
                self.pubSteering.publish(steering_matrix)
                # Wait for a moment
                r.sleep()
                # Stop the drone
                self.pubSteering.publish(self.constants.STOP_MOVING)

                return False

            # Drone is sufficiently close to the tag
            elif self.lastNavdata.tags_distance[0] <= 1.30:
                return True

        # Tag is visible but not centered
            else:
                print "Tag is not centered"

    def findTagThread(self):
        ''' A thread that recursively calls the "FindTag()" method
            and stops once it has found and centered it
        '''

        # The function which gets put into the thread
        def threadedLoop(navigator):

            while not navigator.FindTag():
                sleep(1);
                print("Tag not centered")

        # Creating the thread
        thread = Thread(target = threadedLoop, args = (self,))
        thread.start()
        thread.join()
        print("Tag is centered")


    def approachTagThread(self):
        ''' A thread that tries to approach the tag via calling the "ApproachTag()" and "CenterTag()"
            whenever necessary
        '''
        # The function which gets put into the thread
        def threadedLoop(navigator):


            while not navigator.ApproachTag():
                if navigator.CenterTag():
                   pass
                else:
                    navigator.CenterTag()

        # Creating the thread
        thread = Thread(target = threadedLoop, args = (self,))
        thread.start()
        thread.join()
        print("Tag is approached")


# Set up a ROS node
rospy.init_node('landing_navigator')

# Create the navigator
navigator = Landing_Navigator()
'''
centeredTag = False

# Check at a rate of 2 Hz
r = rospy.Rate(2)
'''
#try:
'''
while True:
    # Find and center the tag:
    while not centeredTag:
        centeredTag = navigator.FindTag()
        r.sleep()

    print "Tag is centered"
    centeredTag = False
    raw_input("Press Enter to look again")
'''
'''except:
    print "Stopping"
    sys.exit(1)
'''

