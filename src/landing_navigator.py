# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib;
roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages
from ardrone_autonomy.msg import matrix33 # for steering commands
from ardrone_autonomy.msg import Navdata # for receiving navdatas from the drone, especially the tags

# Import the class containing all the constants
from navigator_constants import NavigatorConstants

# Import stuff for threads
from time import sleep

# Import stuff to kill sript when necessary via ctrl+c
import signal
import sys

# Import the PID controller
from pid_controller import PID

import math



class Landing_Navigator(object):

    def __init__(self):

        # Store the last navdata
        self.recentNavdata = None

        # Subscribe to the /ardrone/navdata topic, so we get updated about the tags
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)

        # Allow the navigator to send commands to the controller
        self.pubSteering = rospy.Publisher('ardrone/steering_commands', matrix33)

        # Get some constants
        self.constants = NavigatorConstants()

        # Create variables to store the index of the corresponding tags

        self.bottomTag = -1
        self.frontTag = -1

        # A flag to check if the current task should be stopped
        self.stopTask = False

        # Attach the keyboard-listener (to stop the script when necessary)
        signal.signal(signal.SIGINT, self.signal_handler)


    def signal_handler(signal, frame, placeholderArgument):
        ''' Handles keyboard-interruptions, I honestly don't know why it needs
            3 arguments, but placeholderArgument is just a placeholder to make
            it work
        '''
        signal.stopTask = True


    def ReceiveNavdata(self, navdata):
        ''' Recieves the incoming navdata and stores it'''

        # !!!! TODO Check for possible collisions, when both the subscriber and the code try to access the variable
        self.recentNavdata = navdata

        self.frontTag = -1
        self.bottomTag = -1



    def getFrontTagIndex(self, navdata):
        '''
            returns the index in the current navdata of the front tag
            returns -1 if none is found
        '''
        for tagtype in navdata.tags_type:
            if tagtype == self.constants.frontTagType:
                return navdata.tags_type.index(tagtype)
        return -1

    def getBottomTagIndex(self, navdata):
        '''
            returns the index in the given navdata of the bottom tag
            returns -1 if none is found
        '''
        for tagtype in navdata.tags_type:
            if tagtype == self.constants.bottomTagType:
                return navdata.tags_type.index(tagtype)
        return -1

    def GetTurnValue(self, xc):
        ''' Deprecated, is now handled by the PID-controllers
            Gets the x-position of the tag and calculates the appropriate steering command
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

        myNavdata = self.recentNavdata
        if myNavdata == None:
            raise Exception('No navdata available')
        tagIndex = self.getFrontTagIndex(myNavdata)

        # Necessary to wait for the drone to turn
        r = rospy.Rate(self.constants.FIND_TAG_TURN_RATE)


        # Navdata is available and no tag is visible
        if tagIndex == -1:
            print "No tag found, turning"
            # Turn the drone
            self.pubSteering.publish(matrix33(0.0, 0.0, 0.0, self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0))
            # Wait a moment
            r.sleep()
            # Stop turning
            self.pubSteering.publish(self.constants.STOP_MOVING)
            return False

        # So, now we found a tag, lets try to center it in the field of view
        else:

            controller_input = (myNavdata.tags_xc[tagIndex] - 500.0) / 500.0

            controller_output = self.findController.update(controller_input)
            print "Navdata %i" % myNavdata.tags_xc[tagIndex]
            print "In %f" % controller_input
            print "Out %f " % controller_output

            # The necessary correction is little, we assume it is centered and we can exit
            if math.fabs(controller_output) < 0.2 :
                return True

            # Sometimes the output might be bigger than |1.0|, which we want to avoid
            controller_output = self.findController.avoid_drastic_corrections(controller_output)

            # In case it isn't centerd, turn the drone accordingly
            turnCommand = matrix33(0.0, 0.0, 0.0, controller_output * self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.pubSteering.publish(turnCommand)
            # Wait a moment
            r.sleep()
            # Stop turning
            self.pubSteering.publish(self.constants.STOP_MOVING)
            return False




    def CenterTag(self):
        ''' Moves the drone laterally in order to center it
            Throws an exception if no tag was found
            @return True tag is centered
        '''
        # Create a copy of the current Navdata, so it doesn't get updated
        # while running this method
        myNavdata = self.recentNavdata
        tagIndex = self.getFrontTagIndex(myNavdata)

        if myNavdata == None:
            raise Exception('No navdata available')

        # No tag? Raise error
        if tagIndex == -1:
            raise Exception('No Tag available')


        # Create the timer
        r = rospy.Rate(self.constants.TAG_APPROACH_RATE)

        # Tag, but not centered? Move laterally
        # Ask the controller for a factor:

        controller_input = (myNavdata.tags_xc[tagIndex] - 500.0) / 500.0
        controller_output = self.centerController.update(controller_input)

        controller_output = self.centerController.avoid_drastic_corrections(controller_output)

        # Thing is, the corrections done by the controller will have a pretty huge impact once we are
        # close to the tag, therefore we reduce those corrections depending on the distance from the tag

        reducingFactor = self.constants.reduceFactor * myNavdata.tags_distance[tagIndex]  / self.constants.reduceDistance
        if reducingFactor > 1:
            reducingFactor = 1
        sidewardsMovement = reducingFactor * controller_output * self.constants.TAG_CENTER_VELOCITY

        if myNavdata.tags_xc[tagIndex] > 480 and myNavdata.tags_xc[tagIndex] < 520:
            return True

        else:
            print "CenterTag: Trying to center tag"
            steering_matrix = matrix33(0.0, sidewardsMovement, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            # Send the command
            self.pubSteering.publish(steering_matrix)
            # Wait for a moment
            r.sleep()
            # Stop the drone
            self.pubSteering.publish(self.constants.STOP_MOVING)

            return False


    # Tells the drone to approach the tag
    def ApproachTag(self):
        ''' Tells the drone to approach the tag up to distance of 1.3 m
            @return True if it is sufficiently close, False otherwise
        '''


        # Create a copy of the current Navdata, so it doesn't get updated
        # while running this method
        myNavdata = self.recentNavdata
        tagIndex = self.getFrontTagIndex(myNavdata)



        if myNavdata == None:
            raise Exception('No navdata available')

        if not (self.getBottomTagIndex(myNavdata) == -1):
            print "Botton tag is visible, my duty is done"
            return True

        # No tag? Raise error
        if tagIndex == -1:
            raise Exception('No Tag available')

        # Necessary to wait for the drone to move
        r = rospy.Rate(self.constants.TAG_APPROACH_RATE)

        # We want the drone to stop 130cm in front of the tag
        # Also, we want it to go full speed up to controllerDistance away from that point, followed by a
        # distance, where the controller handles the speed

        controller_input = (myNavdata.tags_distance[tagIndex] - self.constants.desiredDistance) / self.constants.controllerDistance
        controller_output = self.approachController.update(controller_input)
        #print "input: %f" % controller_input
        #print "output: %f" % controller_output

        # The output value needs to be inverted
        controller_output = - self.approachController.avoid_drastic_corrections(controller_output)


        # If the distance is bigger than the desiredDistance
        if myNavdata.tags_distance[tagIndex] >= self.constants.desiredDistance:

            print "Approaching"

            steering_matrix = matrix33(controller_output * self.constants.TAG_APPROACH_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            # Send the command
            self.pubSteering.publish(steering_matrix)
            # Wait for a moment
            r.sleep()
            # Stop the drone
            self.pubSteering.publish(self.constants.STOP_MOVING)

            return False

        # Drone is sufficiently close to the tag
        else:
            print "Drone is close enough to the tag, but hasn't detected the bottom tag yet"
            return True


    def OrientateOnTag(self):

        myNavdata = self.recentNavdata
        tagIndex = self.getBottomTagIndex(myNavdata)

        if tagIndex == -1:
            raise Exception("No bottom tag detected")

        print myNavdata.tags_orientation[tagIndex]
#         controller_input = myNavdata.tags_orientation[self.getBottomTagIndex(myNavdata)]
#         controller_output = self.bottomTagController.update(input)
#         print controller_input
#         print controller_output

#         controller_output = self.bottomTagController.avoid_drastic_corrections(controller_output)







    def findTagThread(self):
        ''' A thread that creates a PID controller, recursively calls the "FindTag()" method
            and stops once it has found and centered it
        '''


        # Set up the controller, returns factors with which the TAG_CENTER_VELOCITY is multiplied
        # in order to find and center the tag
        self.findController = PID()
        # 0 represents center
        self.findController.setPoint(0)

        while not self.FindTag():

            # A flag which is set via ctrl+c
            if self.stopTask:
                print""
                print "Interrupting task"
                self.stopTask = False
                break

            sleep(1)


        print "Tag is centered"


    def approachTagThread(self):
        ''' A thread that creates a PID controller,
            tries to approach the tag via calling the "ApproachTag()" and "CenterTag()"
            whenever necessary
        '''

        # Create 2 controllers, which return factors with which the corresponding velocities ar
        # multiplied
        self.approachController = PID(0.4, 0.0, 2.0)
        #self.centerController = PID(1.0, 0.5, 0.0)
        self.centerController = PID(0.4, 0.0, 2.0)

        # Set the desired value to 0, all other distance-related calculations happen in the method
        self.approachController.setPoint(0)

        # Set the target value to 0, which represents center
        self.centerController.setPoint(0)

        tagMissingCounter = 0

        while not self.stopTask and (tagMissingCounter < 10):


            try:


                if self.ApproachTag():
                    break

                # Reset the counter
                tagMissingCounter = 0


                # Wait till the drone is centered again
                while( not self.CenterTag() and not self.stopTask):
                    print "Centering"
                    pass



                # A flag which is set via ctrl+C to stop the current loop
                if self.stopTask:
                    print ""
                    print "Interrupting task"
                    self.stopTask = False
                    break

            except:
                # Maybe the tag just wasn't detected in the last frame, but is still there
                # Check how long the tag is missing:
#                 if tagMissingCounter > 3:
#                     print "Couldn't find tag"
#                     break
                # Wait a moment
                print "Waiting ... %i " % tagMissingCounter
                r = rospy.Rate(self.constants.FIND_TAG_TURN_RATE)
                r.sleep()
                tagMissingCounter = tagMissingCounter + 1



        print("Tag is approached")


# Set up a ROS node
rospy.init_node('landing_navigator')

# Create the navigator
navigator = Landing_Navigator()
