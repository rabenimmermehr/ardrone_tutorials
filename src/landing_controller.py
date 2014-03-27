# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib;
roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages
from ardrone_autonomy.msg import matrix33 # for steering commands
from ardrone_autonomy.msg import Navdata # for receiving navdatas from the drone, especially the tags

# Import the constants
from landing_constants import LandingConstants

# Import the PID controller
from pid_controller import PID

# Import stuff to kill sript when necessary via ctrl+c
import signal
import sys



class LandingController(object):

    def __init__(self):

        # Stores the most recent navdata
        self.recentNavdata = None

        # Subscribe to the /ardrone/navdata topic, so we get updated whenever new information arrives from the drone
        self.subscribeNavdata = rospy.Subscriber('ardrone/navdata', Navdata, self.__ReceiveNavdata)

        # Allow us to publish to the steering topic
        self.publishSteering = rospy.Publisher('ardrone/steering_commands', matrix33)

        # Store the constants
        self.constants = LandingConstants()

        # The flag which we set via ctrl + c, when we want to stop the current loop
        self.stopTask = False

        # Attach the keyboard listener
        signal.signal(signal.SIGINT, self.__SignalHandler)


        # Create controllers, which weigh the steering commands according to different variables
        self.findTagController = PID(3.0, 0.0, 1.0)
        self.findTagController.setPoint(0.0)

        self.centerFrontController = PID(0.4, 0.0, 2.0)
        self.centerFrontController.setPoint(0)

        self.approachFrontController = PID(0.4, 0.0, 2.0)
        self.approachFrontController.setPoint(0)

        # Stores if we centered the tag once already (In that case, we can assume we are on a direct path
        # towards the tag and can now move laterally instead of turning in one place)
        self.wasTagCentered = False

    def __SignalHandler(self, frame, placeholderArgument):
        '''
            Sets the stopTask flag, so our loops stop
        '''

        self.stopTask = True

    def __ReceiveNavdata(self, navdata):
        '''
            Stores the incoming navdata, resets the indizes for the tags
        '''

        self.recentNavdata = navdata


    def BringMeHome(self):

        if self.recentNavdata == None:
            print "No navdata available, exiting"
            return

        r = rospy.Rate(self.constants.COMMAND_PUBLISH_RATE)

        # Reset flags
        self.stopTask = False
        self.wasTagCentered = False

        # Reset the controllers
        self.findTagController.setPoint(0)
        self.centerFrontController.setPoint(0)
        self.approachFrontController.setPoint(0)

        while not self.stopTask:

            # Create a copy of the current navdata, so it doesn't get refreshed while we work on it
            workingNavdata = self.recentNavdata

            # Receive the necessary actions
            steeringCommand = self.TellMeWhatToDo(workingNavdata)

            # Publish them
            self.publishSteering.publish(steeringCommand)
            # Let the drone actually do that action for a moment
            r.sleep()
            # Stop the movement
            self.publishSteering.publish(self.constants.STOP_MOVING)



    def TellMeWhatToDo(self, navdata):

        # We got 3 cases: No tag visible, front tag visble, bottom tag visible

        if navdata.tags_count == 0:
            # No tag visible
            # We want the drone to turn in one place

            steeringCommand = matrix33(0.0, 0.0, 0.0, self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
            return steeringCommand

        else:
            # Ok, so at least 1 tag is visble
            # We need to find out which tag has which index in the arrays

            bottomTagIndex = self.GetBottomTagIndex(navdata)
            frontTagIndex = self.GetFrontTagIndex(navdata)

            if bottomTagIndex > -1:
                # Sweet, the bottom tag is visible!

                # We need to check if we are turned the right way above the tag
                currentAngle = navdata.tags_orientation[bottomTagIndex]

                if currentAngle > 170.0 and currentAngle < 190.0:
                    # Ok, so we are turned correctly, we need to center the bottom tag now. If it is centered
                    # the command to land will be sent
                    return self.LandingPreparations(navdata)

                else:
                    # We need to turn the drone on the spot
                    return self.AlignBottomTag(navdata)


            else:
                # Only the front tag is visble
                # Now we got 2 cases:
                # - We are still in the spot we started and we are trying to center it (Meaning we are turning on the spot)
                # - We are already on our way towards that tag

                # The x position is represented in values between 0 - 1000 from left to right
                x = navdata.tags_xc[frontTagIndex]

                if x > 480 and x < 520:
                    # It is pretty centered, set the flag and start approaching
                    self.wasTagCentered = True
                    print "Approaching"
                    return self.TellMeApproach(navdata)

                else:
                    # Check if we already centered it once
                    if self.wasTagCentered:
                        # We did? Ok, then lets move laterally
                        print "Centering laterally"
                        return self.TellMeCenter(navdata)

                    else:
                        # The tag hasn't been centered yet, turn on the spot
                        print "Turning"
                        return self.TellMeFindTag(navdata)



    def TellMeFindTag(self, navdata):
        '''
            Returns a matrix33 with appropriate commands to center the front tag in the field of view,
            where those commands will turn the drone on the spot
        '''

        tagIndex = self.GetFrontTagIndex(navdata)

        # We feed the controller with values between -1 and 1, and we receive factors, with which we multiply the speed at which the
        # drone is turned
        controller_input = (navdata.tags_xc[tagIndex] - 500.0) / 500.0
        controller_output = self.findTagController.update(controller_input)
        # Sometimes the controller might want to do some drastic actions, which we want to avoid
        controller_output = self.findTagController.avoid_drastic_corrections(controller_output)

        return matrix33(0.0, 0.0, 0.0, controller_output * self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)

    def TellMeCenter(self, navdata):
        '''
            Returns a matrix33 with appropriate commands to center the front tag in the field of view,
            where those commands will move the drone laterally
        '''
        tagIndex = self.GetFrontTagIndex(navdata)

        controller_input = (navdata.tags_xc[tagIndex] - 500.0) / 500.0
        controller_output = self.centerFrontController.update(controller_input)

        controller_output = self.centerFrontController.avoid_drastic_corrections(controller_output)

        # Thing is, the corrections done by the controller will have a pretty huge impact once we are
        # close to the tag, therefore we reduce those corrections depending on the distance from the tag

        reducingFactor = self.constants.reduceFactor * navdata.tags_distance[tagIndex]  / self.constants.reduceDistance
        if reducingFactor > 1:
            reducingFactor = 1
        sidewardsMovement = reducingFactor * controller_output * self.constants.TAG_CENTER_VELOCITY

        return matrix33(0.0, sidewardsMovement, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def TellMeApproach(self, navdata):
        '''
            Returns a matrix33 whith commands to approach the tag
        '''
        tagIndex = self.GetFrontTagIndex(navdata)

        # We want the drone to stop at a certain point in front of the tag
        # Also, we want it to go full speed up to controllerDistance away from that point, followed by a
        # distance, where the controller handles the speed

        controller_input = (navdata.tags_distance[tagIndex] - self.constants.desiredDistance) / self.constants.controllerDistance
        controller_output = self.approachFrontController.update(controller_input)

        # The output value needs to be inverted
        controller_output = - self.approachFrontController.avoid_drastic_corrections(controller_output)

        return matrix33(controller_output * self.constants.TAG_APPROACH_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


    def GetBottomTagIndex(self, navdata):
        '''
            returns the index in the given navdata of the bottom tag
            returns -1 if none is found
        '''
        for tagtype in navdata.tags_type:
            if tagtype == self.constants.bottomTagType:
                return navdata.tags_type.index(tagtype)
        return -1

    def GetFrontTagIndex(self, navdata):
        '''
            returns the index in the current navdata of the front tag
            returns -1 if none is found
        '''
        for tagtype in navdata.tags_type:
            if tagtype == self.constants.frontTagType:
                return navdata.tags_type.index(tagtype)
        return -1


# Set this up as a ROS-Node, so we can subscribe & publish
rospy.init_node('landing_controller')

# Create the controller as a object, so we can use it from the python command line
controller = LandingController()













