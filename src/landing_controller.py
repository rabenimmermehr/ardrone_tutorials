# -*- coding: utf-8 -*-
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

        # Create a timer, which is used to wait for the drone to execute the command, until the command to stop is sent
        self.r = rospy.Rate(self.constants.COMMAND_PUBLISH_RATE)

        # Create controllers, which weigh the steering commands according to different variables
        self.findTagController = PID(1.0, 0.3, 0.5)
        self.findTagController.setPoint(0.0)

        # self.centerFrontController = PID(0.8, 0.01, 1.5)
        # The one below worked pretty good
        # self.centerFrontController = PID(0.8, 0.01, 0.5)
        self.centerFrontController = PID (2.0, 0.0, 0.0)
        self.centerFrontController.setPoint(0)

        self.approachFrontController = PID(1.5, 2.0, 2.0)
        self.approachFrontController.setPoint(0)

        # self.centerBottomXController = PID(0.8, 0.5, 3.0)
        # self.centerBottomXController = PID(0.8, 0.5, 1.0)
        self.centerBottomXController = PID(0.8, 0.2, 0.6)
        self.centerBottomXController.setPoint(0)

        # self.centerBottomYController = PID(0.8, 0.5, 3.0)
        # self.centerBottomYController = PID(0.8, 0.5, 1.0)
        self.centerBottomYController = PID(0.8, 0.2, 0.6)
        self.centerBottomYController.setPoint(0)

        self.alignBottomController = PID()
        self.alignBottomController.setPoint(0)


        # Stores if we centered the tag once already (In that case, we can assume we are on a direct path
        # towards the tag and can now move laterally instead of turning in one place)
        self.wasTagCentered = False

        # A flag to stop the loop for when we are done
        self.success  = False

        # Some counters to evaluate how we are doing
        self.searchCounter = 0
        self.rotateCounter = 0
        self.approachCounter = 0
        self.centerCounter = 0
        self.bottomCounter = 0

    def __SignalHandler(self, frame, placeholderArgument):
        '''
            Sets the stopTask flag, so our loops stop when interrupted via keyboard
        '''

        self.stopTask = True

    def __ReceiveNavdata(self, navdata):
        '''
            Stores the incoming navdata
        '''

        self.recentNavdata = navdata


    def BringMeHome(self):
        '''
            Core function to land the drone.
            It will turn the drone till it finds a 3-colored tag witht the front camera, approach this tag until
            it sees a A4-tag on the bottom and will land the drone on this tag
        '''

        # This is the big loop for all the functions. It resets all flags and periodically asks for instructions to be sent to the drone.

        if self.recentNavdata == None:
            print "No navdata available, exiting"
            return

        # Reset flags
        self.stopTask = False
        self.wasTagCentered = False
        self.success = False

        # Reset the controllers
        self.findTagController.setPoint(0)
        self.centerFrontController.setPoint(0)
        self.approachFrontController.setPoint(0)
        self.centerBottomXController.setPoint(0)
        self.centerBottomYController.setPoint(0)
        self.alignBottomController.setPoint(0)

        # Some counters to evaluate how we are doing
        self.searchCounter = 0
        self.rotateCounter = 0
        self.approachCounter = 0
        self.centerCounter = 0
        self.bottomCounter = 0

        # Get the drone up to a good height:
        workingNavdata = self.recentNavdata

        while (workingNavdata.altd < 1450):

            if self.stopTask:
                # In case of keyboard interruption
                break

            workingNavdata = self.recentNavdata
            self.ExecuteCommand(matrix33(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            print "Gaining Altitude"



        # Now we are at a proper height and we can start finding & approaching the tag

        while not self.stopTask and not self.success:

            # Create a copy of the current navdata, so it doesn't get refreshed while we work on it
            workingNavdata = self.recentNavdata

            # Receive the necessary actions
            steeringCommand = self.TellMeWhatToDo(workingNavdata)
            # Send them to the drone
            self.ExecuteCommand(steeringCommand)

        # In case we got stopped via the keyboard, we want to make sure that our last command is to stop the drone from
        # doing anything
        steeringCommand = self.constants.STOP_MOVING
        self.ExecuteCommand(steeringCommand)

        print "Steps searching: %i" % self.searchCounter
        print "Steps rotating: %i" % self.rotateCounter
        print "Steps approaching: %i" % self.approachCounter
        print "Steps centering: %i" % self.centerCounter
        print "Steps centering bottom tag %i " % self.bottomCounter


    def ExecuteCommand(self, steeringCommand):
        '''
            Takes a matrix33, publishes it to the drone, and waits for a moment
        '''

        # Publish the command
        self.publishSteering.publish(steeringCommand)
        # Let the drone actually do that action for a moment
        self.r.sleep()
        # Stop the movement
        #self.publishSteering.publish(self.constants.STOP_MOVING)
        # Wait a moment for the drone to calm down
        #self.r.sleep()

    def TellMeWhatToDo(self, navdata):
        '''
            Gets the current navdata and returns a matrix33 with appropriate steering commands
            Automatically decides which commands to send, based on the available tags
        '''

        # We got 3 cases: No tag visible, front tag visble, bottom tag visible

        # Check where the tags are in the arrays in the navdata (You can check which information the navdata contains from the commandline with "rosmsg show ardrone/Navdata")
        bottomTagIndex = self.GetBottomTagIndex(navdata)
        frontTagIndex = self.GetFrontTagIndex(navdata)

        if navdata.tags_count == 0:
            # No tag visible
            # We want the drone to turn in one place

            print "Searching"
            self.searchCounter+=1

            steeringCommand = matrix33(0.0, 0.0, 0.0, self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)
            return steeringCommand
            #return self.constants.STOP_MOVING

        elif bottomTagIndex > -1:
            # Sweet, the bottom tag is visible!

            print "Centering bottom tag"
            self.bottomCounter+=1

            # We need to check if we are turned the right way above the tag
            currentAngle = navdata.tags_orientation[bottomTagIndex]

            # Actually this step doesn't seem to be necessary so we skip it
            # It should have led to a bigger precision, but mostly the drone is oriented the right way
            return self.LandingPreparations(navdata)

#             if currentAngle > 170.0 and currentAngle < 190.0:
#                 # Ok, so we are turned correctly, we need to center the bottom tag now. If it is centered
#                 # the command to land will be returned
#                 return self.LandingPreparations(navdata)

#             else:
#                 # We need to turn the drone on the spot
#                 return self.AlignBottomTag(navdata)

        else:
            # Only the front tag is visble
            # Now we got 2 cases:
            # - We are still in the spot we started and we are trying to center it (Meaning we are turning on the spot)
            # - We are already on our way towards that tag

            # The x position is represented in values between 0 - 1000 from left to right
            x = navdata.tags_xc[frontTagIndex]

            if x > 450 and x < 550:
                # It is pretty centered, set the flag and start approaching
                self.wasTagCentered = True

                print "Approaching"
                self.approachCounter+=1

                return self.TellMeApproach(navdata)

            else:
                # Check if we already centered it once
                if self.wasTagCentered:
                    # We did? Ok, then lets move laterally

                    print "Centering laterally"
                    self.centerCounter+=1

                    return self.TellMeCenter(navdata)

                else:
                    # The tag hasn't been centered yet, turn on the spot

                    print "Turning"
                    self.rotateCounter+=1
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

        #reducingFactor = self.constants.reduceFactor * navdata.tags_distance[tagIndex]  / self.constants.reduceDistance
        reducingFactor = navdata.tags_distance[tagIndex]  / self.constants.reduceDistance

        if reducingFactor > 1:
            reducingFactor = 1
        sidewardsMovement = reducingFactor * controller_output * self.constants.TAG_CENTER_VELOCITY

        return matrix33(0.0, sidewardsMovement, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def TellMeApproach(self, navdata):
        '''
            Returns a matrix33 whith commands to approach the tag up to a certain distance
        '''
        tagIndex = self.GetFrontTagIndex(navdata)

        # We want the drone to stop at a certain point in front of the tag
        # Also, we want it to go full speed up to controllerDistance away from that point, followed by a
        # distance, where the controller handles the speed (The controller isn't given the actual distance but a float value representing how
        # far away we are. It then returns a factor, with which we multiply our TAG_APPROACH_VELOCITY)

        controller_input = (navdata.tags_distance[tagIndex] - self.constants.desiredDistance) / self.constants.controllerDistance
        controller_output = self.approachFrontController.update(controller_input)

        # The output value needs to be inverted, avoid drastic controller outputs
        controller_output = - self.approachFrontController.avoid_drastic_corrections(controller_output)

        return matrix33(controller_output * self.constants.TAG_APPROACH_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def LandingPreparations(self, navdata):
        '''
            Returns the appropriate commands to center the bottom tag. Once it is centered, it returns the command to land
        '''
        tagIndex = self.GetBottomTagIndex(navdata)
        tagXPosition = navdata.tags_xc[tagIndex]
        tagYPosition = navdata.tags_yc[tagIndex]

        controller_input_y = (tagYPosition - 500.0) / 500.0
        controller_input_x = (tagXPosition - 500.0) / 500.0

        if not (tagYPosition > 400 and tagYPosition < 600) or not (tagXPosition > 400 and tagXPosition < 600):
            # We aren't centered, now we determine in which direction we are more off, so we can deal with that one first

            if (controller_input_y * controller_input_y) > (controller_input_x * controller_input_x):

                controller_output_y = self.centerBottomYController.update(controller_input_y)
               # controller_output_y = self.centerBottomYController.avoid_drastic_corrections(controller_output_y)

                print "Y"
                print controller_input_y
                print controller_output_y

                return matrix33(controller_output_y * self.constants.CENTER_BOTTOM_Y_VELOCITY,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            else:

                controller_output_x = self.centerBottomXController.update(controller_input_x)
              #  controller_output_x = self.centerBottomXController.avoid_drastic_corrections(controller_output_x)

                print "X"
                print controller_input_x
                print controller_output_x

                return matrix33(0.0, controller_output_x * self.constants.CENTER_BOTTOM_X_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        else:
            print "READY"
            #return self.constants.STOP_MOVING
            # Tag is centered in both directions, we are done, we can land!
            self.success = True
            return matrix33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)


    def AlignBottomTag(self, navdata):
        '''
            Returns the command to orient the drone in a 180° angle over the bottom tag
        '''
        tagIndex = self.GetBottomTagIndex(navdata)

        controller_input = navdata.tags_orientation[tagIndex] - 180.0 / 360.0
        controller_output = self.alignBottomController.update(controller_input)
        controller_output = self.alignBottomController.avoid_drastic_corrections(controller_output)

        return matrix33(0.0, 0.0, 0.0, controller_output * self.constants.ALIGN_BOTTOM_TAG_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)

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

    def BringMeCenterRotate(self):
        '''
            A test method that only rotates the drone
        '''

        # Reset flags
        self.stopTask = False
        self.wasTagCentered = False

        self.findTagController.setPoint(0)

        while not self.stopTask and ( self.recentNavdata.tags_xc[self.GetFrontTagIndex(self.recentNavdata)] > 480 or
                                    self.recentNavdata.tags_xc[self.GetFrontTagIndex(self.recentNavdata)] < 520 ) :

            # Create a copy of the current navdata, so it doesn't get refreshed while we work on it
            workingNavdata = self.recentNavdata

            if workingNavdata.tags_count == 0:
                # No tag visible
                # We want the drone to turn in one place

                steeringCommand = matrix33(0.0, 0.0, 0.0, self.constants.FIND_TAG_TURN_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0)

            else:
                # Receive the necessary actions
                steeringCommand = self.TellMeFindTag(workingNavdata)

            self.ExecuteCommand(steeringCommand)

        print "Done"

    def BringMeCenterLateral(self):
        '''
            A test method that only moves the drone laterally
        '''

        # Reset flags
        self.stopTask = False
        self.wasTagCentered = False

        self.centerFrontController.setPoint(0)

        while not self.stopTask:

            # Create a copy of the current navdata, so it doesn't get refreshed while we work on it
            workingNavdata = self.recentNavdata

            # Receive the necessary actions
            steeringCommand = self.TellMeCenter(workingNavdata)
            self.ExecuteCommand(steeringCommand)


    def GetMeAligned(self):
        '''
            Test method that rotates the drone above the bottom tag
        '''

         # Reset flags
        self.stopTask = False
        self.wasTagCentered = False
        self.alignBottomController.setPoint(0)


        while not self.stopTask:

            workingNavdata = self.recentNavdata

            steeringCommand = self.AlignBottomTag(workingNavdata)
            self.ExecuteCommand(steeringCommand)

    def GetMeReady(self):
        '''
            Test method, that centers the drone above the bottom tag
        '''

         # Reset flags
        self.stopTask = False
        self.wasTagCentered = False

        self.centerBottomXController.setPoint(0)
        self.centerBottomYController.setPoint(0)

        while not self.stopTask:
            workingNavdata = self.recentNavdata

            steeringCommand = self.LandingPreparations(workingNavdata)
            self.ExecuteCommand(steeringCommand)


# Set this up as a ROS-Node, so we can subscribe & publish
rospy.init_node('landing_controller')

# Create the controller as a object, so we can use it from the python command line
controller = LandingController()













