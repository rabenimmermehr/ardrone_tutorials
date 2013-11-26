#!/usr/bin/env python

# The Keyboard  Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This  extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Load the Drone class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
#from drone_controller import BasicDrone
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

# Import the messages, which are to be send
from ardrone_autonomy.msg import matrix33

# Here we define the keyboard map for our  (note that python has no enums, so we use a class)
class KeyMapping(object):
        PitchForward     = QtCore.Qt.Key.Key_E
        PitchBackward    = QtCore.Qt.Key.Key_D
        RollLeft         = QtCore.Qt.Key.Key_S
        RollRight        = QtCore.Qt.Key.Key_F
        YawLeft          = QtCore.Qt.Key.Key_W
        YawRight         = QtCore.Qt.Key.Key_R
        IncreaseAltitude = QtCore.Qt.Key.Key_Q
        DecreaseAltitude = QtCore.Qt.Key.Key_A
        Takeoff          = QtCore.Qt.Key.Key_Y
        Land             = QtCore.Qt.Key.Key_H
        Emergency        = QtCore.Qt.Key.Key_Space


# Our  definition, note that we extend the DroneVideoDisplay class
class Keyboard(DroneVideoDisplay):
        def __init__(self):
                super(Keyboard,self).__init__()

                self.pitch = 0
                self.roll = 0
                self.yaw_velocity = 0
                self.z_velocity = 0

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
        def keyPressEvent(self, event):
                key = event.key()
                steering_matrix = matrix33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                # If we have constructed the drone controller and the key is not generated from an auto-repeating key
                if not event.isAutoRepeat():
                        # Handle the important cases first!
                        if key == KeyMapping.Emergency:
                               steering_matrix.m33 = 1.0
                        elif key == KeyMapping.Takeoff:
                                steering_matrix.m31 = 1.0
                        elif key == KeyMapping.Land:
                                steering_matrix.m32 = 1.0
                        else:
                                # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
                                if key == KeyMapping.YawLeft:
                                        self.yaw_velocity += 1
                                elif key == KeyMapping.YawRight:
                                        self.yaw_velocity += -1

                                elif key == KeyMapping.PitchForward:
                                        self.pitch += 1
                                elif key == KeyMapping.PitchBackward:
                                        self.pitch += -1

                                elif key == KeyMapping.RollLeft:
                                        self.roll += 1
                                elif key == KeyMapping.RollRight:
                                        self.roll += -1

                                elif key == KeyMapping.IncreaseAltitude:
                                        self.z_velocity += 1
                                elif key == KeyMapping.DecreaseAltitude:
                                        self.z_velocity += -1

                        # finally we set the command to be sent. The controller handles sending this at regular intervals
                        steering_matrix.m11 = self.pitch
                        steering_matrix.m12 = self.roll
                        steering_matrix.m13 = self.z_velocity
                        steering_matrix.m21 = self.yaw_velocity
                        pub_steering.publish(steering_matrix)


        def keyReleaseEvent(self,event):
                key = event.key()

                steering_matrix = matrix33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                # If we have constructed the drone  and the key is not generated from an auto-repeating key
                if not event.isAutoRepeat():
                        # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
                        # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
                        if key == KeyMapping.YawLeft:
                                self.yaw_velocity -= 1
                        elif key == KeyMapping.YawRight:
                                self.yaw_velocity -= -1

                        elif key == KeyMapping.PitchForward:
                                self.pitch -= 1
                        elif key == KeyMapping.PitchBackward:
                                self.pitch -= -1

                        elif key == KeyMapping.RollLeft:
                                self.roll -= 1
                        elif key == KeyMapping.RollRight:
                                self.roll -= -1

                        elif key == KeyMapping.IncreaseAltitude:
                                self.z_velocity -= 1
                        elif key == KeyMapping.DecreaseAltitude:
                                self.z_velocity -= -1

                        # finally we set the command to be sent. The controller handles sending this at regular intervals
                        steering_matrix.m11 = self.pitch
                        steering_matrix.m12 = self.roll
                        steering_matrix.m13 = self.z_velocity
                        steering_matrix.m21 = self.yaw_velocity
                        pub_steering.publish(steering_matrix)



# Setup the application
if __name__=='__main__':

        import sys
        # Firstly we setup a ros node, so that we can communicate with the other packages
        rospy.init_node('ardrone_keyboard_')

        # Now we construct our Qt Application and associated s and windows
        app = QtGui.QApplication(sys.argv)

        # Set up the publisher for the steering commands
        pub_steering = rospy.Publisher('ardrone/steering_commands', matrix33)

        # Set up the display
        display = Keyboard()
        display.show()

        # executes the QT application
        status = app.exec_()

        # and only progresses to here once the application has been shutdown
        rospy.signal_shutdown('Great Flying!')
        sys.exit(status)
