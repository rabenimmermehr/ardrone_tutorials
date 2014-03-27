#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist      # for sending commands to the drone
from std_msgs.msg import Empty           # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from ardrone_autonomy.msg import matrix33# for steering commands

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):
    def ReceiveNavdata(self,navdata):
            # Although there is a lot of data in this packet, we're only interested in the state at the moment
            self.status = navdata.state

    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        if(self.status == DroneStatus.Landed):
                    self.pubTakeoff.publish(Empty())

    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())

    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        self.pubReset.publish(Empty())

    def SetCommand(self, steering_matrix):
        # m11 = pitch
        # m12 = roll
        # m13 = z_velocity
        # m21 = yaw_velocity
        # m22 = ...
        # m23 = ...
        # m31 = Takeoff
        # m32 = Land
        # m33 = Emergency

        if not steering_matrix.m31 == 0.0:
            self.SendTakeoff()

        if not steering_matrix.m32 == 0.0:
            self.SendLand()

        if not steering_matrix.m33 == 0.0:
            self.SendEmergency()

        self.command.linear.x = steering_matrix.m11
        self.command.linear.y = steering_matrix.m12
        self.command.linear.z = steering_matrix.m13
        self.command.angular.z = steering_matrix.m21

        print self.command
        print "pitch/m11: %f" % steering_matrix.m11
        print "roll/m12: %f" % steering_matrix.m12
        print "z_velocity/m13: %f" % steering_matrix.m13
        print "yaw_velocity/m21: %f" % steering_matrix.m21
        print "-------------------------------------------------------"

    def SendCommand(self,event):
        # The previously set command is then sent out periodically if the drone is flying
        if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
            self.pubCommand.publish(self.command)
            pass


    def __init__(self):
        # Holds the current drone status
        self.status = -1

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
        self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)



# Set up the ros node, so that one controller can take commands from multiple sources
rospy.init_node('ardrone_controller')

#Create the controller
controller = BasicDroneController()

#Listen for steering-commands
steering_commands = rospy.Subscriber('/ardrone/steering_commands', matrix33, controller.SetCommand)

# keep the node running
while not rospy.is_shutdown():
    pass
