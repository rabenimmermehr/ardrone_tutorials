import roslib
import rospy

roslib.load_manifest('ardrone_tutorials')

from ardrone_autonomy.msg import Navdata

def ReceiveData(data):
    print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)

rospy.init_node('minimum_viable_subscriber')
sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)

while not rospy.is_shutdown():
    pass