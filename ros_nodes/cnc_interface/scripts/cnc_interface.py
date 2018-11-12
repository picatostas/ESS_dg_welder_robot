import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from xyz import XYZ

xyz = XYZ()
xyz.startup()

respPositionTopic = None

def cmd_move_callback(msg):
	rospy.loginfo(rospy.get_name() + ": " + str(msg))
	print msg.linear.x, msg.linear.y, msg.linear.z
	xyz.move_to(msg.linear.x, msg.linear.y, msg.linear.z, block_until_complete=True)

def query_position(msg):
	respPositionTopic.publish(Twist(*xyz.p))

def init_ros_listener():
	rospy.init_node('cnc_interface', anonymous=True)
	rospy.Subscriber('cnc_cmd', Twist, cmd_move_callback)
	rospy.Subscriber('query_position', String, query_position_callback)
	respPositionTopic = rospy.Publisher('cnc_position_state', Twist)
	rospy.spin()

init_ros_listener()
