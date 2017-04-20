import math, tf, rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray


class Node:
	def __init__():
		self.


	def function():
		pass


class FrontierNode:
	def __init__():
		self.

	def function():
		pass

class Frontier:
	def __init__():
		self.

	def function():
		pass



if __name__ == '__main__':
	rospy.init_node('Lab5')

	#subscribers
	rospy.Subscriber("/odom", Odometry, odomCallBack)
	rospy.Subscriber("/map", OccupancyGrid, mapCallBack, queue_size = 1)
	rospy.Subscriber('/move_base/status', GoalStatusArray, queue_size = 1)

	#publishers
	twistPub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
	goalPub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size = 1)