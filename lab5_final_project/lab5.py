import math, tf, rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

AngularSpeed = 1
expandThreshold = 60
expandBuffer = 0.125
expandedGridRes = 0.2


class Node:

	def __init__(self, x, y, new):
		self.x = x
		self.y = y
		self.parent = (0,0)
		self.cost = 0
		self.intensity = new
		
	#makes sure the node is within the bounds of the map
	def inBounds(self, node):
		(x,y) = node
		x = x - gridOrigin.position.x
		y = y - gridOrigin.position.y
		return 0 <= x < (width/(expandedGridRes/res))*expandedGridRes and 0 <= y < (height/(expandedGridRes/res))*expandedGridRes

	#finds and returns all neighbors (including diagonals)
	def neighbors(self):
		neighbors = [(self.x+expandedGridRes, self.y), (self.x, self.y-expandedGridRes), (self.x-expandedGridRes, self.y), (self.x,self.y+expandedGridRes),(self.x+expandedGridRes,self.y+expandedGridRes),(self.x-expandedGridRes,self.y+expandedGridRes),(self.x-expandedGridRes,self.y-expandedGridRes),(self.x+expandedGridRes,self.y-expandedGridRes)]
		neighbors = filter(self.inBounds, neighbors)
		neighbors = filter(open, neighbors)
		return neighbors

	def isFrontier(self):
		if(self.intensity == -1):
			for next in self.neighbors:
				if(next.intensity == 0):
					return True

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


def mapCallBack(msg):
	print "grabbing map"
	global grid
	global width
	global height
	global res
	global gridOrigin
	global newMap
	grid = []
	width = msg.info.width
	height = msg.info.height
	res = msg.info.resolution
	print "grid res is %f m" % res
	print "width %f height %f" % (width,height)
	countCol = 0
	countRow = 0
	startX = msg.info.origin.position.x
	startY = msg.info.origin.position.y
	gridOrigin = msg.info.origin
	print "0,0   %f  %f " % (startX,startY)
	count = 0
	for new in msg.data:
		x = (countCol)*res + gridOrigin.position.x
		y = (countRow)*res + gridOrigin.position.y
		if(countCol == width): #hit the end of the column
			countRow = countRow + 1
			countCol = 0
		if(countRow <= height):
			grid.append(Node(x,y,new))
			count += 1
			countCol += 1
	expandMap()
	newMap = True

def expandMap():
	global expandedMap
	global expandedMap2
	global expandedWidth
	expandedMap = copy.deepcopy(grid)
	print "Expanded is %f" % len(grid)
	print "expanded width %f height %f" % (width,height)
	for i in range (0, height):
		for j in range (0, width):
			if (grid[j + (width * i)].intensity >= 100):
				for k in range (j - int(round(expandBuffer/res)), j + int(round(expandBuffer/res)) + 1):
					for l in range (i - int(round(expandBuffer/res)), i + int(round(expandBuffer/res))+1):
						if (k > 0 and k < width and l > 0 and l < height):
							#print "Trying index %f  k %f l %f"  % (k + (width * l),k,l)
							expandedMap[k + (width * l)].intensity = 100
	expandedMap2 = []
	added = False
	block = False
	for i in range(0,int(height/(expandedGridRes/res))):
		for j in range(0, int(width/(expandedGridRes/res))):
			block = False
			for y in range(0, int(expandedGridRes/res)):
				for x in range(0, int(expandedGridRes/res)):
					if(expandedMap[int(i*width*(expandedGridRes/res)+y*width+j*(expandedGridRes/res)+x)].intensity > expandThreshold):
						block =True
			if(block):
				expandedMap2.append(Node(j*expandedGridRes + gridOrigin.position.x,i*expandedGridRes + gridOrigin.position.y,100))
			else:
				if(expandedMap[int(i*width*(expandedGridRes/res)+y*width+j*(expandedGridRes/res)+x)].intensity >= 0):
					expandedMap2.append(Node(j*expandedGridRes + gridOrigin.position.x,i*expandedGridRes + gridOrigin.position.y,0))
				else:
					expandedMap2.append(Node(j*expandedGridRes + gridOrigin.position.x,i*expandedGridRes + gridOrigin.position.y,-1))
	newWidth = int(width/(expandedGridRes/res))
	newHeight = int(height/(expandedGridRes/res))
	expandedWidth = newWidth

	ocGrid = OccupancyGrid()
	pub_map = []
	meta = MapMetaData()
	meta.map_load_time = rospy.Time.now()
	meta.width = newWidth
	meta.height = newHeight
	meta.resolution = expandedGridRes

	for next in expandedMap2:
		pub_map.append(next.intensity)
	ocGrid.header.frame_id = '/map'
	ocGrid.header.stamp = rospy.Time.now()
	ocGrid.info = meta
	ocGrid.info.origin = gridOrigin
	ocGrid.data = pub_map
	expanded_pub.publish(ocGrid)


def odomCallBack(data):
	global theta
	global pose
	global twist
	pose = Pose()
	twist = Twist()
	twist = data.twist.twist
	xPos = data.pose.pose.position.x
	yPos = data.pose.pose.position.y
	orientation = data.pose.pose.orientation
	q = [orientation.x,orientation.y,orientation.z,orientation.w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
	theta = math.degrees(yaw)
	pose.position.x = xPos
	pose.position.y = yPos
	pose.orientation = orientation

def statusCallback(msg):
	global status
	status = GoalStatusArray()
	status = msg.status_list.goal_id
	print(status)

def spin360():
	startAngle = theta
	finalAngle = startAngle + angle
	while(finalAngle > 180):
		finalAngle = finalAngle - 180
	while(finalAngle < -180):
		finalAngle = finalAngle + 180
	publishTwist(0, AngularSpeed)

def publishTwist(linear,angular):
	twist = Twist()
	twist.linear.x = linear
	twist.angular.z = angular
	twistPub.publish(twist)

def findFrontiers():
	frontiers = []
	for next in expandedMap2:
		if(next.isFrontier):
			frontiers.append(next)
	return frontiers

def createBlobs(nodes):
	for next in nodes:
		frontierList = []




if __name__ == '__main__':
	rospy.init_node('Lab5')

	#subscribers
	rospy.Subscriber("/odom", Odometry, odomCallBack)
	rospy.Subscriber("/map", OccupancyGrid, mapCallBack, queue_size = 1)
	rospy.Subscriber('/move_base/status', GoalStatusArray, statusCallback, queue_size = 1)

	#publishers
	twistPub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
	goalPub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size = 1)
	expanded_pub = rospy.Publisher("lab5/map",OccupancyGrid, queue_size = 1)


	print("starting final project")