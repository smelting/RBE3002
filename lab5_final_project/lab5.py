import math, tf, rospy, copy, Queue
import heapq, random
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

AngularSpeed = 1
expandThreshold = 45
expandBuffer = 0.175
expandedGridRes = 0.1
newGoalDisplacement = 0.05
GLOBALCOSTTHRES = 35

MAX_ANGULAR_V = 1
SLEW_STEP = 0.0125
SLEW_STEP_TIME = 0.075
SLEW_STEPS = 50


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

	def inBounds2(self, node):
		(x,y) = node
		x = x - gridOrigin.position.x
		y = y - gridOrigin.position.y
		return 0 <= x < (width*res) and 0 <= y < (height*res)

	def inIndex(self, node):
		return matchPoseIndex(node,expandedGridRes,expandedWidth) < len(expandedMap2)

	#finds and returns all neighbors (excluding diagonals)
	def neighbors(self):
		neighbors = [(self.x+expandedGridRes, self.y), (self.x, self.y-expandedGridRes), (self.x-expandedGridRes, self.y), (self.x,self.y+expandedGridRes)]
		neighbors = filter(self.inBounds, neighbors)
		#neighbors = filter(open, neighbors)
		return neighbors

	def neighbors3(self):
		neighbors = [(self.x+res, self.y), (self.x, self.y-res), (self.x-res, self.y), (self.x,self.y+res)]
		neighbors = filter(self.inBounds2, neighbors)
		#neighbors = filter(open, neighbors)
		return neighbors

	#finds neighbors including diagonals
	def neighbors2(self):
		neighbors = [(self.x+expandedGridRes, self.y), (self.x, self.y-expandedGridRes), (self.x-expandedGridRes, self.y), (self.x,self.y+expandedGridRes), (self.x-expandedGridRes, self.y-expandedGridRes), (self.x-expandedGridRes, self.y+expandedGridRes),(self.x+expandedGridRes, self.y-expandedGridRes), (self.x+expandedGridRes, self.y+expandedGridRes)]
		neighbors = filter(self.inBounds, neighbors)
		neighbors = filter(self.inIndex, neighbors)
		#neighbors = filter(open, neighbors)
		return neighbors

	def isFrontier(self):
		if(self.intensity == 0):
			for next in self.neighbors():
				if(matchPoseIndex(next,expandedGridRes,expandedWidth) < expandedWidth*expandedHeight):
					if(matchPoseIndex(next,expandedGridRes,expandedWidth) < len(expandedMap2)):
						if(expandedMap2[matchPoseIndex(next,expandedGridRes,expandedWidth)].intensity == -1):
							return True
		return False

class FrontierNode:
	def __init__():
		pass
	def function():
		pass

class Frontier:
	def __init__():
		pass
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
	print "finding frontiers"
	findFrontiers()
	if(status_num == 1 and len(frontiers) > 0):
		pass
		#navToPos(nextGoal())
	newMap = True


def mapCallBack_update(msg):
	print "getting map update"
	global grid

	mapUpdateData = []
	grid = list(grid)
	mapUpdateWidth = msg.width
	mapUpdateHeight = msg.height
	mapUpdateData = msg.data
	startX = msg.x
	startY = msg.y
	index = 0
	for y in range(startY, startY + mapUpdateHeight):
		for x in range(startX, startX + mapUpdateWidth):
			#print "%f %f %f %f" % (len(costMap),costMapWidth*y + x,len(costMapUpdateData),index)
			if(index < len(mapUpdateData)):
				grid[width*y + x].intensity = mapUpdateData[index]
				index = index + 1
			else:
				#print" %f  %f" %(len(costMapUpdateData),index)
				break
	expandMap()
	findFrontiers()
	newMap = True

def expandMap():
	global expandedMap
	global expandedMap2
	global expandedWidth
	global expandedHeight
	print "expanding Map"
	expandedMap = copy.deepcopy(grid)
	for i in range (0, height):
		for j in range (0, width):
			if (grid[j + (width * i)].intensity >= expandThreshold):
				for k in range (j - int(round(expandBuffer/res)), j + int(round(expandBuffer/res)) + 1):
					for l in range (i - int(round(expandBuffer/res)), i + int(round(expandBuffer/res))+1):
						if (k > 0 and k < width and l > 0 and l < height):
							#print "Trying index %f  k %f l %f"  % (k + (width * l),k,l)
							expandedMap[k + (width * l)].intensity = 100
	expandedMap2 = []
	added = False
	block = False
	unknown = False
	for i in range(0,int(height/(expandedGridRes/res))):
		for j in range(0, int(width/(expandedGridRes/res))):
			block = False
			unknown = False
			for y in range(0, int(expandedGridRes/res)):
				for x in range(0, int(expandedGridRes/res)):
					if(expandedMap[int(i*width*(expandedGridRes/res)+y*width+j*(expandedGridRes/res)+x)].intensity == 100):
						block =True
					if(expandedMap[int(i*width*(expandedGridRes/res)+y*width+j*(expandedGridRes/res)+x)].intensity == -1):
						unknown = True
			if(block):
				expandedMap2.append(Node(j*expandedGridRes + gridOrigin.position.x,i*expandedGridRes + gridOrigin.position.y,100))
			elif(unknown):
				expandedMap2.append(Node(j*expandedGridRes + gridOrigin.position.x,i*expandedGridRes + gridOrigin.position.y,-1))
			else:
				expandedMap2.append(Node(j*expandedGridRes + gridOrigin.position.x,i*expandedGridRes + gridOrigin.position.y,0))
	newWidth = int(width/(expandedGridRes/res))
	newHeight = int(height/(expandedGridRes/res))
	expandedWidth = newWidth
	expandedHeight = newHeight

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
	print "done expanding"

#moves a point to the nearest grid position
def matchGridPose(p,resD):
	(x,y) = p
	(x,y) = (round(x/resD), round(y/resD))
	return (x*resD, y*resD)

#determines where a point is in the grid array
def matchPoseIndex(p,resD,widthD):
	(x,y) = p
	x = x - gridOrigin.position.x
	y = y - gridOrigin.position.y
	ticks = x/resD + (y/resD*widthD)
	return int(ticks)

#determines if the point is open (not blocked)
def open(p):
	return expandedMap2[matchPoseIndex(p,expandedGridRes,expandedWidth)].intensity != 100

def odomCallBack(data):
	global theta
	global pose
	global twist
	global xPos
	global yPos
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
	global status_num
	global status_text
	status_num = 0
	status_text = "PENDING"
	status = GoalStatusArray()
	status = msg
	if(len(status.status_list) > 0):
		status_text = status.status_list[0].text
		status_num = status.status_list[0].status

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
	global frontiers
	global blobsSeen
	global blobLimit
	blobLimit = 4
	blobsSeen = 0
	frontiers = []
	frontiers_points = []
	for next in expandedMap2:
		if(next.isFrontier()):
			if(next not in frontiers):
				frontiers.append(next)
				frontiers_points.append(makeGridCell(next.x+expandedGridRes/2,next.y+expandedGridRes/2))
	frontierGrid = GridCells()
	frontierGrid.header.frame_id = '/map'
	frontierGrid.header.stamp = rospy.Time.now()
	frontierGrid.cell_width = expandedGridRes
	frontierGrid.cell_height = expandedGridRes
	frontierGrid.cells = frontiers_points
	frontier_pub.publish(frontierGrid)
	print("Frontiers found %f") %len(frontiers)
	return frontiers

def createBlobs(nodes):
	for next in nodes:
		frontierList = []

def makeGridCell(x, y):
	point = Point()
	point.x = x
	point.y = y
	point.z = 0
	return point

def nextGoal():
	done = False
	global blobLimit
	count = 0
	while(not done):
		count = count + 1
		bestGoal = ()
		bestDist = 0
		newX = 0
		newY = 0
		for next in frontiers:
			dist = math.hypot(pose.position.x - next.x, pose.position.y - next.y)
			if(dist > bestDist):
				bestDist = dist
				bestGoal = (next.x, next.y)
		(bestGoal,done) = blobFrontiers(bestGoal)
		print " LOOL"
		if(blobLimit > 2):
			blobLimit = blobLimit - 1
		if(len(frontiers) < 4 and done == False or count > 100):
			print "can't find a blob to go to!"
			bestGoal = (xPos - .25,yPos -.25)
			done = True
	# while(costMap[matchPoseIndex(matchGridPose(stepFromGoal(bestGoal),costMapRes),costMapRes,costMapWidth)] > GLOBALCOSTTHRES or costMap[matchPoseIndex(matchGridPose(stepFromGoal(bestGoal),costMapRes),costMapRes,costMapWidth)] == -1):
	# 	bestGoal = stepFromGoal(bestGoal)
	# 	if(math.fabs(xPos - bestGoal[0]) < .5 and math.fabs(yPos - bestGoal[1]) < .5): #suck at it's own position
	# 		break
	# 	print "cost of X: %f Y: %f is %f" % (bestGoal[0],bestGoal[1],costMap[matchPoseIndex(matchGridPose(stepFromGoal(bestGoal),costMapRes),costMapRes,costMapWidth)])

	print "Going to X: %f Y: %f" %(bestGoal[0],bestGoal[1])

	return bestGoal

def stepFromGoal(goal):
	(newX,newY) = goal
	slope = math.atan2((yPos - newY),(xPos - newX))

	if(math.hypot(xPos - (newX + newGoalDisplacement*math.cos(slope)),yPos - (newY + newGoalDisplacement*math.sin(slope))) > math.hypot(xPos - newX,yPos - newY)):
		newX = newX - newGoalDisplacement*math.cos(slope)
		newY = newY - newGoalDisplacement*math.sin(slope)
	else:
		newX = newX + newGoalDisplacement*math.cos(slope)
		newY = newY + newGoalDisplacement*math.sin(slope)
	bestGoal = (newX,newY)
	return bestGoal

def globalCostMapCallback(msg):
	global costMap
	global costMapWidth
	global costMapHeight
	global costMapRes
	global costMapOrigin

	costMap = []
	costMap = msg.data
	costMapWidth = msg.info.width
	costMapHeight = msg.info.height
	costMapRes = msg.info.resolution
	# print "costMap costMapRes is %f m" % costMapRes
	# print "costMapWidth %f costMapHeight %f" % (costMapWidth,costMapHeight)
	countCol = 0
	countRow = 0
	startX = msg.info.origin.position.x
	startY = msg.info.origin.position.y
	costMapOrigin = msg.info.origin

def costMapUpdateCallback(msg):
	global costMap
	global costMapWidthUpdate
	global costMapHeightUpdate
	global costMapOrigin
	global newCostMap
	costMapPub = OccupancyGrid()
	costMapUpdateData = []
	costMap = list(costMap)
	costMapWidthUpdate = msg.width
	costMapHeightUpdate = msg.height
	costMapUpdateData = msg.data
	startX = msg.x
	startY = msg.y
	index = 0
	# print "costMap update x: %f y: %f" % (startX,startY)
	# print "costMap origin is at x: %f y:: %f" %(costMapOrigin.position.x,costMapOrigin.position.y)
	# print "costMapWidth %f costMapHeight %f" % (costMapWidth,costMapHeight)

	for y in range(startY, startY + costMapHeightUpdate):
		for x in range(startX, startX + costMapWidthUpdate):
			#print "%f %f %f %f" % (len(costMap),costMapWidth*y + x,len(costMapUpdateData),index)
			if(index < len(costMapUpdateData)):
				costMap[costMapWidth*y + x] = costMapUpdateData[index]
				index = index + 1
			else:
				#print" %f  %f" %(len(costMapUpdateData),index)
				break
	costMapPub.header.frame_id = '/map'
	costMapPub.header.stamp = rospy.Time.now()
	costMapPub.info.resolution = costMapRes
	costMapPub.info.origin = costMapOrigin
	costMapPub.info.width = costMapWidth
	costMapPub.info.height = costMapHeight
	costMapPub.data = costMap
	costMap_pub.publish(costMapPub)
	newCostMap = True

def stop():
	publishTwist(0,0)

def spinWheels(u1, u2, time):
	timeStamp = rospy.get_time()
	if(u1 != u2):
		radius = (u1+u2)/(2*(u2-u1))
		angular = (u2 - u1)/9
		linear = angular*radius
	else:
		angular  = 0
		linear = u1

	start_angular = twist.angular.z
	start_linear = twist.linear.x
	if(angular > MAX_ANGULAR_V):
		angular = MAX_ANGULAR_V
	if(angular < -MAX_ANGULAR_V):
		angular = -MAX_ANGULAR_V
	while(rospy.get_time() - timeStamp < time):
		if(start_linear < linear):
			start_linear += SLEW_STEP
		if(start_angular < angular):
			start_angular += SLEW_STEP
		#print "linear %f angular %f" % (start_linear,start_angular)
		publishTwist(linear,angular)
		rospy.sleep(SLEW_STEP_TIME)

def navToPos(p):
	(x,y) = p
	goal = PoseStamped()
	goal2 = PoseStamped()
	goal.header.frame_id = '/map'
	goal.header.stamp = rospy.Time.now()
	goal2.header.frame_id = '/map'
	goal2.header.stamp = rospy.Time.now()
	goal.pose.position.x = x
	goal.pose.position.y = y
	goal2.pose.position.x = startBest[0]
	goal2.pose.position.y = startBest[1]
	# theta = random.randint(0,360)
	# quaternion = (
 #    pose.orientation.x,
 #    pose.orientation.y,
 #    pose.orientation.z,
 #    pose.orientation.w)
	# euler = tf.transformations.euler_from_quaternion(quaternion)
	# roll = euler[0]
	# pitch = euler[1]
	# yaw = euler[2]
	# yaw = yaw + theta
	# if(yaw > 350):
	# 	yaw = yaw - 350
	# orien =  tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	goal.pose.orientation = pose.orientation
	goal2.pose.orientation = pose.orientation
	print("publishing goal to navstack")
	goalPub.publish(goal)
	goal_Nav_Pub.publish(goal)
	goal_Nav_Pub1.publish(goal2)

def blobFrontiers(goal):
	global blobsSeen
	global startBest
	startBest = (0,0)
	blob = []
	if(len(goal) == 2 and matchPoseIndex(goal,expandedGridRes,expandedWidth) < len(expandedMap2)):
		blob.append(expandedMap2[matchPoseIndex(goal,expandedGridRes,expandedWidth)])
		for next in blob:
			for next2 in next.neighbors2():
				if(expandedMap2[matchPoseIndex(next2,expandedGridRes,expandedWidth)] in frontiers):
					blob.append(expandedMap2[matchPoseIndex(next2,expandedGridRes,expandedWidth)])
					frontiers.remove(expandedMap2[matchPoseIndex(next2,expandedGridRes,expandedWidth)])
		blobsSeen = blobsSeen + 1
		if(len(blob) > blobLimit):	
			blobGoal = [0,0]
			for next in blob:
				blobGoal[0] = blobGoal[0] + next.x
				blobGoal[1] = blobGoal[1] + next.y
			blobGoal[0] = (blobGoal[0]/len(blob))
			blobGoal[1] = (blobGoal[1]/len(blob))

			blobGoal = matchGridPose(blobGoal,costMapRes)
			startBest = blobGoal
			# checking = []
			# found = False
			# heapq.heappush(checking,(0,blobGoal))
			# count = 0
			# while(not found and count < 1000):
			# 	count = count + 1
			# 	(p,node) = heapq.heappop(checking)
			# 	if(costMap[matchPoseIndex(node,costMapRes,costMapWidth)] > 10 or costMap[matchPoseIndex(node,costMapRes,costMapWidth)] < 0 or expandedMap2[matchPoseIndex(node,expandedGridRes,expandedWidth)].intensity < 0):
			# 		if(matchPoseIndex(node,expandedGridRes,expandedWidth) < len(expandedMap2)):
			# 			for newNode in expandedMap2[matchPoseIndex(node,expandedGridRes,expandedWidth)].neighbors():
			# 				dist = math.hypot(startBest[0] - newNode[0], startBest[1] - newNode[1])
			# 				heapq.heappush(checking,(dist,newNode))
			# 	else:
			# 		found = True
			# 		blobGoal = node
			# 		blobGoal = list(blobGoal)
			print "done with blobThing"
			return (blobGoal,True)
	return ([],False)

if __name__ == '__main__':
	rospy.init_node('Lab5')
	global twistPub
	global goalPub
	global expanded_pub
	global frontier_pub
	global newMap
	global goal_Nav_Pub
	global goal_Nav_Pub1
	global costMap_pub
	global newCostMap
	newCostMap = False
	newMap = False
	#subscribers




	rospy.Subscriber("/odom", Odometry, odomCallBack)
	rospy.Subscriber('/move_base/status', GoalStatusArray, statusCallback, queue_size = 1)
	rospy.Subscriber("/map", OccupancyGrid, mapCallBack, queue_size = 1)
	rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, globalCostMapCallback, queue_size = 1)
	rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, costMapUpdateCallback, queue_size = 1)
	rospy.Subscriber('/map_updates', OccupancyGridUpdate, mapCallBack_update, queue_size = 1)
			#publishers
	twistPub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
	goalPub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size = 1)
	expanded_pub = rospy.Publisher("lab5/map",OccupancyGrid, queue_size = 1)
	costMap_pub = rospy.Publisher('/lab5/costMap',OccupancyGrid, queue_size = 1)
	frontier_pub = rospy.Publisher('/lab5/frontier', GridCells, queue_size = 1)
	goal_Nav_Pub = rospy.Publisher('/lab5/navGoal',PoseStamped,queue_size = 1)
	goal_Nav_Pub1 = rospy.Publisher('/lab5/navGoal2',PoseStamped,queue_size = 1)


	print("starting final project")
	startTime = rospy.get_time()
	startTheta = theta
	timeSince = 0
	while(timeSince < 3):
		publishTwist(0,.79)
		timeSince = rospy.get_time() - startTime
	# while(math.fabs(theta - startTheta)>3):
	# 	publishTwist(0,.5)
	stop()
	while(newMap == False):
		pass
	while(1):
		rospy.sleep(1)
		print "current Status %f " % status_num
		if(status_num == 3 or status_num == 0 or status_num == 4):
			if(len(frontiers) < 4 and newMap == True):
			 	break;
			while(newCostMap == False):
				pass
			if(newMap):
				print "calculating new Goal"
				navToPos(nextGoal())
				newCostMap = False
				newMap = False
				rospy.sleep(5) 
		elif(status_num == 2):
			findFrontiers()
			rospy.sleep(2)
			print "shit in a bucket"
	print "Done with this dank shit"
