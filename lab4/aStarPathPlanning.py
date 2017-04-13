
import math, rospy, copy, tf
from Queue import PriorityQueue
import heapq
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

expandThreshold = 60
expandBuffer = 0.125
expandedGridRes = 0.2

#class for each node on the map grid
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

#callback for RVIZ startpose position
def startPoseCallback(msg):
	global startPose
	startPose = Pose()
	startPose = msg.pose.pose
	setStart = True
	print "starting at x %f y %f" %(startPose.position.x,startPose.position.y)

#callback for RVIZ goalpose position
def goalPoseCallback(msg):
	global newGoal
	global goalPose
	gaolPose = Pose()
	goalPose = msg.pose
	newGoal = True
	print "ending at x %f y %f" %(goalPose.position.x,goalPose.position.y)

#determines the heuristic cost from point a to b using euclidian distance
def heuristic(a, b):
	(x1,y1) = a
	(x2,y2) = b
	return math.sqrt(math.pow(x1-x2,2) + math.pow(y1-y2,2))

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

#interpret a map and create the grid using the width/height and map info
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
				expandedMap2.append(Node(j*expandedGridRes + gridOrigin.position.x,i*expandedGridRes + gridOrigin.position.y,0))
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


#publish the path that A* creates (only important landmarks)
def publishPath(p):
	poseArray = []
	msg = Path()
	count = 0
	for thing in p:
		newPose = PoseStamped()
		newPose.pose.position.x = thing[0]
		newPose.pose.position.y = thing[1]
		newPose.header.seq = count
		newPose.header.frame_id = 'map'
		count += 1
		poseArray.append(newPose)
	msg.poses = poseArray
	msg.header.frame_id = '/map'
	path_pub.publish(msg)


#creates the most direct path using A* generated path
def genPath(start,goal):
	path = []
	current = goal
	prevSlope = 0
	prevNode = goal
	path.append(goal)
	print " start X %f Y: %f" % (start[0],start[1])
	print " goal X %f Y %f" % (goal[0],goal[1])
	while math.fabs(current[0] - start[0]) > 0.005 and math.fabs(current[1] - start[1]) > 0.005 and not rospy.is_shutdown():
		newSlope = calcSlope(current,prevNode)
		print "new slope %f Old %f " % (newSlope, prevSlope)
		print "current pos x %f y %f" % (expandedMap2[matchPoseIndex(current,expandedGridRes,expandedWidth)].x,expandedMap2[matchPoseIndex(current,expandedGridRes,expandedWidth)].y)
		if(newSlope != prevSlope):
			path.append(prevNode)
		prevNode = current
		prevSlope = newSlope
		current = expandedMap2[matchPoseIndex(current,expandedGridRes,expandedWidth)].parent
	return path

#calculates the slope between 2 points, returns inf when verticle
def calcSlope(p,b):
	(x1,y1) = p
	(x2,y2) = b
	if(x1 - x2 == 0 and (y1 - y2 != 0)):
		return float('inf')
	elif(y1 - y2 == 0):
		return 0
	return (y1-y2)/(x1-x2)

#uses A* with heapq to determine best path between two points on a map with costs
def a_star_search(start, goal):

	global newGoal
	front = {}
	goal = matchGridPose(goal,expandedGridRes)
	start = matchGridPose(start,expandedGridRes)
	frontier = []
	print "i: %f %f %f" % (matchPoseIndex(start,expandedGridRes,expandedWidth),expandedMap2[matchPoseIndex(start,expandedGridRes,expandedWidth)].x,expandedMap2[matchPoseIndex(start,expandedGridRes,expandedWidth)].y)
	print "START: X %f Y %f" % (start[0],start[1])
	print "GOAL: x %f  y%f" % (goal[0],goal[1])
	heapq.heappush(frontier,(0,expandedMap2[matchPoseIndex(start,expandedGridRes,expandedWidth)]))
	front[matchPoseIndex(start,expandedGridRes,expandedWidth)] = start 
	visited = {}
	
	print "starting A* at x %f y %f" % (start)
	while 1 and not rospy.is_shutdown():
		(p,current) = heapq.heappop(frontier)
		if matchPoseIndex((current.x,current.y),expandedGridRes,expandedWidth) in front:
			del front[matchPoseIndex((current.x,current.y),expandedGridRes,expandedWidth)]
		dumpShit(front,visited)
		print "current: x %f y %f" %(current.x,current.y)
		if(math.fabs(current.x - goal[0]) < 0.005 and math.fabs(current.y - goal[1]) < 0.005):
			break
		for next in current.neighbors():
			moveCost = math.sqrt(math.pow(current.x - next[0],2) + math.pow(current.y - next[1],2))
			newCost = current.cost + moveCost
			if ((not visited.has_key(matchPoseIndex(next,expandedGridRes,expandedWidth))) or newCost < expandedMap2[matchPoseIndex(next,expandedGridRes,expandedWidth)].cost) and not visited.has_key(matchPoseIndex(next,expandedGridRes,expandedWidth)):
				expandedMap2[matchPoseIndex(next,expandedGridRes,expandedWidth)].cost = newCost
				priority = newCost + heuristic(goal, next) + costMap[matchPoseIndex(next,res,width)]/80
				#print "cost %f Heuristic %f  %f  x: %f y: %f" % (newCost, heuristic(goal, next), priority, next[0], next[1])
				front[matchPoseIndex(next,expandedGridRes,expandedWidth)] = next 
				heapq.heappush(frontier,(priority,expandedMap2[matchPoseIndex(next,expandedGridRes,expandedWidth)]))
				expandedMap2[matchPoseIndex(next,expandedGridRes,expandedWidth)].parent = (current.x,current.y)
				visited[matchPoseIndex(next,expandedGridRes,expandedWidth)] = expandedMap2[matchPoseIndex(next,expandedGridRes,expandedWidth)]
				#rospy.sleep(0.001)
	dumpShit(front,visited)
	print "done with A*"
	publishPath(genPath(start,goal))


#publishes the frontier and visited nodes for RVIZ display while A* runs
def dumpShit(q,p):
	stuffz = []
	stuffzOld = []
	for key in q:
		(x,y) = q[key]
		newPoint = makeGridCell(x,y)
		stuffz.append(newPoint)
	for key,place in p.iteritems():
		newPoint = makeGridCell(place.x,place.y)
		stuffzOld.append(newPoint)
	publishGrid(stuffz,'frontier')
	publishGrid(stuffzOld, 'visited')

#publishes a grid with the given type (visited or frontier)
def publishGrid(cells, type):
	global frontier_pub
	gridMsg = GridCells()
	gridMsg.header.stamp = rospy.Time.now()
	gridMsg.header.frame_id = '/map'
	gridMsg.cell_width = expandedGridRes
	gridMsg.cell_height = expandedGridRes
	gridMsg.cells = cells
	if type == 'frontier':
		frontier_pub.publish(gridMsg)
	elif type == 'visited':
		visited_pub.publish(gridMsg)

#makes a point given an x and y 
def makeGridCell(x, y):
	point = Point()
	point.x = x
	point.y = y
	point.z = 0
	return point


def odomLit(data):
	global xPos
	global yPos
	global theta
	global pose
	pose = Pose()
	xPos = data.pose.pose.position.x
	yPos = data.pose.pose.position.y
	orientation = data.pose.pose.orientation
	q = [orientation.x,orientation.y,orientation.z,orientation.w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
	theta = math.degrees(yaw)
	pose.position.x = xPos
	pose.position.y = yPos
	pose.orientation = orientation

def globalCostmapUpdate(data):
	global costMap
	costMap = []
	mapWidth = data.info.width
	mapHeight = data.info.height
	costRes = data.info.resolution
	countCol = 0
	countRow = 1
	print "costMap height %f weight %f" % (data.info.height,data.info.width)
	for new in data.data:
		x = (countCol - 1)*costRes
		y = (countRow - 1)*costRes
		if(countCol == mapWidth): #hit the end of the column
			countRow = countRow + 1
			countCol = 0
		if(countRow <= mapHeight):
			costMap.append(new)
			countCol += 1


if __name__ == '__main__':
	rospy.init_node('Lab3_pathPlan')
	global newGoal
	global setStart
	global frontier_pub
	global expanded_pub
	global newMap
	setStart = False
	newGoal = False
	newMap = False

	rospy.Subscriber("/move_base_simple/goal",PoseStamped, goalPoseCallback, queue_size = 1)
	rospy.Subscriber("/initialpose",PoseWithCovarianceStamped, startPoseCallback, queue_size = 1)
	rospy.Subscriber("/map", OccupancyGrid, mapCallBack, queue_size = 1)
	rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapUpdate, queue_size = 1)
	rospy.Subscriber('/odom',Odometry, odomLit, queue_size = 1)

	expanded_pub = rospy.Publisher("lab4/map",OccupancyGrid, queue_size = 1)

	frontier_pub = rospy.Publisher('/lab3/frontier', GridCells, queue_size = 10)
	visited_pub = rospy.Publisher('/lab3/visited', GridCells, queue_size = 1)
	path_pub = rospy.Publisher('/lab3/path', Path, queue_size = 1)

	while(1):
		rospy.sleep(0.1)
		while(newMap and newGoal):
			a_star_search((xPos,yPos),(goalPose.position.x,goalPose.position.y))
			newMap = False
