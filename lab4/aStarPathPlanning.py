
import math, rospy, copy
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
from geometry_msgs.msg import Point

expandThreshold = 60
expandBuffer = 0.254
expandedGridRes = 0.1

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
		global width
		global height
		(x,y) = node
		return 0 <= x < width*res and 0 <= y < height*res

	#finds and returns all neighbors (including diagonals)
	def neighbors(self):
		neighbors = [(self.x+res, self.y), (self.x, self.y-res), (self.x-res, self.y), (self.x,self.y+res),(self.x+res,self.y+res),(self.x-res,self.y+res),(self.x-res,self.y-res),(self.x+res,self.y-res)]
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
	ticks = x/resD + (y/resD*widthD)
	return int(ticks)

#determines if the point is open (not blocked)
def open(p):
	return grid[matchPoseIndex(p,res,width)].intensity != 100

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
		x = (countCol)*res
		y = (countRow)*res
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
	global expandedWidth
	expandedMap = []
	added = False
	block = False
	for i in range(0,int(height/(expandedGridRes/res))):
		for j in range(0, int(width/(expandedGridRes/res))):
			block = False
			for y in range(0, int(expandedGridRes/res)):
				for x in range(0, int(expandedGridRes/res)):
					if(grid[int(i*width*(expandedGridRes/res)+y*width+j*(expandedGridRes/res)+x)].intensity > expandThreshold):
						block =True
			if(block):
				expandedMap.append(Node(j*expandedGridRes,i*expandedGridRes,100))
			else:
				expandedMap.append(Node(j*expandedGridRes,i*expandedGridRes,0))
	newWidth = int(width/(expandedGridRes/res))
	newHeight = int(height/(expandedGridRes/res))
	expandedWidth = newWidth
	expandedMap2 = copy.deepcopy(expandedMap)
	print "Expanded is %f" % len(expandedMap)
	print "expanded width %f height %f" % (newWidth,newHeight)
	for i in range (0, newHeight):
		for j in range (0, newWidth):
			if (grid[j + (newWidth * i)].intensity >= 100):
				for k in range (j - int(round(expandBuffer/expandedGridRes)), j + int(round(expandBuffer/expandedGridRes)) + 1):
					for l in range (i - int(round(expandBuffer/expandedGridRes)), i + int(round(expandBuffer/expandedGridRes))+1):
						if (k > 0 and k < newWidth and l > 0 and l < newHeight):
							#print "Trying index %f  k %f l %f"  % (k + (width * l),k,l)
							expandedMap2[k + (newWidth * l)].intensity = 100
	ocGrid = OccupancyGrid()
	pub_map = []
	meta = MapMetaData()
	meta.map_load_time = rospy.Time.now()
	meta.width = newWidth
	meta.height = newHeight
	meta.resolution = expandedGridRes

	for next in expandedMap:
		pub_map.append(next.intensity)
	ocGrid.header.frame_id = '/map'
	ocGrid.header.stamp = rospy.Time.now()
	ocGrid.info = meta
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
	while current != start and not rospy.is_shutdown():
		newSlope = calcSlope(current,prevNode)
		print "new slope %f Old %f " % (newSlope, prevSlope)
		if(newSlope != prevSlope):
			path.append(prevNode)
		prevNode = current
		prevSlope = newSlope
		current = grid[matchPoseIndex(current,expandedGridRes,expandedWidth)].parent
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
	print "i: %f %f %f" % (matchPoseIndex(start,expandedGridRes,expandedWidth),expandedMap[matchPoseIndex(start,expandedGridRes,expandedWidth)].x,expandedMap[matchPoseIndex(start,expandedGridRes,expandedWidth)].y)
	heapq.heappush(frontier,(0,expandedMap[matchPoseIndex(start,expandedGridRes,expandedWidth)]))
	front[matchPoseIndex(start,expandedGridRes,expandedWidth)] = start 
	visited = []
	
	print "starting A* at x %f y %f" % (start)
	while 1 and not rospy.is_shutdown():
		(p,current) = heapq.heappop(frontier)
		if matchPoseIndex((current.x,current.y),expandedGridRes,expandedWidth) in front:
			del front[matchPoseIndex((current.x,current.y),expandedGridRes,expandedWidth)]
		dumpShit(front,visited)
		#print "current: x %f y %f" %(current.x,current.y)
		if(current.x == goal[0] and current.y == goal[1]):
			break
		for next in current.neighbors():
			moveCost = math.sqrt(math.pow(current.x - next[0],2) + math.pow(current.y - next[1],2))
			newCost = current.cost + moveCost
			if (expandedMap[matchPoseIndex(next,expandedGridRes,expandedWidth)] not in visited or newCost < expandedMap[matchPoseIndex(next,expandedGridRes,expandedWidth)].cost) and expandedMap[matchPoseIndex(next,expandedGridRes,expandedWidth)] not in visited:
				expandedMap[matchPoseIndex(next,expandedGridRes,expandedWidth)].cost = newCost
				priority = newCost + heuristic(goal, next) + costMap[matchPoseIndex(next,expandedGridRes,expandedWidth)]
				#print "cost %f Heuristic %f  %f  x: %f y: %f" % (newCost, heuristic(goal, next), priority, next[0], next[1])
				front[matchPoseIndex(next,expandedGridRes,expandedWidth)] = next 
				heapq.heappush(frontier,(priority,expandedMap[matchPoseIndex(next,expandedGridRes,expandedWidth)]))
				expandedMap[matchPoseIndex(next,expandedGridRes,expandedWidth)].parent = (current.x,current.y)
				visited.append(expandedMap[matchPoseIndex(next,expandedGridRes,expandedWidth)])
				#rospy.sleep(0.001)
	newGoal = False
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
	for place in p:
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
	gridMsg.cell_width = res
	gridMsg.cell_height = res
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

	expanded_pub = rospy.Publisher("lab4/map",OccupancyGrid, queue_size = 1)

	frontier_pub = rospy.Publisher('/lab3/frontier', GridCells, queue_size = 10)
	visited_pub = rospy.Publisher('/lab3/visited', GridCells, queue_size = 1)
	path_pub = rospy.Publisher('/lab3/path', Path, queue_size = 1)

	while(1):
		rospy.sleep(0.1)
		while(newMap and newGoal):
			a_star_search((startPose.position.x,startPose.position.y),(goalPose.position.x,goalPose.position.y))
			newMap = False
