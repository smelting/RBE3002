
import math, rospy
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
def matchGridPose(p):
	global res
	(x,y) = p
	(x,y) = (round(x/res), round(y/res))
	return (x*res, y*res)

#determines where a point is in the grid array
def matchPoseIndex(p):
	(x,y) = p
	ticks = x/res + (y/res*width)
	return int(ticks)

#determines if the point is open (not blocked)
def open(p):
	return grid[matchPoseIndex(p)].intensity != 100

#interpret a map and create the grid using the width/height and map info
def mapCallBack(msg):
	print "grabbing map"
	global grid
	global width
	global height
	global res
	global gridOrigin
	grid = []
	width = msg.info.width
	height = msg.info.height
	res = msg.info.resolution
	print "grid res is %f m" % res
	print "width %f height %f" % (width,height)
	countCol = 1
	countRow = 1
	startX = msg.info.origin.position.x
	startY = msg.info.origin.position.y
	gridOrigin = msg.info.origin
	print "0,0   %f  %f " % (startX,startY)
	count = 0
	for new in msg.data:
		x = (countCol - 1)*res
		y = (countRow - 1)*res
		if(countCol == width): #hit the end of the column
			countRow = countRow + 1
			countCol = 0
		if(countRow <= height):
			grid.append(Node(x,y,new))
			count += 1
			countCol += 1

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
	while current != start:
		newSlope = calcSlope(current,prevNode)
		print "new slope %f Old %f " % (newSlope, prevSlope)
		if(newSlope != prevSlope):
			path.append(prevNode)
		prevNode = current
		prevSlope = newSlope
		current = grid[matchPoseIndex(current)].parent
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
	goal = matchGridPose(goal)
	start = matchGridPose(start)
	frontier = []
	print "i: %f %f %f" % (matchPoseIndex(start),grid[matchPoseIndex(start)].x,grid[matchPoseIndex(start)].y)
	heapq.heappush(frontier,(0,grid[matchPoseIndex(start)]))
	front[matchPoseIndex(start)] = start 
	visited = []
	
	print "starting A* at x %f y %f" % (start)
	while 1:
		(p,current) = heapq.heappop(frontier)
		if matchPoseIndex((current.x,current.y)) in front:
			del front[matchPoseIndex((current.x,current.y))]
		dumpShit(front,visited)
		#print "current: x %f y %f" %(current.x,current.y)
		if(current.x == goal[0] and current.y == goal[1]):
			break
		for next in current.neighbors():
			newCost = current.cost + res
			if grid[matchPoseIndex(next)] not in visited or newCost < grid[matchPoseIndex(next)].cost:
				grid[matchPoseIndex(next)].cost = newCost
				priority = newCost + heuristic(goal, next)*1.5
				#print "cost %f Heuristic %f  %f  x: %f y: %f" % (newCost, heuristic(goal, next), priority, next[0], next[1])
				front[matchPoseIndex(next)] = next 
				heapq.heappush(frontier,(priority,grid[matchPoseIndex(next)]))
				grid[matchPoseIndex(next)].parent = (current.x,current.y)
				visited.append(grid[matchPoseIndex(next)])
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


if __name__ == '__main__':
	rospy.init_node('Lab3_pathPlan')
	global newGoal
	global setStart
	global frontier_pub
	setStart = False
	newGoal = False

	rospy.Subscriber("/move_base_simple/goal",PoseStamped, goalPoseCallback, queue_size = 1)
	rospy.Subscriber("/initialpose",PoseWithCovarianceStamped, startPoseCallback, queue_size = 1)
	rospy.Subscriber("/map", OccupancyGrid, mapCallBack, queue_size = 1)

	frontier_pub = rospy.Publisher('/lab3/frontier', GridCells, queue_size = 10)
	visited_pub = rospy.Publisher('/lab3/visited', GridCells, queue_size = 1)
	path_pub = rospy.Publisher('/lab3/path', Path, queue_size = 1)

	while(1):
		rospy.sleep(0.1)
		if(newGoal):
			a_star_search((startPose.position.x,startPose.position.y),(goalPose.position.x,goalPose.position.y))
			
			newGoal = False
