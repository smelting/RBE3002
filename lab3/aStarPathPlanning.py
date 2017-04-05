
import math, rospy
from Queue import PriorityQueue
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from collections import defaultdict


class Node:

	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.parent = (0,0)
		self.cost = 0

	def inBounds(self, node):
		global width
		global height
		(x,y) = node
		return 0 <= x < width*res and 0 <= y < height*res

	def neighbors(self):
		neighbors = [(self.x+res, self.y), (self.x, self.y-res), (self.x-res, self.y), (self.x,self.y+res)]
		neighbors = filter(self.inBounds, neighbors)
		return neighbors


def startPoseCallback(msg):
	global startPose
	startPose = Pose()
	startPose = msg.pose.pose
	setStart = True
	print "starting at x %f y %f" %(startPose.position.x,startPose.position.y)

def goalPoseCallback(msg):
	global newGoal
	global goalPose
	gaolPose = Pose()
	goalPose = msg.pose
	newGoal = True
	print "ending at x %f y %f" %(goalPose.position.x,goalPose.position.y)

def heuristic(a, b):
	(x1,y1) = a
	(x2,y2) = b
	return abs(x1 - x2) + abs(y1 - y2)

def matchGridPose(p):
	global res
	(x,y) = p
	(x,y) = (round(x/res), round(y/res))
	return (x*res, y*res)

def matchPoseIndex(p):
	(x,y) = p
	ticks = x/res + (y/res*width)
	return int(ticks)


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
		x = (countCol + startX - 1)*res
		y = (countRow + startY - 1)*res
		if(countCol == width): #hit the end of the column
			countRow = countRow + 1
			countCol = 0
		if(countRow <= height and new != 100):
			if(count < 10):
				print "Making index %f x:%f y:%f with inten %f" %(count,x,y,new)
			grid.append(Node(x,y))
			count += 1
			countCol += 1


def genPath(start,goal):
	path = []
	current = goal
	while current != start:
		path.append(current)
		current = grid[matchPoseIndex(current)].parent
		print "next x %f y %f" % (current)
		rospy.sleep(.5)

def a_star_search(start, goal):

	global newGoal
	goal = matchGridPose(goal)
	start = matchGridPose(start)
	frontier = PriorityQueue()
	print "i: %f %f %f" % (matchPoseIndex(start),grid[matchPoseIndex(start)].x,grid[matchPoseIndex(start)].y)
	frontier.put(grid[matchPoseIndex(start)], 0)
	visited = []
	print "starting A* at x %f y %f" % (start)
	while not frontier.empty():
		current = frontier.get()
		print "current: x %f y %f" %(current.x,current.y)
		if(current.x == goal[0] and current.y == goal[1]):
			break

		for next in current.neighbors():
			newCost = current.cost + 1
			if grid[matchPoseIndex(next)] not in visited or newCost < grid[matchPoseIndex(next)].cost:
				grid[matchPoseIndex(next)].cost = newCost
				priority = newCost + heuristic(goal, start)
				frontier.put(grid[matchPoseIndex(next)],priority)
				grid[matchPoseIndex(next)].parent = (current.x,current.y)
				visited.append(grid[matchPoseIndex(next)])
				#print "LOLOL"
				#rospy.sleep(0.01)
	newGoal = False
	print "done with A*"
	return genPath(start,goal)


if __name__ == '__main__':
	rospy.init_node('Lab3_pathPlan')
	global newGoal
	global setStart
	setStart = False
	newGoal = False

	rospy.Subscriber("/move_base_simple/goal",PoseStamped, goalPoseCallback, queue_size = 1)
	rospy.Subscriber("/initialpose",PoseWithCovarianceStamped, startPoseCallback, queue_size = 1)
	rospy.Subscriber("/map", OccupancyGrid, mapCallBack, queue_size = 1)

	while(1):
		rospy.sleep(0.1)
		if(newGoal):
			a_star_search((startPose.position.x,startPose.position.y),(goalPose.position.x,goalPose.position.y))
			newGoal = False