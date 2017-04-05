
import math, rospy
from geometry_msgs import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid


class Node:

	def __init__(self, x, y):
	self.x = x
	self.y = y

	def inBounds(self, pos):
		global width
		global height
		(x,y) = pos
		return 0 <= x < width and 0 <= y < height

	def neighbors(self, pos):
		(x,y) = pos
		neighbors = [(x+1, y), (x, y-1), (x-1, y), (x,y+1)]
		neighbors = filter(self.in_bounds, neighbors)
		return neighbors


def startPoseCallback(msg):
	global startPose
	startPose = Pose()
	startPose = msg.pose.pose
	setStart = True

def goalPoseCallback(msg):
	global goalPose
	gaolPose = Pose()
	goalPose = msg.pose
	newGoal = True

def heuristic(a, b):
	(x1, y1) = a
	(x2, y2) = b
	return abs(x1 - x2) + abs(y1 - y2)

def matchGridPose(p):
	global res
	(x,y) = p
	(x,y) = (round(x/res), round(y/res))
	return (x*res, y*res)


def mapCallBack(msg):
	global grid
	global width
	global height
	global res
	grid = []
	width = msg.info.width
	height = msg.info.height
	res = msg.info.resolution
	countCol = 1
	countRow = 1
	startX = msg.info.pose.x
	startY = msg.info.pose.y
	for new in msg.data:
		if(countCol = width): #hit the end of the column
			countRow = countRow + 1
			countCol = 0
		if(countRow <= height):
			grid.append(Node(countCol + startX - 1, countRow + startY - 1))


def a_star_search(start, goal):
	global newGoal
	frontier = PriorityQueue()
	frontier.put(start, 0)
	came_from = {}
	cost_so_far = {}
	came_from[start] = None
	cost_so_far[start] = 0
	
	while not frontier.empty():
		current = frontier.get()
		
		if current == goal:
			break
		
		for next in grid.neighbors(current):
			new_cost = cost_so_far[current] + graph.cost(current, next)
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost
				priority = new_cost + heuristic(goal, next)
				frontier.put(next, priority)
				came_from[next] = current
	newGoal = False
	return came_from, cost_so_far


if __name__ == '__main__':
	rospy.init_node('Lab3_pathPlan')
	global newGoal
	global setStart
	setStart = False
	newGoal = False

	rospy.Subscriber("/move_base_simple/goal",PoseStamped, goalPoseCallback, queuesize = 1)
	rospy.Subscriber("/initialpose",PoseWithCovarianceStamped, startPoseCallback, queuesize = 1)
	rospy.Subscriber("/map", OccupancyGrid, mapCallBack, queuesize = 1)

	while(1):
		rospy.sleep(0.1)
		if(newGoal):
			a_star_search(startPose,goalPose)