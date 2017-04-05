
import math, rospy
from geometry_msgs import Point
from geometry_msgs.msg import Pose


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


def startPoseCallback(msg):
	global startPose
	startPose = msg

def goalPoseCallback(msg):
	global goalPose
	goalPose = msg

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal):
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
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far


if __name__ == '__main__':
	rospy.init_node('Lab3_pathPlan')

	rospy.Subscriber("/move_base_simple/goal",PoseStamped, goalPoseCallback, queuesize = 1)
	rospy.Subscriber("/initialpose",PoseStamped, startPoseCallback, queuesize = 1)