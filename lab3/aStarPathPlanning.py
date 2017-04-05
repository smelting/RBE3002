
import math, rospy
from geometry_msgs import Point
from geometry_msgs.msg import Pose


class Node:

	def __init__(self, x, y):
	self.x = x
	self.y = y
	self.gCost = 0
	self.Point()
	
	def calcGCost(self):
		global startPose
		self.gCost = fabs(self.x-startPose.position.x) + fabs(self.y-startPose.position.y)


def startPoseCallback(msg):
	global startPose
	startPose = msg

def goalPoseCallback(msg):
	global goalPose
	goalPose = msg

def runAStar(start, goal):
	do the do



if __name__ == '__main__':

	rospy.Subscriber("/move_base_simple/goal",PoseStamped, goalPoseCallback, queuesize = 1)
	rospy.Subscriber("/initialpose",PoseStamped, startPoseCallback, queuesize = 1)