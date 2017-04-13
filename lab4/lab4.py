#!/usr/bin/env python

import rospy, tf
import math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Path
from nav_msgs.msg import GridCells
from nav_msgs.msg import Odometry


MAX_ANGULAR_V = 1
SLEW_STEP = 0.0125
SLEW_STEP_TIME = 0.075
SLEW_STEPS = 50
# Add additional imports for each of the message types used


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	global busy
	busy = True
	goalPose = Pose()

	goalPose.position.x = goal.pose.position.x
	goalPose.position.y = goal.pose.position.y
	orientation = goal.pose.orientation
	q = [orientation.x,orientation.y,orientation.z,orientation.w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
	thetaGoal = math.degrees(yaw)

	x = goalPose.position.x - pose.position.x
	y = goalPose.position.y - pose.position.y
	print "Goal x: %f y: %f theta: %f" % (x,y,thetaGoal)

	angle = math.atan2(y,x)
	angle = math.degrees(angle)
	print "rotating to goal %f" % angle
	rotate(angle)
	distanceToGo = calcDistance(goalPose,pose)
	print "driving to goal %f" % distanceToGo
	driveStraight(.25,distanceToGo)
	print "rotating to final pose %f" % thetaGoal
	#rotate(thetaGoal)
	print "Went to x: %f y: %f theta: %f" % (xPos,yPos,theta)
	if navReady:
		stop()
	print "Done with navigation"
	busy = False
	


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	print "Don't bump me asshole"
	driveStraight(.26,.6)
	rotate(90)
	driveStraight(0.25,.45)
	rotate(135)




#This function accepts two wheel velocities and a time interval.
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

def stop():
	publishTwist(0,0)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	start_pose = pose
	#print "Start x: %f y: %f" % (start_pose.position.x,start_pose.position.y)
	while(calcDistance(pose,start_pose) < distance and navReady):
		#print "X: %f Y: %f" % (pose.position.x,pose.position.y)
		#print "%f" % calcDistance(pose,start_pose)
		#print "Pose: x: %f y: %f Dist: %f" % (pose.position.x,pose.position.y,calcDistance(pose,start_pose))
		spinWheels(speed,speed,.01)
		#print "Done with Straight path"
	stop()


def calcDistance(pose1,pose2):
	distance = math.sqrt(math.pow(pose1.position.x-pose2.position.x,2) + math.pow(pose1.position.y-pose2.position.y,2))
	return distance


def publishTwist(linear,angular):
	twist = Twist()
	twist.linear.x = linear
	twist.angular.z = angular
	pub.publish(twist)
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	start_angle = theta
	new_angle = angle - start_angle
	while(new_angle > 180):
		new_angle = new_angle - 180
	while(new_angle < -180):
		new_angle = new_angle + 180


	#get within 1 degree of the desired angle
	while(math.fabs(theta - angle) >= 1):
		error = theta - angle
		spinWheels(error*0.6,-error*0.6,.01)
		#print "Desired %f cur %f pwr: %f" % (angle, theta,error*0.4)
	stop()



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	angular = speed/radius
	linear = speed
	start_angle = theta

	start_linear = 0
	start_angular = 0

	step_linear = linear/SLEW_STEPS
	step_angular = angular/SLEW_STEPS
	while(math.fabs(theta - start_angle) < angle):
		if(start_linear < linear):
			start_linear += step_linear
		if(start_angular < angular):
			start_angular += step_angular
		publishTwist(start_linear,start_angular)
		rospy.sleep(SLEW_STEP_TIME) 
	stop()

def occupancyCallBack(grid):
	meta = MapMetaData()
	data = grid.data
	meta = grid.info


def metaCallBack(data):
	resolution = data.resolution
	width = data.width
	height = data.height
	metaPose = Pose()
	metaPose = origin


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
    	executeTrajectory()

def getOdom():
	odomSub = rospy.Subscriber("/odom", Odometry, odomCallBack)

def odomCallBack(data):
	global xPos
	global yPos
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

	goal = PoseStamped()
	goal.header.frame_id = "/base_link"
	goal.header.stamp = rospy.Time.now()
	goal.pose.position.z = 0
	goal.pose.position.x = xPos
	goal.pose.position.y = yPos
	goal.pose.orientation = pose.orientation
	pos_pub.publish(goal)
	
def pathCallback(msg):
	global hasPath
	global navReady
	global path
	path = msg.poses
	navReady = False
	hasPath = True

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('Lab2_nav')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global odom_tf
    global odom_list
    global pose
    global path
    global busy
    global navReady
    global hasPath
    hasPath = False
    path = Path()
    navReady = True
    busy = False
    pose = Pose()
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1) # Publisher for commanding robot motion
    pos_pub = rospy.Publisher("/lab2/pose",PoseStamped, queue_size = 1)
    #bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    #nav_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped, navToPose)

    #metaMap_sub = rospy.Subscriber('/map_metadata',MapMetaData,metaCallBack,queue_size = 1)
    occupancy_sub = rospy.Subscriber('/map',OccupancyGrid ,occupancyCallBack, queue_size = 1)
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    path_sub = rospy.Subscriber('lab3/path', Path, pathCallback)
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"
    getOdom()
    print "yo"
    #make the robot keep doing something...
    rospy.sleep(rospy.Duration(5, 0))
    #driveStraight(0.25,.5)
    #rotate(90)
    #driveArc(.5,.5,90)
    while(not hasPath):
        rospy.sleep(.1)
    print "received path"
    navReady = True
    while(1 and not rospy.is_shutdown()):
        for next in path:
            if navReady:
                navToPose(next)
            break
        navReady = True
    # Make the robot do stuff...
    print "Lab 2 complete!"

