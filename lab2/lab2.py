#!/usr/bin/env python

import rospy, tf
import math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


MAX_ANGULAR_V = 1  #max angular speed allowed
SLEW_STEP = 0.0125  #step amount 
SLEW_STEP_TIME = 0.075 #time between steps
SLEW_STEPS = 50 #num of steps to have before full speed is reached
# Add additional imports for each of the message types used


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	global busy  #the robot is busy 
	busy = True
	goalPose = Pose() #pose to end at

	goalPose.position.x = goal.pose.position.x
	goalPose.position.y = goal.pose.position.y
	orientation = goal.pose.orientation
	q = [orientation.x,orientation.y,orientation.z,orientation.w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
	thetaGoal = math.degrees(yaw) #theta to end at

	x = goalPose.position.x - pose.position.x #distance to go in x
	y = goalPose.position.y - pose.position.y #distance to go in y
	print "Goal x: %f y: %f theta: %f" % (x,y,thetaGoal)

	angle = math.atan2(y,x)  #angle in global needed to drive to desired point
	angle = math.degrees(angle)
	print "rotating to goal %f" % angle
	rotate(angle) #rotate to point at goal point
	distanceToGo = calcDistance(goalPose,pose) #drive to the point
	print "driving to goal %f" % distanceToGo
	driveStraight(.25,distanceToGo) 
	print "rotating to final pose %f" % thetaGoal
	rotate(thetaGoal) #rotate to have the desired end theta
	print "Went to x: %f y: %f theta: %f" % (xPos,yPos,theta)
	stop() #stop the chassis
	print "Done with navigation"
	busy = False #not busy anymore. soooo
	


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	#this is called when the bumper is pressed
	print "Don't bump me asshole" 
	driveStraight(.26,.6)  #drive straight
	rotate(90) #rotate to 90 global
	driveStraight(0.25,.45) #drive straight
	rotate(135) #rotate to global 135 degrees




#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	timeStamp = rospy.get_time() #when this function was called
	if(u1 != u2): #check if we are set to go straight
		radius = (u1+u2)/(2*(u2-u1))
		angular = (u2 - u1)/9
		linear = angular*radius
	else: #going straight
		angular  = 0
		linear = u1

	start_angular = twist.angular.z #grab the start speed as how fast the robot is currently moving when this function is called
	start_linear = twist.linear.x #angular speed of the robot
	if(angular > MAX_ANGULAR_V): #cap the angular speed 
		angular = MAX_ANGULAR_V
	if(angular < -MAX_ANGULAR_V):
		angular = -MAX_ANGULAR_V
	while(rospy.get_time() - timeStamp < time): #check and see if we should be done moving
		if(start_linear < linear): #slew, increment the linear speed each loop
			start_linear += SLEW_STEP
		if(start_angular < angular): #increment the angular speed each loop
			start_angular += SLEW_STEP
		#print "linear %f angular %f" % (start_linear,start_angular)
		publishTwist(linear,angular) #publish the desired speeds
		rospy.sleep(SLEW_STEP_TIME) #sleep for the slew stime

def stop():
	publishTwist(0,0) #tell the chassis to stop

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	start_pose = pose #the pose of the robot when function is called
	#print "Start x: %f y: %f" % (start_pose.position.x,start_pose.position.y)
	while(calcDistance(pose,start_pose) < distance): #check if we have driven far enough
		#print "X: %f Y: %f" % (pose.position.x,pose.position.y)
		#print "%f" % calcDistance(pose,start_pose)
		#print "Pose: x: %f y: %f Dist: %f" % (pose.position.x,pose.position.y,calcDistance(pose,start_pose))
		spinWheels(speed,speed,.01) #same speed for both wheels
		#print "Done with Straight path"
	stop() #we're done!


def calcDistance(pose1,pose2):
	#distance between to poses
	distance = math.sqrt(math.pow(pose1.position.x-pose2.position.x,2) + math.pow(pose1.position.y-pose2.position.y,2))
	return distance


#publishes the twist message given a scalar linear and angular speed
def publishTwist(linear,angular):
	twist = Twist()
	twist.linear.x = linear
	twist.angular.z = angular
	pub.publish(twist)
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle): 
	#get within 1 degree of the desired angle
	while(math.fabs(theta - angle) >= 1): #if we're within 1 degree of the desired angle in global
		error = theta - angle #calculate the error 
		spinWheels(error*0.4,-error*0.4,.01) #use a gain to determine the speed of rotation
		#print "Desired %f cur %f pwr: %f" % (angle, theta,error*0.4)
	stop() #done with rotation



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	angular = speed/radius #calculate the angular speed
	linear = speed #set the linear speed from the speed given
	start_angle = theta #the starting angle of the robot

	start_linear = 0 #speed starts at 0
	start_angular = 0

	step_linear = linear/SLEW_STEPS  #calculate the step given a number of step to be at full speed by
	step_angular = angular/SLEW_STEPS
	while(math.fabs(theta - start_angle) < angle): #check if we've rotated so many degrees
		if(start_linear < linear): 
			start_linear += step_linear #increase speed by the previously calculated step
		if(start_angular < angular):
			start_angular += step_angular
		publishTwist(start_linear,start_angular) #Send the SPEEEEEDS
		rospy.sleep(SLEW_STEP_TIME)  #chill out for a bit
	stop() #you're there!



#Bumper Event Callback function
def readBumper(msg): #bumper has been read
    if (msg.state == 1):
    	executeTrajectory()

def getOdom(): #create a subcriber to the odom info
	odomSub = rospy.Subscriber("/odom", Odometry, odomCallBack)

def odomCallBack(data):
	global xPos #create some useful variables 
	global yPos
	global theta
	global pose
	global twist
	pose = Pose() 
	twist = Twist()
	twist = data.twist.twist #grabs the robots current speeds
	xPos = data.pose.pose.position.x #grabs the robots current x and y pos
	yPos = data.pose.pose.position.y
	orientation = data.pose.pose.orientation #grab the robots current orientation
	q = [orientation.x,orientation.y,orientation.z,orientation.w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(q) #convert quaternion to radians
	theta = math.degrees(yaw) #convert the angle into degrees
	pose.position.x = xPos #set all the data we just grabbed to our global pose
	pose.position.y = yPos
	pose.orientation = orientation

	goal = PoseStamped() #create a poseStamped to show in rviz 
	goal.header.frame_id = "/base_link" 
	goal.header.stamp = rospy.Time.now() #should actually be the same time as the odom info but I don't care
	goal.pose.position.z = 0
	goal.pose.position.x = xPos
	goal.pose.position.y = yPos
	goal.pose.orientation = pose.orientation
	pos_pub.publish(goal) #publish this message everytime we get some new odom info

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('Lab2_nav')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub #publisher for the teleop input
    global odom_tf 
    global odom_list
    global pose

    global busy
    busy = False #if the robot is busy, used so that the bumper doesn't trigger while navigating to a path
    pose = Pose() 
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1) # Publisher for commanding robot motion
    pos_pub = rospy.Publisher("/lab2/pose",PoseStamped, queue_size = 1) #poseStamped for visualization in rviz
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    nav_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped, navToPose, queue_size = 1) #subcribe to the 2d nav goals in rviz
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"
    getOdom() #make the odom sub
    print "yo"
    #make the robot keep doing something...
    rospy.sleep(rospy.Duration(5, 0)) #wait for odom info to come in
    #driveStraight(0.25,.5)
    #rotate(90)
    driveArc(.5,.5,90)
    while(1):
    	rospy.sleep(.5) #chill out forever..
    # Make the robot do stuff...
    print "Lab 2 complete!"

