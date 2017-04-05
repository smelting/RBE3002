import rospy, math, tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point

def makeGridCell(x, y):
	point = Point()
	point.x = x
	point.y = y
	point.z = 0
	return point


def publishGrid(cells, type):
	gridMsg = GridCells()
	gridMsg.header.stamp = rospy.Time.now()
	gridMsg.header.frame_id = '/map'
	gridMsg.cell_width = .2
	gridMsg.cell_height = .2
	gridMsg.cells = cells
	if type == 'frontier':
		frontierGrid.publish(gridMsg)
	elif type == 'visited':
		visitedGrid.publish(gridMsg)
	elif type == 'notVisited':
		notVisitedGrid.publish(gridMsg)
	else:
		pubGrid.publish(gridMsg)

def frontierCallBack(msg):
	publishGrid(msg, 'frontier')

def visitedCallBack(msg):
	publishGrid(msg, 'visited')

def notVisitedCallBack(msg):
	publishGrid(msg, 'notVisited')

if __name__ == '__main__':
	rospy.init_node('lab3_grid_cell')

	#subscribers
	occupancy_sub = rospy.Subscriber('/map',frontierCallBack, queue_size = 1)
	occupancy_sub = rospy.Subscriber('/map',visidtedCallBack, queue_size = 1)
	occupancy_sub = rospy.Subscriber('/map',notVisitedCallBack, queue_size = 1)

	#publishers
	pubGrid = rospy.Publisher('/rand_grid', GridCells, queue_size = 1)
	frontierGrid = rospy.Publisher('/frontier_grid', GridCells, queue_size = 1)
	visitedGrid = rospy.Publisher('/visited_grid', GridCells, queue_size = 1)
	notVisitedGrid = rospy.Publisher('/not_visited_grid', GridCells, queue_size = 1)


	tempGrid = []
	tempGrid.append(makeGridCell(1,1))
	tempGrid.append(makeGridCell(3,6))
	tempGrid.append(makeGridCell(0,4))

	publishGrid(tempGrid, 'random')
	print "published"
	while(1):
		rospy.sleep(.25)
		publishGrid(tempGrid)


