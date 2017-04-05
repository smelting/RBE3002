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


def publishGrid(cells):
	gridMsg = GridCells()
	gridMsg.header.stamp = rospy.Time.now()
	gridMsg.header.frame_id = '/map'
	gridMsg.cell_width = .2
	gridMsg.cell_height = .2
	gridMsg.cells = cells
	pubGrid.publish(gridMsg)

if __name__ == '__main__':
	rospy.init_node('lab3_grid_cell')

	tempGrid = []

	tempGrid.append(makeGridCell(1,1))
	tempGrid.append(makeGridCell(3,6))
	pubGrid = rospy.Publisher('/rand_grid', GridCells, queue_size = 1)
	publishGrid(tempGrid)
	print "published"
	while(1):
		rospy.sleep(.25)
		publishGrid(tempGrid)


