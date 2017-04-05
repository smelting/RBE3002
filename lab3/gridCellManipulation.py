import rospy, math, tf
from nav_msgs import OccupancyGrid
from geometry_msgs.msg import Point

def makeGridCell(x, y):
	point = Point()
	point.x = x
	point.y = y
	point.z = 0
	return point


def publishGrid(cells):
	global grid
	gridMsg = GridCells()
	gridMsg.header.frame_id = 'map'
	gridMsg.cell_width = .2
	gridMsg.cell_height = .2
	gridMsg.cells = cells
	grid.publish(gridMsg)

if __name__ == '__main__':
	global pubGrid
	global tempGrid

	tempGrid = []

	tempGrid.append(makeGridCell(1,1))
	tempGrid.append(makeGridCell(3,6))

	pubGrid = rospy.Publisher('randGrid', GridCells, queue_size = 10)


