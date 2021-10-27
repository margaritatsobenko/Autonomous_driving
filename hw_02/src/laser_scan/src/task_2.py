#! /usr/bin/python

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class LaserMap:
	def __init__(self):
		self.subscriber = rospy.Subscriber('/base_scan', LaserScan, self.callback)
		self.publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
		self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
		
	def filter(self, msg):	
		ranges = msg.ranges
		good_dots = [0]
		for i in range(2, len(ranges)):
			cur_mean = (ranges[i - 2] + ranges[i])/2
			if abs(ranges[i - 1] - cur_mean) < 0.05:
				good_dots.append(i - 1)			
		good_dots.append(len(ranges) - 1)
		return good_dots
	
	def create_marker(self):
		marker = Marker()

		marker.header.frame_id = "base_laser_link"
		marker.header.stamp = rospy.Time.now()

		# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
		marker.type = 8
		marker.id = 0
		marker.action = 0

		# Set the scale of the marker
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
	
		# Set the color
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0
	
		# Set the pose of the marker
		marker.pose.position.x = 0
		marker.pose.position.y = 0
		marker.pose.position.z = 0
		
		return marker
		
	def create_grid(self, x_coord, y_coord, radius=7, resolution=1e-1):
		grid = OccupancyGrid()
		grid.info.resolution = resolution
		grid.header.frame_id = "base_laser_link"
		size = 2 * int(radius / resolution) + 1
		grid.info.width = size
		grid.info.height = size
		
		data = np.zeros((size, size), dtype=int)
		
		for x, y in zip(x_coord, y_coord):
			if abs(x) < radius and abs(y) < radius:
				i = int((x + radius) / resolution)
				j = int((y + radius) / resolution)
				data[i, j] = 100
			
		data = data.transpose().flatten()
		grid.data = data
		
		grid.info.origin.position.x = -radius
		grid.info.origin.position.y = -radius
		grid.info.origin.position.z = 0
		
		return grid
		
	def callback(self, msg):	
		good_dots = self.filter(msg)
		
		ranges = np.array(msg.ranges)[good_dots]
		
		alphas = [msg.angle_min + i * msg.angle_increment for i in good_dots]
		
		x_coord = [ranges[i] * np.cos(alphas[i]) for i in range(len(good_dots))]
		y_coord = [ranges[i] * np.sin(alphas[i]) for i in range(len(good_dots))]
		
		marker = self.create_marker()
		marker.points = [Point(x, y, 0) for x, y in zip(x_coord, y_coord)]
		self.marker_pub.publish(marker)
		grid = self.create_grid(x_coord, y_coord)
		
		self.publisher.publish(grid)
		
rospy.init_node("task_2_code")
LaserMap()
rospy.spin()
		
