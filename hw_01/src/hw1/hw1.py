#! /usr/bin/python
import math
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Predator:
	def __init__(self):
		self.x_2 = 0
		self.y_2 = 0
		self.z_2 = 0
		rospy.Subscriber('/turtle1/pose', Pose, self.callback1)
		rospy.Subscriber('/turtle2/pose', Pose, self.callback2)
		self.pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

	def callback2(self, msg):
		self.x_2 = msg.x
		self.y_2 = msg.y
		self.z_2 = msg.theta
	
	def callback1(self, msg):
		x_1 = msg.x
		y_1 = msg.y
		
		msg_2 = Twist()
		msg_2.linear.x = x_1 - self.x_2
		msg_2.linear.y = y_1 - self.y_2
		msg_2.angular.z = -self.z_2 + math.atan2(y_1 - self.y_2, x_1 - self.x_2)
		self.pub_turtle2.publish(msg_2)
		
	
		
rospy.init_node('Predator')
Predator()
rospy.spin()	
