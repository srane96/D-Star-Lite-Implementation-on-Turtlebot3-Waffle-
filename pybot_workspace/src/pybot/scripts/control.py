#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

x = 0.0
y = 0.0
theta = 0.0


def newOdom(msg):
	global x
	global y
	global theta	
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	(_, _, theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])



rospy.init_node('speed_controller')

sub = rospy.Subscriber('/odom', Odometry, newOdom)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(4)

path = [(1,1), (1,2), (1,3), (2,3), (2,4), (3,4), (4,4), (5,5)]


new_path = []
for i in path:
	goal = Point()	
	goal.x = i[0]
	goal.y = i[1]	
	new_path.append(goal)

print(new_path)
vel = Twist()

while not rospy.is_shutdown():
	for g in new_path:
		print("Heading towards: ", (g.x, g.y))
		print("Current position: ", (x, y))
		del_x = g.x - x
		del_y = g.y - y	
		while sqrt((del_x)**2+(del_y)**2) > 0.1: 	
			del_x = g.x - x
			del_y = g.y - y	
			angle_to_goal = atan2(del_y, del_x)

			if (angle_to_goal - theta) > 0.1:
				
				vel.linear.x = 0.0
				vel.angular.z = 0.3
			elif (theta - angle_to_goal) > 0.1:
				vel.linear.x = 0.0
				vel.angular.z = -0.3
			else:
				
				vel.linear.x = 0.3	
				vel.angular.z = 0.0
	
			pub.publish(vel)
			rate.sleep()
		vel.linear.x = 0.0	
		vel.angular.z = 0.0
		print("Goal reached:", (g.x, g.y))
		print("Odom position: ", (x, y))	
		pub.publish(vel)
		rospy.sleep(1)
	vel.linear.x = 0.0	
	vel.angular.z = 0.0
	print("Goal reached:", path[-1])	
	print("Odom position: ", (x, y))
	pub.publish(vel)
	rospy.sleep(5	)


