#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
#from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt

yaw = 0
obstacles = np.zeros((1, 1))
currentx = 0
currenty = 0
scale = 0.32
neighbours = np.zeros((1, 1))
obst_neighbours = []


def findNeighbours(i, j):  # i is y and j is x
    neighbours = []
    for k in range(-1, 2):
        for l in range(-1, 2):
            if((i+k) < 505 and (l+j) < 555 and (i+k) >= -505 and (l+j) >= -555 and (k != 0 or l != 0)):
                neighbours.append((l+j, i+k))
    return neighbours


def callback_laser(data):
    global yaw
    global currentx
    global currenty
    global obstacles
    global scale
    global neighbours
    global obst_neighbours
    
    obst_neighbours = []
    #range_min = data.range_min
    ranges = np.array(data.ranges)
    if(min(ranges) <= scale):
        obstacles = [(round(currentx + ranges[i]*100*math.cos(i*(math.pi/180)+yaw)), round(currenty +
                                                                                           ranges[i]*100*math.sin(i*(math.pi/180)+yaw))) for i in range(len(ranges)) if ranges[i] <= scale]
        neighbours = np.array(findNeighbours(currenty, currentx))
        obstacles = np.array(obstacles)
        for obstacle in obstacles:
            min_dist = float("inf")
            obst_neighbour = 0
            for neighbour in neighbours:
                dist = (neighbour[0] - obstacle[0])**2 + \
                    (neighbour[1] - obstacle[1])**2
                if(dist < min_dist):
                    min_dist = dist
                    obst_neighbour = neighbour
            obst_neighbours.append((obst_neighbour))
        obst_neighbours = np.array(obst_neighbours)
        obst_neighbours = np.unique(obst_neighbours,axis=0)
        print(obst_neighbours)
        """        # print(obstacles.shape)
        plt.ion
        plt.plot(obst_neighbours[:, 0], obst_neighbours[:, 1], 'ro')
        plt.plot(currentx, currenty, 'go')
        #rospy.loginfo("Obstacles = %s", str((obst_neighbours))
        plt.show()
        plt.pause(0.001)
        """


def callback_angle(data):
    global yaw
    global currentx
    global currenty
    pose = data.pose.pose.position
    currentx = round(pose.x*100)
    currenty = round(pose.y*100)
    rot_q = data.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #rospy.loginfo("Pose = %d, %d", currentx,currenty)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback_laser)
    rospy.Subscriber("odom", Odometry, callback_angle)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
