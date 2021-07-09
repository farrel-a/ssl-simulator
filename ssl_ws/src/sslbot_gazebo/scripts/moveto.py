#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import time

# initialize as float
x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("moveto")

sub = rospy.Subscriber("/robot_1/odom", Odometry, newOdom)
pub = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(1000)

goal = Point()
goal.x = 0
goal.y = 0

while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)

    if (angle_to_goal - theta) > 0.2:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    elif (angle_to_goal - theta) < -0.2:
        speed.linear.x = 0.0
        speed.angular.z = -0.3
    else:
        speed.angular.z = 0.0 
        speed.linear.x = 0.8

    pub.publish(speed)
    r.sleep()