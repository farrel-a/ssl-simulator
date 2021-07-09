#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.msg import ModelStates
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

goal = Point() #global goal variable

def ballPos(msg):
    global goal
    goal.x = msg.pose[0].position.x # x values of ball
    goal.y = msg.pose[0].position.y # y values of ball


sub = rospy.Subscriber("/robot_1/odom", Odometry, newOdom)
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
pub = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(1000)

while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)

    if (angle_to_goal - theta) > 0.3:
        speed.linear.x = 0.0
        speed.angular.z = 0.8
    elif (angle_to_goal - theta) < -0.3:
        speed.linear.x = 0.0
        speed.angular.z = -0.8
    else:
        speed.angular.z = 0.0 
        speed.linear.x = 0.5

    pub.publish(speed)
    r.sleep()