#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.msg import ModelStates
from math import atan2
import time
from gazebo_msgs.srv import *
from std_srvs.srv import *
import math

# initialize as float
x = 0.0
y = 0.0 
p = 0.0
q = 0.0 
theta = 0.0
theta2 = 0.0
ori = []
angle_to_friend = 1000.0

def newOdom(msg):
    global x
    global y
    global theta
    global ori

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    ori = []

    rot_q = msg.pose.pose.orientation
    ori.append(rot_q.x)
    ori.append(rot_q.y)
    ori.append(rot_q.z)
    ori.append(rot_q.w)
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def isInEnemyPenalty(a,b):
    if ((-7.0<=a<=-4.0) and (-0.5<=b<=0.5)):
        return True
    else:
        False

def newOdom2(msg):
    global p
    global q
    global theta2

    p = msg.pose.pose.position.x
    q = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta2) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("pass")
rospy.wait_for_service('/gazebo/set_model_state')
set_ball_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

ballstate = SetModelStateRequest() #initialize ballstate
goal = Point() #global goal variable
passing = False
ball_x = 0.0
ball_y = 0.0
distance = 0.0
def ballPos(msg):
    global goal
    global ballstate
    global passing
    global ball_x
    global ball_y
    global distance
    distance = ((goal.x-x)**2 + (goal.y-y)**2)**0.5
    if (distance > 0.15): #distance between ball and bot needed to dribble the ball
        goal.x = msg.pose[0].position.x # x values of ball
        goal.y = msg.pose[0].position.y # y values of ball

    else:
        if (speed.linear.x==0 and speed.angular.z==0 and (abs(angle_to_friend-theta) <= 0.05)):
            ball_x = x + (0.09*math.sin((math.pi-(theta))-(math.pi/2)))
            ball_y = y + (0.09*math.cos((math.pi-(theta))-(math.pi/2)))
            ballstate.model_state.model_name = "ssl_ball_1"
            ballstate.model_state.reference_frame = "ssl_ball_1"
            ballstate.model_state.pose.position.x = 0
            ballstate.model_state.pose.position.y = 0
            ballstate.model_state.pose.position.z = 0.05
            ballstate.model_state.twist.linear.x = 2.0
            set_ball_service(ballstate)
            time.sleep(3)
            passing = True
        else:
            ballstate.model_state.model_name = "ssl_ball_1"
            ballstate.model_state.pose.position.x = x + (0.09*math.sin((math.pi-(theta))-(math.pi/2)))
            ballstate.model_state.pose.position.y = y + (0.09*math.cos((math.pi-(theta))-(math.pi/2)))
            ballstate.model_state.pose.position.z = 0.05
            ballstate.model_state.pose.orientation.x = ori[0]
            ballstate.model_state.pose.orientation.y = ori[1]
            ballstate.model_state.pose.orientation.z = ori[2]
            ballstate.model_state.pose.orientation.w = ori[3]
            set_ball_service(ballstate) #call set_model_state to be in front of bot
            ball_x = x + (0.09*math.sin((math.pi-(theta))-(math.pi/2)))
            ball_y = y + (0.09*math.cos((math.pi-(theta))-(math.pi/2)))



sub = rospy.Subscriber("/robot_1/odom", Odometry, newOdom)
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
sub3 = rospy.Subscriber("/robot_3/odom", Odometry, newOdom2)
pub = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size = 10)

speed = Twist()

r = rospy.Rate(1000)
while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y
    inc_x_friend = p -x
    inc_y_friend = q -y
    angle_to_goal = atan2(inc_y, inc_x)
    angle_to_friend = atan2(inc_y_friend, inc_x_friend)
    if (distance >= 0.15 and (not(passing))):
        if (angle_to_goal - theta) > 0.2:
            speed.linear.x = 0.0
            speed.angular.z = 0.4
        elif (angle_to_goal - theta) < -0.2:
            speed.linear.x = 0.0
            speed.angular.z = -0.4
        else:
            speed.angular.z = 0.0 
            speed.linear.x = 0.6
    else:
        if (angle_to_friend - theta) > 0.05:
            speed.angular.z = 0.2
            speed.linear.x = 0.0
        elif (angle_to_friend - theta) < -0.05:
            speed.angular.z = -0.2
            speed.linear.x = 0.0
        else :
            speed.angular.z = 0.0
            speed.linear.x = 0.0

    pub.publish(speed)
    r.sleep()