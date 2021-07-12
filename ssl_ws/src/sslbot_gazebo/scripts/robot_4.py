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
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def isInEnemyPenalty(a,b):
    if ((4.0<=a<=7.0) and (-0.5<=b<=0.5)):
        return True
    else:
        False

rospy.init_node("robot_4")
rospy.wait_for_service('/gazebo/set_model_state')
set_ball_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

ballstate = SetModelStateRequest() #initialize ballstate
goal = Point() #global goal variable
visited = False
ball_x = 0.0
ball_y = 0.0
def ballPos(msg):
    global goal
    global ballstate
    global visited
    global ball_x
    global ball_y
    distance = ((goal.x-x)**2 + (goal.y-y)**2)**0.5
    if (distance < 0.15): #distance between ball and bot needed to dribble the ball
        if ((1.9<=x<=2.1) and (1.9<=y<=2.1)):
            visited = True

        if not(visited):
            goal.x = 2.0
            goal.y = 2.0
        else:
            goal.x = 4.5
            goal.y = 0.0

        if (not(isInEnemyPenalty(x,y))):
            ballstate.model_state.model_name = "ssl_ball_1"
            ballstate.model_state.pose.position.x = x + (0.09*math.sin((math.pi-(theta))-(math.pi/2)))
            ballstate.model_state.pose.position.y = y + (0.09*math.cos((math.pi-(theta))-(math.pi/2)))
            ballstate.model_state.pose.position.z = 0.05
            set_ball_service(ballstate) #call set_model_state to be in front of bot
        elif (isInEnemyPenalty(x,y)): 
            if (speed.linear.x==0 and speed.angular.z==0 and (0.0<=abs(theta)<=0.05)):
                ball_x = x + (0.09*math.sin((math.pi-(theta))-(math.pi/2)))
                ball_y = y + (0.09*math.cos((math.pi-(theta))-(math.pi/2)))
                ballstate.model_state.model_name = "ssl_ball_1"
                ballstate.model_state.pose.position.x = ball_x
                ballstate.model_state.pose.position.y = ball_y
                ballstate.model_state.pose.position.z = 0.05
                ballstate.model_state.twist.linear.x = 2.0
                set_ball_service(ballstate)
                time.sleep(3)
                visited = False
            else:
                ballstate.model_state.model_name = "ssl_ball_1"
                ballstate.model_state.pose.position.x = x + (0.09*math.sin((math.pi-(theta))-(math.pi/2)))
                ballstate.model_state.pose.position.y = y + (0.09*math.cos((math.pi-(theta))-(math.pi/2)))
                ballstate.model_state.pose.position.z = 0.05
                set_ball_service(ballstate) #call set_model_state to be in front of bot
                ball_x = x + (0.09*math.sin((math.pi-(theta))-(math.pi/2)))
                ball_y = y + (0.09*math.cos((math.pi-(theta))-(math.pi/2)))

    else:
        goal.x = msg.pose[0].position.x # x values of ball
        goal.y = msg.pose[0].position.y # y values of ball


sub = rospy.Subscriber("/robot_4/odom", Odometry, newOdom)
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
pub = rospy.Publisher("/robot_4/cmd_vel", Twist, queue_size = 10)

speed = Twist()

r = rospy.Rate(1000)
while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y
    angle_to_goal = atan2(inc_y, inc_x)
    if not(isInEnemyPenalty(x,y)):
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
        if ((0.0<=abs(theta)<=0.05) and speed.angular.z != 0):
            speed.angular.z = 0.0
            speed.linear.x = 0.0
        elif ((0.0<=abs(theta)<=0.05) and speed.angular.z == 0):
            speed.angular.z = 0.0
            speed.linear.x = 0.0
        else :
            speed.angular.z = 0.2
            speed.linear.x = 0.0

    pub.publish(speed)
    r.sleep()