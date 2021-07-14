#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int64
from math import atan2
import time
from gazebo_msgs.srv import *
from std_srvs.srv import *
import math

# initialize as float
x2 = 0.0
y2 = 0.0 
theta2 = 0.0
rot2_q = Quaternion()

def newOdom2(msg):
    global x2
    global y2
    global theta2
    global rot2_q

    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

    rot2_q = msg.pose.pose.orientation
    (roll, pitch, theta2) = euler_from_quaternion([rot2_q.x, rot2_q.y, rot2_q.z, rot2_q.w])

x3 = 0.0
y3 = 0.0
theta3 = 0.0
rot3_q = Quaternion()

def newOdom3(msg):
    global x3
    global y3
    global theta3
    global rot3_q
    x3 = msg.pose.pose.position.x
    y3 = msg.pose.pose.position.y

    rot3_q = msg.pose.pose.orientation
    (roll, pitch, theta3) = euler_from_quaternion([rot3_q.x, rot3_q.y, rot3_q.z, rot3_q.w])

def isDribbling (d):
    # d : distance
    if (d < 0.15):
        return True
    else:
        return False

rospy.init_node("keeper")
rospy.wait_for_service('/gazebo/set_model_state')
set_ball_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

ballstate = SetModelStateRequest() #global ballstate request varible
goal = Point() #global goal variable
visited = False
# ball_x = 0.0
# ball_y = 0.0
dribbling = False #global driblling variable
passing = False #global passing condition (True : should pass, False : should not pass)
distance = 0.0

def ballPos(msg):
    global goal
    global ballstate
    global dribbling
    global visited
    global distance
    global passing
    distance = ((goal.x-x2)**2 + (goal.y-y2)**2)**0.5
    dribbling = isDribbling(distance)
    if (dribbling): #dribbling == True
        if (planar_speed.linear.y==0 and planar_speed.angular.z==0 and (abs(angle_to_friend3-theta2) < 0.01) and not(passing)):
            ballstate.model_state.model_name = "ssl_ball_1"
            ballstate.model_state.pose.position.x = 0.0
            ballstate.model_state.pose.position.y = 0.0
            ballstate.model_state.pose.position.z = 0.0
            ballstate.model_state.twist.linear.x = 2.0
            ballstate.model_state.reference_frame = "ssl_ball_1"
            set_ball_service(ballstate)
            time.sleep(3)
            passing = True

        elif (not(passing)): #passing == False
            ballstate.model_state.model_name = "ssl_ball_1"
            ballstate.model_state.pose.position.x = x2 + (0.09*math.sin((math.pi-(theta2))-(math.pi/2)))
            ballstate.model_state.pose.position.y = y2 + (0.09*math.cos((math.pi-(theta2))-(math.pi/2)))
            ballstate.model_state.pose.position.z = 0.05
            ballstate.model_state.pose.orientation = rot2_q
            ballstate.model_state.reference_frame = "world"
            set_ball_service(ballstate) #call set_model_state to be in front of bot

    else: #dribbling == False
        passing = False
        goal.x = msg.pose[0].position.x # x values of ball
        goal.y = msg.pose[0].position.y # y values of ball

# Publisher & Subscriber definition
sub = rospy.Subscriber("/robot_2/odom", Odometry, newOdom2)
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
sub3 = rospy.Subscriber("/robot_3/odom", Odometry, newOdom3)
pub2 = rospy.Publisher("/robot_2/planar_vel", Twist, queue_size = 10)

speed = Twist() #global speed variable
planar_speed = Twist()

r = rospy.Rate(1000) #1000 Hz
while not rospy.is_shutdown():


    inc_x = goal.x -x2
    inc_y = goal.y -y2
    inc_x_friend3 = x3 - x2
    inc_y_friend3 = y3 - y2
    angle_to_goal = atan2(inc_y, inc_x)
    angle_to_friend3 = atan2(inc_y_friend3, inc_x_friend3)

    if (not(dribbling) and (not(passing))):
        if (inc_y) > 0.15:
            if (inc_y) < 0.3:
                planar_speed.linear.y = -0.3
            elif (y2 < 0.85) :
                planar_speed.linear.y = -0.6
            else:
                planar_speed.linear.y = 0.0
        elif (inc_y) < -0.15:
            if (inc_y) > -0.3:
                planar_speed.linear.y = 0.3
            if (y2 > -0.85) :
                planar_speed.linear.y = 0.6
            else:
                planar_speed.linear.y = 0.0
        else:
            planar_speed.linear.y = 0.0
    elif (dribbling):
        if (angle_to_friend3 - theta2) > 0.01:
            planar_speed.angular.z = 0.5
            planar_speed.linear.y = 0.0
        elif (angle_to_friend3 - theta2) < -0.01:
            planar_speed.angular.z = -0.5
            planar_speed.linear.y = 0.0
        else :
            planar_speed.angular.z = 0.0
            planar_speed.linear.y = 0.0


    pub2.publish(planar_speed)
    r.sleep()
