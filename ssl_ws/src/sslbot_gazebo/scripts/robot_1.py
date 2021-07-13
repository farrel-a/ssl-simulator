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
x1 = 0.0
y1 = 0.0 
theta1 = 0.0
rot1_q = Quaternion()

def newOdom1(msg):
    global x1
    global y1
    global theta1
    global rot1_q

    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    rot1_q = msg.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion([rot1_q.x, rot1_q.y, rot1_q.z, rot1_q.w])

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

def isInEnemyPenalty(a,b):
    if ((-7.0<=a<=-4.0) and (-0.5<=b<=0.5)):
        return True
    else:
        False

def isDribbling (d):
    # d : distance
    if (d < 0.15):
        return True
    else:
        return False

def shouldPass (d1,d2):
    #d1 : distance between robot_1 and enemy goal
    #d2 : distance between robot_1 and robot_3
    if d1<d2:
        return False #do not pass, go to enemy goal
    else:
        return True #pass to robot_3

rospy.init_node("robot_1")
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
    distance = ((goal.x-x1)**2 + (goal.y-y1)**2)**0.5
    dribbling = isDribbling(distance)
    if (dribbling): #dribbling == True
        if (speed.linear.x==0 and speed.angular.z==0 and (abs(angle_to_friend-theta1) < 0.01) and not(passing)):
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
            ballstate.model_state.pose.position.x = x1 + (0.09*math.sin((math.pi-(theta1))-(math.pi/2)))
            ballstate.model_state.pose.position.y = y1 + (0.09*math.cos((math.pi-(theta1))-(math.pi/2)))
            ballstate.model_state.pose.position.z = 0.05
            ballstate.model_state.pose.orientation = rot1_q
            ballstate.model_state.reference_frame = "world"
            set_ball_service(ballstate) #call set_model_state to be in front of bot

    else: #dribbling == False
        passing = False
        goal.x = msg.pose[0].position.x # x values of ball
        goal.y = msg.pose[0].position.y # y values of ball

# Publisher & Subscriber definition
sub = rospy.Subscriber("/robot_1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
sub3 = rospy.Subscriber("/robot_3/odom", Odometry, newOdom3)
pub = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size = 10)
pub2 = rospy.Publisher("/ball_on_robot_t1", Int64, queue_size=1)

speed = Twist() #global speed variable

r = rospy.Rate(1000) #1000 Hz
while not rospy.is_shutdown():
    if (dribbling):
        pub2.publish(1) # 1 : dribbling published (one of robot in t1 is dribbling)

    inc_x = goal.x -x1
    inc_y = goal.y -y1
    inc_x_friend = x3 - x1
    inc_y_friend = y3 - y1
    angle_to_goal = atan2(inc_y, inc_x)
    angle_to_friend = atan2(inc_y_friend, inc_x_friend)

    if (not(dribbling) and (not(passing))):
        if (angle_to_goal - theta1) > 0.2:
            speed.linear.x = 0.0
            speed.angular.z = 0.4
        elif (angle_to_goal - theta1) < -0.2:
            speed.linear.x = 0.0
            speed.angular.z = -0.4
        else:
            speed.angular.z = 0.0 
            speed.linear.x = 0.6
    else:
        if (angle_to_friend - theta1) > 0.01:
            speed.angular.z = 0.4
            speed.linear.x = 0.0
        elif (angle_to_friend - theta1) < -0.01:
            speed.angular.z = -0.4
            speed.linear.x = 0.0
        else :
            speed.angular.z = 0.0
            speed.linear.x = 0.0


    pub.publish(speed)
    r.sleep()