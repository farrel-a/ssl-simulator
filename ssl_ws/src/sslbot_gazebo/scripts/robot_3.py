#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int8
from math import atan2
import time
from gazebo_msgs.srv import *
from std_srvs.srv import *
import math

# initialize as float


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

x5 = 0.0
y5 = 0.0
theta5 = 0.0
rot5_q = Quaternion()
def newOdom5(msg):
    global x5
    global y5
    global theta5
    global rot5_q
    x5 = msg.pose.pose.position.x
    y5 = msg.pose.pose.position.y

    rot5_q = msg.pose.pose.orientation
    (roll, pitch, theta5) = euler_from_quaternion([rot5_q.x, rot5_q.y, rot5_q.z, rot5_q.w])

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

rospy.init_node("robot_3")
rospy.wait_for_service('/gazebo/set_model_state')
set_ball_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

ballstate = SetModelStateRequest() #global ballstate request varible
goal = Point() #global goal variable
visited = False
dribbling = False #global driblling variable
passing = False #global passing condition (True : should pass, False : should not pass)
direction = 0.0 #global shoot direction (in rad)
shooting = False #global shooting variable
x_ball = 0.0
y_ball = 0.0

def ballPos(msg):
    global goal
    global shooting
    global ballstate
    global dribbling
    global visited
    global x_ball
    global y_ball
    x_ball = msg.pose[0].position.x
    y_ball = msg.pose[0].position.y
    distance = ((x_ball-x3)**2 + (y_ball-y3)**2)**0.5
    dribbling = isDribbling(distance)
    if (bt2):
        shooting = False
        goal.x = 5.2
        goal.y = 1.2
    else: #bt2 == False
        if (dribbling): #dribbling == True
            goal.x = -4.50
            goal.y = -0.35

            if (not(isInEnemyPenalty(x3,y3))):
                shooting = False
                ballstate.model_state.model_name = "ssl_ball_1"
                ballstate.model_state.pose.position.x = x3 + (0.11*math.sin((math.pi-(theta3))-(math.pi/2)))
                ballstate.model_state.pose.position.y = y3 + (0.11*math.cos((math.pi-(theta3))-(math.pi/2)))
                ballstate.model_state.pose.position.z = 0.01
                ballstate.model_state.pose.orientation = rot3_q
                ballstate.model_state.reference_frame = "world"
                set_ball_service(ballstate) #call set_model_state to set ball in front of bot

            elif (isInEnemyPenalty(x3,y3) and not(shooting)): 
                if (speed.linear.x==0 and speed.angular.z==0 and (2.90<=abs(theta3)<=2.92)):
                    ballstate.model_state.model_name = "ssl_ball_1"
                    ballstate.model_state.pose.position.x = 0.0
                    ballstate.model_state.pose.position.y = 0.0
                    ballstate.model_state.pose.position.z = 0.0
                    ballstate.model_state.twist.linear.x = 3.0
                    ballstate.model_state.reference_frame = "ssl_ball_1"
                    set_ball_service(ballstate)  #shoot according to the robot's orientation
                    dribbling = False
                    shooting = True
                    time.sleep(3)
                    visited = False
                elif not(shooting):
                    ballstate.model_state.model_name = "ssl_ball_1"
                    ballstate.model_state.pose.position.x = x3 + (0.11*math.sin((math.pi-(theta3))-(math.pi/2)))
                    ballstate.model_state.pose.position.y = y3 + (0.11*math.cos((math.pi-(theta3))-(math.pi/2)))
                    ballstate.model_state.pose.position.z = 0.01
                    ballstate.model_state.pose.orientation = rot3_q
                    ballstate.model_state.reference_frame = "world"
                    set_ball_service(ballstate) # call set_model_state to set ball in front of bot

# Publisher & Subscriber definition
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
sub3 = rospy.Subscriber("/robot_3/odom", Odometry, newOdom3)
sub4 = rospy.Subscriber("/robot_5/odom", Odometry, newOdom5)
pub = rospy.Publisher("/robot_3/cmd_vel", Twist, queue_size = 10)


# Ball Possession Publisher & Subscriber
br1 = False
br2 = False #ball on robot_2
br3 = False #ball on robot_3
br4 = False
br5 = False
br6 = False
bt1 = False #ball on team_1
bt2 = False #ball on team_2

def br2Callback(msg):
    global br2
    if msg.data == 1 :
        br2 = True
    else:
        br2 = False

def br1Callback(msg):
    global br1
    if msg.data == 1:
        br1 = True
    else:
        br1 = False

def br4Callback(msg):
    global br4
    if msg.data == 1:
        br4 = True
    else:
        br4 = False

def br5Callback(msg):
    global br5
    if msg.data == 1:
        br5 = True
    else:
        br5 = False

def br6Callback(msg):
    global br6
    if msg.data == 1:
        br6 = True
    else:
        br6 = False

def checkTeam(br_1, br_2, br_3, br_4, br_5, br_6):
    global bt1 
    global bt2
    if (br_1 or br_2 or br_3):
        bt1 = True
        bt2 = False
    elif (br_4 or br_5 or br_6):
        bt1 = False
        bt2 = True
    else:
        bt1 = False
        bt2 = False

pub2 = rospy.Publisher("/ball_on_robot_3", Int8, queue_size=1)
sub4 = rospy.Subscriber("/ball_on_robot_2", Int8, br2Callback)
sub5 = rospy.Subscriber("/ball_on_robot_1", Int8, br1Callback)
sub6 = rospy.Subscriber("/ball_on_robot_4", Int8, br4Callback)
sub7 = rospy.Subscriber("/ball_on_robot_5", Int8, br5Callback)
sub8 = rospy.Subscriber("/ball_on_robot_6", Int8, br6Callback)

speed = Twist() #global speed variable

r = rospy.Rate(1000) #1000 Hz
while not rospy.is_shutdown():
    if (dribbling):
        pub2.publish(1) # 1 : dribbling published 
    else:
        pub2.publish(0) # 0 : not dribbling published
    
    checkTeam(br1, br2, br3, br4, br5, br6)
    inc_x = goal.x -x3
    inc_y = goal.y -y3
    angle_to_goal = atan2(inc_y, inc_x)
    if (bt2):
        if (angle_to_goal - theta3) > 0.2:
            speed.linear.x = 0.0
            if (angle_to_goal - theta3) > 0.5 :
                speed.angular.z = 2.0
            else:
                speed.angular.z = 0.4
        elif (angle_to_goal - theta3) < -0.2:
            speed.linear.x = 0.0
            if (angle_to_goal - theta3) < -0.5:
                speed.angular.z = -2.0
            else:
                speed.angular.z = -0.4
        else:
            speed.angular.z = 0.0 
            speed.linear.x = 0.6
    else: #bt2 == False
        if (not(isInEnemyPenalty(x3,y3)) and dribbling):
            if (angle_to_goal - theta3) > 0.2:
                speed.linear.x = 0.0
                if (angle_to_goal - theta3) > 0.5 :
                    speed.angular.z = 2.0
                else:
                    speed.angular.z = 0.4
            elif (angle_to_goal - theta3) < -0.2:
                speed.linear.x = 0.0
                if (angle_to_goal - theta3) < -0.5:
                    speed.angular.z = -2.0
                else:
                    speed.angular.z = -0.4
            else:
                speed.angular.z = 0.0 
                speed.linear.x = 0.6
        else:
            if (isInEnemyPenalty(x3,y3)):
                if ((2.90<=abs(theta3)<=2.92) and speed.angular.z != 0):
                    speed.angular.z = 0.0
                    speed.linear.x = 0.0
                elif ((2.90<=abs(theta3)<=2.92) and speed.angular.z == 0):
                    speed.angular.z = 0.0
                    speed.linear.x = 0.0
                elif (y5 > 0):
                    speed.angular.z = -1.0
                    speed.linear.x = 0.0
                elif (y5 < 0):
                    speed.angular.z = 1.0
                    speed.linear.x = 0.0
            else:
                speed.linear.x = 0.0
                speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()