#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from rospy.impl.transport import INBOUND
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
x4 = 0.0
y4 = 0.0 
theta4 = 0.0
rot4_q = Quaternion()

def newOdom4(msg):
    global x4
    global y4
    global theta4
    global rot4_q

    x4 = msg.pose.pose.position.x
    y4 = msg.pose.pose.position.y

    rot4_q = msg.pose.pose.orientation
    (roll, pitch, theta4) = euler_from_quaternion([rot4_q.x, rot4_q.y, rot4_q.z, rot4_q.w])

x6 = 0.0
y6 = 0.0
theta6 = 0.0
rot6_q = Quaternion()

def newOdom6(msg):
    global x6
    global y6
    global theta6
    global rot6_q
    x6 = msg.pose.pose.position.x
    y6 = msg.pose.pose.position.y

    rot6_q = msg.pose.pose.orientation
    (roll, pitch, theta6) = euler_from_quaternion([rot6_q.x, rot6_q.y, rot6_q.z, rot6_q.w])

def isInEnemyPenalty(a,b):
    if ((7.0<=a<=4.0) and (-0.5<=b<=0.5)):
        return True
    else:
        False

def isDribbling (d):
    # d : distance
    if (d < 0.15):
        return True
    else:
        return False

rospy.init_node("robot_4")
rospy.wait_for_service('/gazebo/set_model_state')
set_ball_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

ballstate = SetModelStateRequest() #global ballstate request varible
goal = Point() #global goal variable
x_ball = 0.0
y_ball = 0.0
dribbling = False #global driblling variable
passing = False #global passing condition (True : should pass, False : should not pass)
distance = 0.0

def ballPos(msg):
    global goal
    global ballstate
    global dribbling
    global distance
    global passing
    global x_ball
    global y_ball
    x_ball = msg.pose[0].position.x
    y_ball = msg.pose[0].position.y
    distance = ((x_ball-x4)**2 + (y_ball-y4)**2)**0.5
    dribbling = isDribbling(distance)
    if (bt1): #bt1 == True (ball on team 1)
        passing = False
        goal.x = -5.2
        goal.y = 1.2
    elif(not(bt1)): #bt1 == False
        if (dribbling): #dribbling == True
            if (speed.linear.x==0 and speed.angular.z==0 and (abs(angle_to_friend-theta4) < 0.01) and not(passing)):
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
                ballstate.model_state.pose.position.x = x4 + (0.11*math.sin((math.pi-(theta4))-(math.pi/2)))
                ballstate.model_state.pose.position.y = y4 + (0.11*math.cos((math.pi-(theta4))-(math.pi/2)))
                ballstate.model_state.pose.position.z = 0.01
                ballstate.model_state.pose.orientation = rot4_q
                ballstate.model_state.reference_frame = "world"
                set_ball_service(ballstate) #call set_model_state to be in front of bot
        elif (passing):
            goal.x = 1.5
            goal.y = 1.5
        else: #not dribbilng and not passing
            dribbling = False
            goal.x = msg.pose[0].position.x # x values of ball
            goal.y = msg.pose[0].position.y # y values of ball

# Publisher & Subscriber Definition
sub = rospy.Subscriber("/robot_4/odom", Odometry, newOdom4)
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
sub3 = rospy.Subscriber("/robot_6/odom", Odometry, newOdom6)
pub = rospy.Publisher("/robot_4/cmd_vel", Twist, queue_size = 10)


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

def br3Callback(msg):
    global br3
    if msg.data == 1:
        br3 = True
    else:
        br3 = False

def br1Callback(msg):
    global br1
    if msg.data == 1:
        br1 = True
    else:
        br1 = False

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

pub2 = rospy.Publisher("/ball_on_robot_4", Int8, queue_size=1)
sub4 = rospy.Subscriber("/ball_on_robot_2", Int8, br2Callback)
sub5 = rospy.Subscriber("/ball_on_robot_3", Int8, br3Callback)
sub6 = rospy.Subscriber("/ball_on_robot_1", Int8, br1Callback)
sub7 = rospy.Subscriber("/ball_on_robot_5", Int8, br5Callback)
sub8 = rospy.Subscriber("/ball_on_robot_6", Int8, br6Callback)

speed = Twist() #global speed variable

r = rospy.Rate(1000) #1000 Hz
while not rospy.is_shutdown():
    if (dribbling):
        br4 = True
        pub2.publish(1) # 1 : dribbling published
    else:
        br4 = False
        pub2.publish(0) # 0 : not dribbling published
    
    checkTeam(br1, br2, br3, br4, br5, br6)

    inc_x = goal.x -x4
    inc_y = goal.y -y4
    inc_x_friend = x6 - x4
    inc_y_friend = y6 - y4
    angle_to_goal = atan2(inc_y, inc_x)
    angle_to_friend = atan2(inc_y_friend, inc_x_friend)
    if (br5):
        speed.linear.x = 0.0
        speed.angular.z = 0.0
    else: #br5 == False
        if (not(dribbling)): 
            if (angle_to_goal - theta4) > 0.2:
                speed.linear.x = 0.0
                if (angle_to_goal - theta4) > 0.5 :
                    speed.angular.z = 2.0
                else:
                    speed.angular.z = 0.4
            elif (angle_to_goal - theta4) < -0.2:
                speed.linear.x = 0.0
                if (angle_to_goal - theta4) < -0.5:
                    speed.angular.z = -2.0
                else:
                    speed.angular.z = -0.4
            else:
                speed.angular.z = 0.0 
                speed.linear.x = 0.6
        elif (dribbling and not passing):
            if (angle_to_friend - theta4) > 0.01:
                speed.angular.z = 0.5
                speed.linear.x = 0.0
            elif (angle_to_friend - theta4) < -0.01:
                speed.angular.z = -0.5
                speed.linear.x = 0.0
            else :
                speed.angular.z = 0.0
                speed.linear.x = 0.0
                dribbling = False

    pub.publish(speed)
    r.sleep()