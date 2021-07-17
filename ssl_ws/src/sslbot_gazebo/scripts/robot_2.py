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

def isDribbling (d):
    # d : distance
    if (d < 0.15):
        return True
    else:
        return False

rospy.init_node("robot_2gk") #gk stands for goalkeeper
rospy.wait_for_service('/gazebo/set_model_state')
set_ball_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

ballstate = SetModelStateRequest() #global ballstate request varible
goal = Point() #global goal variable
visited = False
dribbling = False #global driblling variable
passing = False #global passing condition (True : should pass, False : should not pass)
distance = 0.0
x_ball = 0.0
y_ball = 0.0
moving = True
def ballPos(msg):
    global goal
    global ballstate
    global dribbling
    global visited
    global distance
    global passing
    global x_ball
    global y_ball
    global moving
    x_ball = msg.pose[0].position.x
    y_ball = msg.pose[0].position.y
    distance = ((x_ball-x2)**2 + (y_ball-y2)**2)**0.5
    dribbling = isDribbling(distance)
    if (dribbling):
        if (moving and not passing):
            passing = False
            ballstate.model_state.model_name = "ssl_ball_1"
            ballstate.model_state.pose.position.x = x2 + (0.11*math.sin((math.pi-(theta2))-(math.pi/2)))
            ballstate.model_state.pose.position.y = y2 + (0.11*math.cos((math.pi-(theta2))-(math.pi/2)))
            ballstate.model_state.pose.position.z = 0.01
            ballstate.model_state.pose.orientation = rot2_q
            ballstate.model_state.reference_frame = "world"
            set_ball_service(ballstate)
        else: # moving == False
            if ((abs(angle_to_friend1-theta2) < 0.01) and not(passing)):
                ballstate.model_state.model_name = "ssl_ball_1"
                ballstate.model_state.pose.position.x = 0.0
                ballstate.model_state.pose.position.y = 0.0
                ballstate.model_state.pose.position.z = 0.0
                ballstate.model_state.twist.linear.x = 2.0
                ballstate.model_state.reference_frame = "ssl_ball_1"
                moving = True
                dribbling = False
                passing = True
                set_ball_service(ballstate)
                time.sleep(3)
                
            elif (not(passing)): # planar_speed.angular.z != 0
                ballstate.model_state.model_name = "ssl_ball_1"
                ballstate.model_state.pose.position.x = x2 + (0.11*math.sin((math.pi-(theta2))-(math.pi/2)))
                ballstate.model_state.pose.position.y = y2 + (0.11*math.cos((math.pi-(theta2))-(math.pi/2)))
                ballstate.model_state.pose.position.z = 0.01
                ballstate.model_state.pose.orientation = rot2_q
                ballstate.model_state.reference_frame = "world"
                set_ball_service(ballstate)
    else:
        dribbling = False
        passing = False

# Publisher & Subscriber definition
sub = rospy.Subscriber("/robot_2/odom", Odometry, newOdom2)
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
sub3 = rospy.Subscriber("/robot_1/odom", Odometry, newOdom1)
pub2 = rospy.Publisher("/robot_2/planar_vel", Twist, queue_size = 10)
pub3 = rospy.Publisher("/ball_on_robot_2", Int8, queue_size=1)

speed = Twist() #global speed variable
planar_speed = Twist()
correction = False #GK orientation correction

r = rospy.Rate(1000) #1000 Hz
while not rospy.is_shutdown():
    if dribbling:
        pub3.publish(1) # 1 : dribbling published
    else:
        pub3.publish(0) # 0 : not dribbling published

    inc_x = goal.x -x2
    inc_y = goal.y -y2
    inc_x_friend1 = x1 - x2
    inc_y_friend1 = y1 - y2
    angle_to_goal = atan2(inc_y, inc_x)
    angle_to_friend1 = atan2(inc_y_friend1, inc_x_friend1)
    if correction:
        if (not(3.12 <= abs(theta2) <= 3.14)):
            planar_speed.angular.z = 1.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
        else:
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
            correction = False 
    
    elif (not(dribbling) and not(correction)):
        if ((y_ball - 0.01) <= y2 <= (y_ball + 0.01)):
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
        elif (y_ball < y2):
            if (y2 >= -1.8):
                planar_speed.angular.z = 0.0
                planar_speed.linear.x = 0.0
                planar_speed.linear.y = 0.5
            else:
                planar_speed.angular.z = 0.0
                planar_speed.linear.x = 0.0
                planar_speed.linear.y = 0.0
        elif (y2 < y_ball):
            if (y2 <= 1.8):
                planar_speed.angular.z = 0.0
                planar_speed.linear.x = 0.0
                planar_speed.linear.y = -0.5
            else:
                planar_speed.angular.z = 0.0
                planar_speed.linear.x = 0.0
                planar_speed.linear.y = 0.0
    elif (dribbling and moving and not(correction)):
        if (-0.01 <= y2 <= 0.01):
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
            moving = False
        elif (y2 > 0.01):
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.5
        elif (y2 < -0.01):
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = -0.5
    elif (dribbling and not(moving) and not(correction)):
        if (angle_to_friend1 - theta2) > 0.01:
            planar_speed.angular.z = 0.3
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
            speed.linear.y = 0.0
        elif (angle_to_friend1 - theta2) < -0.01:
            planar_speed.angular.z = -0.3
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
        else :
            correction = True
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
        
    pub2.publish(planar_speed)
    r.sleep()
