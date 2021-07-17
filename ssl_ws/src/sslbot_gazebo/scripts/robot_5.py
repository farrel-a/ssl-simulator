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

def isDribbling (d):
    # d : distance
    if (d < 0.15):
        return True
    else:
        return False

rospy.init_node("robot_5gk") #k stands for goalkeeper
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
    distance = ((x_ball-x5)**2 + (y_ball-y5)**2)**0.5
    dribbling = isDribbling(distance)
    if (dribbling):
        if (moving and not passing):
            passing = False
            ballstate.model_state.model_name = "ssl_ball_1"
            ballstate.model_state.pose.position.x = x5 + (0.11*math.sin((math.pi-(theta5))-(math.pi/2)))
            ballstate.model_state.pose.position.y = y5 + (0.11*math.cos((math.pi-(theta5))-(math.pi/2)))
            ballstate.model_state.pose.position.z = 0.01
            ballstate.model_state.pose.orientation = rot5_q
            ballstate.model_state.reference_frame = "world"
            set_ball_service(ballstate)
        else: # moving == False
            if ((abs(angle_to_friend4-theta5) < 0.01) and not(passing)):
                ballstate.model_state.model_name = "ssl_ball_1"
                ballstate.model_state.pose.position.x = 0.0
                ballstate.model_state.pose.position.y = 0.0
                ballstate.model_state.pose.position.z = 0.0
                ballstate.model_state.twist.linear.x = 2.0
                ballstate.model_state.reference_frame = "ssl_ball_1"
                moving = True
                dribbling = False
                set_ball_service(ballstate)
                time.sleep(3)
                passing = True
                
            elif (not(passing)): # planar_speed.angular.z != 0
                ballstate.model_state.model_name = "ssl_ball_1"
                ballstate.model_state.pose.position.x = x5 + (0.11*math.sin((math.pi-(theta5))-(math.pi/2)))
                ballstate.model_state.pose.position.y = y5 + (0.11*math.cos((math.pi-(theta5))-(math.pi/2)))
                ballstate.model_state.pose.position.z = 0.01
                ballstate.model_state.pose.orientation = rot5_q
                ballstate.model_state.reference_frame = "world"
                set_ball_service(ballstate)
    else:
        dribbling = False
        passing = False

# Publisher & Subscriber definition
sub = rospy.Subscriber("/robot_5/odom", Odometry, newOdom5)
sub2 = rospy.Subscriber("/ball_state", ModelStates, ballPos)
sub3 = rospy.Subscriber("/robot_4/odom", Odometry, newOdom4)
pub2 = rospy.Publisher("/robot_5/planar_vel", Twist, queue_size = 10)
pub3 = rospy.Publisher("/ball_on_robot_5", Int8, queue_size = 1)

speed = Twist() #global speed variable
planar_speed = Twist()
correction = False

r = rospy.Rate(1000) #1000 Hz
while not rospy.is_shutdown():
    if (dribbling):
        pub3.publish(1)
    else:
        pub3.publish(0)

    inc_x = goal.x -x5
    inc_y = goal.y -y5
    inc_x_friend4 = x4 - x5
    inc_y_friend4 = y4 - y5
    angle_to_goal = atan2(inc_y, inc_x)
    angle_to_friend4 = atan2(inc_y_friend4, inc_x_friend4)

    if correction:
        if (not(0.00 <= abs(theta5) <= 0.02)):
            planar_speed.angular.z = 1.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
        else:
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
            correction = False

    elif (not(dribbling) and not correction):
        if ((y_ball - 0.01) <= y5 <= (y_ball + 0.01)):
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
        elif (y_ball < y5):
            if (y5 >= -1.8):
                planar_speed.angular.z = 0.0
                planar_speed.linear.x = 0.0
                planar_speed.linear.y = -0.5
            else:
                planar_speed.angular.z = 0.0
                planar_speed.linear.x = 0.0
                planar_speed.linear.y = 0.0
        elif (y5 < y_ball):
            if (y5 <= 1.8):
                planar_speed.angular.z = 0.0
                planar_speed.linear.x = 0.0
                planar_speed.linear.y = 0.5
            else:
                planar_speed.angular.z = 0.0
                planar_speed.linear.x = 0.0
                planar_speed.linear.y = 0.0
    elif (dribbling and moving and not(correction)):
        if (-0.01 <= y5 <= 0.01):
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
            moving = False
        elif (y5 > 0.01):
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = -0.5
        elif (y5 < -0.01):
            planar_speed.angular.z = 0.0
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.5
    elif (dribbling and not(moving) and not(correction)):
        if (angle_to_friend4 - theta5) > 0.01:
            planar_speed.angular.z = 0.3
            planar_speed.linear.x = 0.0
            planar_speed.linear.y = 0.0
            speed.linear.y = 0.0
        elif (angle_to_friend4 - theta5) < -0.01:
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
