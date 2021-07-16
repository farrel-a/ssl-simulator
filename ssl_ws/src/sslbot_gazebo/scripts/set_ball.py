#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int64
from gazebo_msgs.srv import *
from std_srvs.srv import *
import time

rospy.init_node("set_ball")
rospy.wait_for_service('/gazebo/set_model_state')
set_ball_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
ballstate = SetModelStateRequest() #global ballstate request varible

run = False
def callback(msg):
    global run
    global ballstate
    if ((msg.data == 1) and not(run)): 
            ballstate.model_state.model_name = "ssl_ball_1"
            ballstate.model_state.pose.position.x = 1.86
            ballstate.model_state.pose.position.y = -2.0
            ballstate.model_state.pose.position.z = 0.05
            ballstate.model_state.twist.linear.x = 0.0
            ballstate.model_state.reference_frame = "world"
            set_ball_service(ballstate)
            run = True

    elif ((msg.data == 0) and not(run)): #passing == False
        ballstate.model_state.model_name = "ssl_ball_1"
        ballstate.model_state.pose.position.x = -1.86
        ballstate.model_state.pose.position.y = 2.0
        ballstate.model_state.pose.position.z = 0.05
        ballstate.model_state.twist.linear.x = 0.0
        ballstate.model_state.reference_frame = "world"
        set_ball_service(ballstate) #call set_model_state to be in front of bot
        run = True
    
    time.sleep(5)
    run = False
def listener():

    rospy.Subscriber("/kickoff", Int64, callback)

    rospy.spin()

# Publisher & Subscriber definition

if __name__ == '__main__':
    listener()