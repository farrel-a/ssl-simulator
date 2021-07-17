#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include <unistd.h>

bool pause_physics;
bool reset;
ros::ServiceClient clientPause;
ros::ServiceClient clientReset;
ros::ServiceClient clientSetModel; //global client variable
gazebo_msgs::SetModelState ballstate;

void check_goal_out(const gazebo_msgs::ModelStates& msg)
{
    std_srvs::Empty callPause;
    std_srvs::Empty callReset;
    std_msgs::Float64 x;
    std_msgs::Float64 y;
    x.data = msg.pose[0].position.x;
    y.data = msg.pose[0].position.y;
    if (x.data<=-6.2 || y.data <=-4.5 || y.data >= 4.5 || x.data >=6.2)  //goal or out enemy side
    {
        gazebo_msgs::SetModelState robotstate1; //robot_1
        gazebo_msgs::SetModelState robotstate3; //robot_3
        gazebo_msgs::SetModelState robotstate4; //robot_4
        gazebo_msgs::SetModelState robotstate6; //robot_6
        ballstate.request.model_state.model_name = "ssl_ball_1";
        ballstate.request.model_state.pose.position.x = 0.0;
        ballstate.request.model_state.pose.position.y = 0.0;
        ballstate.request.model_state.pose.position.z = 0.0;
        ballstate.request.model_state.twist.linear.x = 0.0;
        ballstate.request.model_state.reference_frame = "world";
        if (x.data<=-6.2 || y.data <= -4.5)
        {
            robotstate1.request.model_state.model_name = "robot_1";
            robotstate1.request.model_state.pose.position.x = 2.0;
            robotstate1.request.model_state.pose.position.y = -2.0;
            robotstate1.request.model_state.pose.position.z = 0.0;
            robotstate1.request.model_state.pose.orientation.z = 1.0;
            robotstate1.request.model_state.reference_frame = "world";

            robotstate3.request.model_state.model_name = "robot_3";
            robotstate3.request.model_state.pose.position.x = 2.0;
            robotstate3.request.model_state.pose.position.y = 2.0;
            robotstate3.request.model_state.pose.position.z = 0.0;
            robotstate3.request.model_state.pose.orientation.z = 1.0;
            robotstate3.request.model_state.reference_frame = "world";

            robotstate4.request.model_state.model_name = "robot_4";
            robotstate4.request.model_state.pose.position.x = -0.5;
            robotstate4.request.model_state.pose.position.y = 0.5;
            robotstate4.request.model_state.pose.position.z = 0.0;
            robotstate4.request.model_state.pose.orientation.z = 0.377;
            robotstate4.request.model_state.pose.orientation.w = -0.93;
            robotstate4.request.model_state.reference_frame = "world";

            robotstate6.request.model_state.model_name = "robot_6";
            robotstate6.request.model_state.pose.position.x = -0.5;
            robotstate6.request.model_state.pose.position.y = -0.5;
            robotstate6.request.model_state.pose.position.z = 0.0;
            robotstate6.request.model_state.pose.orientation.z = -0.377;
            robotstate6.request.model_state.pose.orientation.w = -0.93;
            robotstate6.request.model_state.reference_frame = "world";
        }
        else if (x.data>=6.2 || y.data >= 4.5)
        {
            robotstate1.request.model_state.model_name = "robot_1";
            robotstate1.request.model_state.pose.position.x = 0.5;
            robotstate1.request.model_state.pose.position.y = -0.5;
            robotstate1.request.model_state.pose.position.z = 0.0;
            robotstate1.request.model_state.pose.orientation.z = 0.91;
            robotstate1.request.model_state.pose.orientation.w = 0.41;
            robotstate1.request.model_state.reference_frame = "world";

            robotstate3.request.model_state.model_name = "robot_3";
            robotstate3.request.model_state.pose.position.x = 0.5;
            robotstate3.request.model_state.pose.position.y = 0.5;
            robotstate3.request.model_state.pose.position.z = 0.0;
            robotstate3.request.model_state.pose.orientation.z = -0.91;
            robotstate3.request.model_state.pose.orientation.w = 0.41;
            robotstate3.request.model_state.reference_frame = "world";

            robotstate4.request.model_state.model_name = "robot_4";
            robotstate4.request.model_state.pose.position.x = -2.0;
            robotstate4.request.model_state.pose.position.y = 2.0;
            robotstate4.request.model_state.pose.position.z = 0.0;
            robotstate4.request.model_state.pose.orientation.z = 0.0;
            robotstate4.request.model_state.reference_frame = "world";

            robotstate6.request.model_state.model_name = "robot_6";
            robotstate6.request.model_state.pose.position.x = -2.0;
            robotstate6.request.model_state.pose.position.y = -2.0;
            robotstate6.request.model_state.pose.position.z = 0.0;
            robotstate6.request.model_state.pose.orientation.z = 0.0;
            robotstate6.request.model_state.reference_frame = "world";
        }

        if (!pause_physics && !reset)
        {
            //reboot the node to refresh the node 
            system("bash -c 'source /opt/ros/noetic/setup.bash; rosnode kill /robot_1.py /robot_2.py /robot_3.py /robot_4.py /robot_5.py /robot_6.py'"); 
            sleep(3);
            // clientPause.call(callPause);
            // sleep(2);    
            pause_physics = true;
        }
        if (pause_physics && !reset)
        {
            clientReset.call(callReset);
            system("bash -c 'source /opt/ros/noetic/setup.bash; rosnode kill /robot_1.py /robot_2.py /robot_3.py /robot_4.py /robot_5.py /robot_6.py'"); 
            sleep(3);
            clientSetModel.call(robotstate1);
            clientSetModel.call(robotstate3);
            clientSetModel.call(robotstate4);
            clientSetModel.call(robotstate6);
            clientSetModel.call(ballstate);
            sleep(1);
            clientSetModel.call(ballstate);
            clientSetModel.call(ballstate);
            sleep(1);
            clientSetModel.call(ballstate);
            reset = true;
        }
    }

    if (pause_physics && reset)
    {
        sleep(1);
        clientSetModel.call(ballstate);
        sleep(0.2);
        clientSetModel.call(ballstate);
        sleep(0.2);
        clientSetModel.call(ballstate);
        sleep(0.2);
        clientSetModel.call(ballstate);
        sleep(0.2);
        clientSetModel.call(ballstate);
        pause_physics = false;
        reset = false;
    }
}

int main (int argc, char **argv)
{
    pause_physics = false;
    reset = false;
    ros::init(argc, argv, "goalout_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/ball_state",100, check_goal_out);
    clientPause = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    clientReset = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    clientSetModel = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::spin();
    return 0;
}