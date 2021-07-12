#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float64.h"
#include <unistd.h>

bool pause_physics;
bool reset;
ros::ServiceClient clientPause;
ros::ServiceClient clientReset;

void check_goal_out(const gazebo_msgs::ModelStates& msg)
{
    std_srvs::Empty callPause;
    std_srvs::Empty callReset;
    std_msgs::Float64 x;
    std_msgs::Float64 y;
    x.data = msg.pose[0].position.x;
    y.data = msg.pose[0].position.y;
    if (x.data<=-6.2)  //goal or out enemy side
    {
        if (!pause_physics && !reset)
        {
            clientPause.call(callPause);
            sleep(2);    
            pause_physics = true;
        }
        if (pause_physics && !reset)
        {
            clientReset.call(callReset);
            sleep(1);
            reset = true;
        }
    }

    if (pause_physics && reset)
    {
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
    ros::spin();
    return 0;
}