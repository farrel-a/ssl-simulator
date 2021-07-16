#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include <unistd.h>

bool pause_physics;
bool reset;
ros::ServiceClient clientPause;
ros::ServiceClient clientReset;
ros::Publisher pub;

void check_goal_out(const gazebo_msgs::ModelStates& msg)
{
    std_srvs::Empty callPause;
    std_srvs::Empty callReset;
    std_msgs::Float64 x;
    std_msgs::Float64 y;
    std_msgs::Int64 n;
    x.data = msg.pose[0].position.x;
    y.data = msg.pose[0].position.y;
    if (x.data<=-6.2 || y.data <=-4.5 || y.data >= 4.5 || x.data >=6.2)  //goal or out enemy side
    {
        if (x.data<=-6.2)
        {
            n.data = 0;
        }
        if (x.data>=6.2)
        {
            n.data = 1;
        }
        if (!pause_physics && !reset)
        {
            system("bash -c 'source /opt/ros/noetic/setup.bash; rosnode kill /robot_1.py /robot_3.py'"); //reboot the node to refresh the node
            clientPause.call(callPause);
            sleep(2);    
            pause_physics = true;
        }
        if (pause_physics && !reset)
        {
            clientReset.call(callReset);
            sleep(1); 
            pub.publish(n);
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
    pub = nh.advertise<std_msgs::Int64>("kickoff", 100);
    clientPause = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    clientReset = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    ros::spin();
    return 0;
}