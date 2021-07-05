#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;
int get_index(std::vector<std::string> arr, std::string val)
{
    int i;
    for (i=0 ; i < arr.size() ; i++)
    {
        if (arr[i]==val) {return i;}
    }
    return -1;
}

void pubstate(const gazebo_msgs::ModelStates& model)
{
    int ball_index = get_index(model.name, "ssl_ball_1");
    gazebo_msgs::ModelStates ball;
    ball.name.push_back(model.name[ball_index]);
    ball.pose.push_back(model.pose[ball_index]);
    ball.twist.push_back(model.twist[ball_index]);
    pub.publish(ball);
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "ball_state_pub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/gazebo/model_states",100, pubstate);
    pub = nh.advertise<gazebo_msgs::ModelStates>("ball_state", 100);
    ros::spin();
    return 0;
}