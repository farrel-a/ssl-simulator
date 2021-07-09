#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/Float64.h"
#include <unistd.h>
#include <math.h>

#define PI 3.14159265

ros::ServiceClient client;

int get_index(std::vector<std::string> arr, std::string val)
{
    int i;
    for (i=0 ; i < arr.size() ; i++)
    {
        if (arr[i]==val) {return i;}
    }
    return -1;
}

bool isRotating (geometry_msgs::Twist a)
{
    if (-0.001<=a.angular.z && a.angular.z<=0.001){return false;}
    else{return true;}
}

int relative_quadrant(std_msgs::Float64 Dx, std_msgs::Float64 Dy)
{
    if (Dx.data<0 && Dy.data>0){return 1;}
    else if (Dx.data>0 && Dy.data>0){return 2;}
    else if (Dx.data>0 && Dy.data<0){return 3;}
    else if (Dx.data<0 && Dy.data<0){return 4;}
}


void movebot(const gazebo_msgs::ModelStates& model)
{
    int ball_index = get_index(model.name, "ssl_ball_1");
    int turtlebot_1_index = get_index(model.name, "turtlebot_1");
    std_msgs::Float64 dx;
    dx.data = (model.pose[ball_index].position.x)-(model.pose[turtlebot_1_index].position.x);
    std_msgs::Float64 dy;
    dy.data = (model.pose[ball_index].position.y)-(model.pose[turtlebot_1_index].position.y);
    int rltv_quadrant = relative_quadrant(dx, dy);
    double Dx = 1.0 * dx.data;
    double Dy = 1.0 * dy.data;
    double target; double theta;
    if (rltv_quadrant == 1)
    {
        theta = atan(Dx/Dy);
        target = 1 + ((theta/(PI/2))*0.15);
    }
    gazebo_msgs::SetModelState objstate;
    ROS_INFO("%f", model.pose[turtlebot_1_index].orientation.z);
    if (!((target-0.01)<=model.pose[turtlebot_1_index].orientation.z<=(target+0.01)))
    {
        if (!isRotating(model.twist[turtlebot_1_index]))
        {
            ROS_INFO("Rotate");
            objstate.request.model_state.model_name = "turtlebot_1";
            objstate.request.model_state.pose.position.x = model.pose[turtlebot_1_index].position.x;
            objstate.request.model_state.pose.position.y = model.pose[turtlebot_1_index].position.y;
            objstate.request.model_state.pose.position.z = model.pose[turtlebot_1_index].position.z;
            objstate.request.model_state.pose.orientation.x = model.pose[turtlebot_1_index].orientation.x;
            objstate.request.model_state.pose.orientation.y = model.pose[turtlebot_1_index].orientation.y;
            objstate.request.model_state.pose.orientation.z = model.pose[turtlebot_1_index].orientation.z;
            objstate.request.model_state.pose.orientation.w = model.pose[turtlebot_1_index].orientation.w;
            objstate.request.model_state.twist.linear.x = 0.0;
            objstate.request.model_state.twist.linear.y = 0.0;
            objstate.request.model_state.twist.linear.z = 0.0;
            objstate.request.model_state.twist.angular.x = 0.0;
            objstate.request.model_state.twist.angular.y = 0.0;
            objstate.request.model_state.twist.angular.z = -0.5;
            objstate.request.model_state.reference_frame = "world";
    
            client.call(objstate);
            sleep(1);
        }

    }
    else 
    {
        // if (model.twist[turtlebot_1_index].angular.z!=0)
        // {
        //     ROS_INFO("Stop");
        //     objstate.request.model_state.model_name = "turtlebot_1";
        //     objstate.request.model_state.pose.position.x = model.pose[turtlebot_1_index].position.x;
        //     objstate.request.model_state.pose.position.y = model.pose[turtlebot_1_index].position.y;
        //     objstate.request.model_state.pose.position.z = model.pose[turtlebot_1_index].position.z;
        //     objstate.request.model_state.pose.orientation.x = model.pose[turtlebot_1_index].orientation.x;
        //     objstate.request.model_state.pose.orientation.y = model.pose[turtlebot_1_index].orientation.y;
        //     objstate.request.model_state.pose.orientation.z = model.pose[turtlebot_1_index].orientation.z;
        //     objstate.request.model_state.pose.orientation.w = model.pose[turtlebot_1_index].orientation.w;
        //     objstate.request.model_state.twist.linear.x = 0.0;
        //     objstate.request.model_state.twist.linear.y = 0.0;
        //     objstate.request.model_state.twist.linear.z = 0.0;
        //     objstate.request.model_state.twist.angular.x = 0.0;
        //     objstate.request.model_state.twist.angular.y = 0.0;
        //     objstate.request.model_state.twist.angular.z = 0;
        //     objstate.request.model_state.reference_frame = "world";
        //     client.call(objstate);
        // }
        // else
        // {
            
            ROS_INFO("Forward");
            objstate.request.model_state.model_name = "turtlebot_1";
            objstate.request.model_state.pose.position.x = model.pose[turtlebot_1_index].position.x;
            objstate.request.model_state.pose.position.y = model.pose[turtlebot_1_index].position.y;
            objstate.request.model_state.pose.position.z = model.pose[turtlebot_1_index].position.z;
            objstate.request.model_state.pose.orientation.x = model.pose[turtlebot_1_index].orientation.x;
            objstate.request.model_state.pose.orientation.y = model.pose[turtlebot_1_index].orientation.y;
            objstate.request.model_state.pose.orientation.z = model.pose[turtlebot_1_index].orientation.z;
            objstate.request.model_state.pose.orientation.w = model.pose[turtlebot_1_index].orientation.w;
            objstate.request.model_state.twist.linear.x = -1.0;
            objstate.request.model_state.twist.linear.y = 0.0;
            objstate.request.model_state.twist.linear.z = 0.0;
            objstate.request.model_state.twist.angular.x = 0.0;
            objstate.request.model_state.twist.angular.y = 0.0;
            objstate.request.model_state.twist.angular.z = 0;
            objstate.request.model_state.reference_frame = "world";
            client.call(objstate);
            sleep(1);
    //     }
     }

    
    
    
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "turtlebot_1_pub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/gazebo/model_states",100, movebot);
    client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::spin();
    return 0;
}