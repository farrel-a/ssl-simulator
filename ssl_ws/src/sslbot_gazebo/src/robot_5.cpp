#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/Int8.h"
#include <math.h>
#include <bits/stdc++.h>
#include <unistd.h>

double PI = 3.14159;

std_msgs::Int8 one;
std_msgs::Int8 zero;

double x5 = 0.0;
double y5 = 0.0;
double theta5 = 0.0;
geometry_msgs::Quaternion rot5_q;

void newOdom5(const nav_msgs::Odometry& msg)
{
    x5 = msg.pose.pose.position.x;
    y5 = msg.pose.pose.position.y;
    rot5_q = msg.pose.pose.orientation;

    tf::Quaternion q(rot5_q.x, rot5_q.y, rot5_q.z, rot5_q.w);
    tf::Matrix3x3 m(q);

    double R,P,Y;

    m.getRPY(R,P,Y);

    theta5 = Y;
}

double x4 = 0.0;
double y4 = 0.0;
double theta4 = 0.0;
geometry_msgs::Quaternion rot4_q;

void newOdom4(const nav_msgs::Odometry& msg)
{
    x4 = msg.pose.pose.position.x;
    y4 = msg.pose.pose.position.y;
    rot4_q = msg.pose.pose.orientation;

    tf::Quaternion q(rot4_q.x, rot4_q.y, rot4_q.z, rot4_q.w);
    tf::Matrix3x3 m(q);

    double R,P,Y;

    m.getRPY(R,P,Y);

    theta4 = Y;
}

bool isDribbling(double d)
{
    //d : distance
    if (d<0.15){return true;}
    else{return false;}
}

ros::ServiceClient set_ball_service; //init in int main
gazebo_msgs::SetModelState ballstate;
geometry_msgs::Point goal;
bool visited = false;
bool dribbling = false;
bool passing = false;
bool moving = true;
bool correction = false;
double x_ball = 0.0;
double y_ball = 0.0;
double distance = 0.0;
double angle_to_goal;
double angle_to_friend4;
geometry_msgs::Twist planar_speed;

void ballPos(const gazebo_msgs::ModelStates& msg)
{
    x_ball = msg.pose[0].position.x;
    y_ball = msg.pose[0].position.y;
    distance = sqrt(pow((x_ball-x5),2.0) + pow((y_ball-y5),2.0));
    dribbling = isDribbling(distance);

    if (dribbling)
    {
        if (moving && !passing)
        {
            passing = false;
            ballstate.request.model_state.model_name = "ssl_ball_1";
            ballstate.request.model_state.pose.position.x = x5 + (0.11*sin(((PI/2)-(theta5))));
            ballstate.request.model_state.pose.position.y = y5 + (0.11*cos(((PI/2)-(theta5))));
            ballstate.request.model_state.pose.position.z = 0.01;
            ballstate.request.model_state.pose.orientation = rot5_q;
            ballstate.request.model_state.reference_frame = "world";
            set_ball_service.call(ballstate);
        }

        else // !moving
        {
            if ((abs(angle_to_friend4-theta5)<0.01) && !passing)
            {
                ballstate.request.model_state.model_name = "ssl_ball_1";
                ballstate.request.model_state.pose.position.x = 0.0;
                ballstate.request.model_state.pose.position.y = 0.0;
                ballstate.request.model_state.pose.position.z = 0.0;
                ballstate.request.model_state.twist.linear.x = 2.0;
                ballstate.request.model_state.reference_frame = "ssl_ball_1";
                moving = true;
                dribbling = false;
                passing = true;
                set_ball_service.call(ballstate);
                sleep(3);
            }

            else if (!passing)
            {
                ballstate.request.model_state.model_name = "ssl_ball_1";
                ballstate.request.model_state.pose.position.x = x5 + (0.11*sin(((PI/2)-(theta5))));
                ballstate.request.model_state.pose.position.y = y5 + (0.11*cos(((PI/2)-(theta5))));
                ballstate.request.model_state.pose.position.z = 0.01;
                ballstate.request.model_state.pose.orientation = rot5_q;
                ballstate.request.model_state.reference_frame = "world";
                set_ball_service.call(ballstate);
            }
        }
    }
    else
    {
        dribbling = false;
        passing = false;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_5gk");
    ros::NodeHandle nh;
    
    // Publisher & Subscriber Definition
    ros::Subscriber sub = nh.subscribe("/robot_5/odom",100, newOdom5);
    ros::Subscriber sub2 = nh.subscribe("/ball_state",1, ballPos);
    ros::Subscriber sub3 = nh.subscribe("/robot_4/odom",100, newOdom4);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/robot_5/planar_vel", 10);
    ros::Publisher pub3 = nh.advertise<std_msgs::Int8>("/ball_on_robot_5", 1);

    set_ball_service = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::Rate loop_rate(400); //400 Hz

    one.data = 1;
    zero.data = 0;

    double inc_x, inc_y, inc_x_friend4, inc_y_friend4;

    while (ros::ok())
    {
        if (dribbling){pub3.publish(one);}
        else{pub3.publish(zero);}
        inc_x = goal.x -x5;
        inc_y = goal.y -y5;
        inc_x_friend4 = x4 - x5;
        inc_y_friend4 = y4 - y5;
        angle_to_goal = atan2(inc_y, inc_x);
        angle_to_friend4 = atan2(inc_y_friend4, inc_x_friend4);

        if (correction)
        {
            if (!(0.00 <=abs(theta5) && abs(theta5)<=0.02))
            {
                planar_speed.angular.z = 1.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
            }
            else
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
                correction = false; 
            }
        }

        else if (!dribbling && !correction)
        {
            if ((y_ball - 0.01 <= y5) && (y5 <= y_ball + 0.01))
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
            }
            else if (y_ball < y5)
            {
                if (y5 >= -1.8)
                {
                    planar_speed.angular.z = 0.0;
                    planar_speed.linear.x = 0.0;
                    planar_speed.linear.y = -0.5;
                }
                else
                {
                    planar_speed.angular.z = 0.0;
                    planar_speed.linear.x = 0.0;
                    planar_speed.linear.y = 0.0;
                }
            }

            else if (y5 < y_ball)
            {
                if (y5 <= 1.8)
                {
                    planar_speed.angular.z = 0.0;
                    planar_speed.linear.x = 0.0;
                    planar_speed.linear.y = 0.5;
                }
                else
                {
                    planar_speed.angular.z = 0.0;
                    planar_speed.linear.x = 0.0;
                    planar_speed.linear.y = 0.0;
                }
            }
        }

        else if (dribbling && moving && !correction)
        {
            if (-0.01 <= y5 && y5<=0.01)
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
                moving = false;
            }

            else if (y5 > 0.01)
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = -0.5;
            }

            else if (y5 < -0.01)
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.5;
            }
        }

        else if (dribbling && !moving && !correction)
        {
            if (angle_to_friend4 - theta5 > 0.01)
            {
                planar_speed.angular.z = 0.3;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
            }

            else if (angle_to_friend4 - theta5 < -0.01)
            {
                planar_speed.angular.z = -0.3;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
            }

            else
            {
                correction = true;
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
            }
        }

        pub2.publish(planar_speed);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}