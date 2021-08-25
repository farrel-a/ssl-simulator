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

double x2 = 0.0;
double y2 = 0.0;
double theta2 = 0.0;
geometry_msgs::Quaternion rot2_q;

void newOdom2(const nav_msgs::Odometry& msg)
{
    x2 = msg.pose.pose.position.x;
    y2 = msg.pose.pose.position.y;
    rot2_q = msg.pose.pose.orientation;

    tf::Quaternion q(rot2_q.x, rot2_q.y, rot2_q.z, rot2_q.w);
    tf::Matrix3x3 m(q);

    double R,P,Y;

    m.getRPY(R,P,Y);

    theta2 = Y;
}

double x1r = 0.0;
double y1r = 0.0;
double theta1 = 0.0;
geometry_msgs::Quaternion rot1_q;

void newOdom1(const nav_msgs::Odometry& msg)
{
    x1r = msg.pose.pose.position.x;
    y1r = msg.pose.pose.position.y;
    rot1_q = msg.pose.pose.orientation;

    tf::Quaternion q(rot1_q.x, rot1_q.y, rot1_q.z, rot1_q.w);
    tf::Matrix3x3 m(q);

    double R,P,Y;

    m.getRPY(R,P,Y);

    theta1 = Y;
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
double angle_to_friend1;
geometry_msgs::Twist planar_speed;

void ballPos(const gazebo_msgs::ModelStates& msg)
{
    x_ball = msg.pose[0].position.x;
    y_ball = msg.pose[0].position.y;
    distance = sqrt(pow((x_ball-x2),2.0) + pow((y_ball-y2),2.0));
    dribbling = isDribbling(distance);

    if (dribbling)
    {
        if (moving && !passing)
        {
            passing = false;
            ballstate.request.model_state.model_name = "ssl_ball_1";
            ballstate.request.model_state.pose.position.x = x2 + (0.11*sin(((PI/2)-(theta2))));
            ballstate.request.model_state.pose.position.y = y2 + (0.11*cos(((PI/2)-(theta2))));
            ballstate.request.model_state.pose.position.z = 0.01;
            ballstate.request.model_state.pose.orientation = rot2_q;
            ballstate.request.model_state.reference_frame = "world";
            set_ball_service.call(ballstate);
        }

        else // !moving
        {
            if ((abs(angle_to_friend1-theta2)<0.01) && !passing)
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
                ballstate.request.model_state.pose.position.x = x2 + (0.11*sin(((PI/2)-(theta2))));
                ballstate.request.model_state.pose.position.y = y2 + (0.11*cos(((PI/2)-(theta2))));
                ballstate.request.model_state.pose.position.z = 0.01;
                ballstate.request.model_state.pose.orientation = rot2_q;
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
    ros::init(argc, argv, "robot_2gk");
    ros::NodeHandle nh;
    
    // Publisher & Subscriber Definition
    ros::Subscriber sub = nh.subscribe("/robot_2/odom",100, newOdom2);
    ros::Subscriber sub2 = nh.subscribe("/ball_state",1, ballPos);
    ros::Subscriber sub3 = nh.subscribe("/robot_1/odom",100, newOdom1);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/robot_2/planar_vel", 10);
    ros::Publisher pub3 = nh.advertise<std_msgs::Int8>("/ball_on_robot_2", 1);

    set_ball_service = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::Rate loop_rate(400); //400 Hz

    one.data = 1;
    zero.data = 0;

    double inc_x, inc_y, inc_x_friend1, inc_y_friend1;

    while (ros::ok())
    {
        if (dribbling){pub3.publish(one);}
        else{pub3.publish(zero);}
        inc_x = goal.x -x2;
        inc_y = goal.y -y2;
        inc_x_friend1 = x1r - x2;
        inc_y_friend1 = y1r - y2;
        angle_to_goal = atan2(inc_y, inc_x);
        angle_to_friend1 = atan2(inc_y_friend1, inc_x_friend1);

        if (correction)
        {
            if (!(3.12 <=abs(theta2) && abs(theta2)<=3.14))
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
            if ((y_ball - 0.01 <= y2) && (y2 <= y_ball + 0.01))
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
            }
            else if (y_ball < y2)
            {
                if (y2 >= -1.8)
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

            else if (y2 < y_ball)
            {
                if (y2 <= 1.8)
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
        }

        else if (dribbling && moving && !correction)
        {
            if (-0.01 <= y2 && y2<=0.01)
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
                moving = false;
            }

            else if (y2 > 0.01)
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.5;
            }

            else if (y2 < -0.01)
            {
                planar_speed.angular.z = 0.0;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = -0.5;
            }
        }

        else if (dribbling && !moving && !correction)
        {
            if (angle_to_friend1 - theta2 > 0.01)
            {
                planar_speed.angular.z = 0.3;
                planar_speed.linear.x = 0.0;
                planar_speed.linear.y = 0.0;
            }

            else if (angle_to_friend1 - theta2 < -0.01)
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