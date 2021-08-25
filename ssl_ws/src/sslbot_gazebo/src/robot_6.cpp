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

double x6 = 0.0;
double y6 = 0.0;
double theta6 = 0.0;
geometry_msgs::Quaternion rot6_q;

void newOdom6(const nav_msgs::Odometry& msg)
{
    x6 = msg.pose.pose.position.x;
    y6 = msg.pose.pose.position.y;
    rot6_q = msg.pose.pose.orientation;

    tf::Quaternion q(rot6_q.x, rot6_q.y, rot6_q.z, rot6_q.w);
    tf::Matrix3x3 m(q);

    double R,P,Y;

    m.getRPY(R,P,Y);

    theta6 = Y;
}

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

bool isInEnemyPenalty(double a, double b)
{
    if ((4.0<=a && a<=7.0) && (-0.5<=b && b<=0.5))
    {
        return true;
    }
    else
    {
        return false;
    }
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
double direction  = 0.0;
double distance = 0.0;
bool shooting = false;
double x_ball = 0.0;
double y_ball = 0.0;
double angle_to_goal;
geometry_msgs::Twist speed;

bool br1 = false;
bool br2 = false;
bool br3 = false;
bool br4 = false;
bool br5 = false;
bool br6 = false;
bool bt1 = false;
bool bt2 = false;

void ballPos(const gazebo_msgs::ModelStates& msg)
{
    x_ball = msg.pose[0].position.x;
    y_ball = msg.pose[0].position.y;
    distance = sqrt(pow((x_ball-x6),2.0) + pow((y_ball-y6),2.0));
    dribbling = isDribbling(distance);
    if (bt1)
    {
        shooting = false;
        goal.x = -5.2;
        goal.y = -1.2;
    }
    else
    {
        if (dribbling)
        {
            goal.x = 4.50;
            goal.y = 0.35;

            if (!isInEnemyPenalty(x6,y6))
            {
                shooting = false;
                ballstate.request.model_state.model_name = "ssl_ball_1";
                ballstate.request.model_state.pose.position.x = x6 + (0.11*sin(((PI/2)-(theta6))));
                ballstate.request.model_state.pose.position.y = y6 + (0.11*cos(((PI/2)-(theta6))));
                ballstate.request.model_state.pose.position.z = 0.01;
                ballstate.request.model_state.pose.orientation = rot6_q;
                ballstate.request.model_state.reference_frame = "world";
                set_ball_service.call(ballstate); //call set_model_state to set ball in front of bot
            }
            else if (isInEnemyPenalty(x6,y6) && !shooting)
            {
                if (speed.linear.x==0 && speed.angular.z==0 && (0.28<=abs(theta6) && abs(theta6)<=0.30))
                {
                    ballstate.request.model_state.model_name = "ssl_ball_1";
                    ballstate.request.model_state.pose.position.x = 0.0;
                    ballstate.request.model_state.pose.position.y = 0.0;
                    ballstate.request.model_state.pose.position.z = 0.0;
                    ballstate.request.model_state.twist.linear.x = 3.0;
                    ballstate.request.model_state.reference_frame = "ssl_ball_1";
                    set_ball_service.call(ballstate);
                    dribbling = false;
                    shooting = true;
                    sleep(3);
                    visited = false;
                }
                else if (!shooting)
                {
                    ballstate.request.model_state.model_name = "ssl_ball_1";
                    ballstate.request.model_state.pose.position.x = x6 + (0.11*sin(((PI/2)-(theta6))));
                    ballstate.request.model_state.pose.position.y = y6 + (0.11*cos(((PI/2)-(theta6))));
                    ballstate.request.model_state.pose.position.z = 0.01;
                    ballstate.request.model_state.pose.orientation = rot6_q;
                    ballstate.request.model_state.reference_frame = "world";
                    set_ball_service.call(ballstate);
                }
            }
        }
    }
}

void br2Callback(const std_msgs::Int8& msg)
{
    if (msg.data == 1){br2 = true;}
    else {br2 = false;}
}

void br1Callback(const std_msgs::Int8 msg)
{
    if (msg.data==1){br1=true;}
    else{br1=false;}
}

void br4Callback(const std_msgs::Int8 msg)
{
    if (msg.data==1){br4=true;}
    else{br4=false;}
}

void br5Callback(const std_msgs::Int8 msg)
{
    if (msg.data==1){br5=true;}
    else{br5=false;}
}

void br3Callback(const std_msgs::Int8 msg)
{
    if (msg.data==1){br3=true;}
    else{br3=false;}
}

void checkTeam(bool br_1, bool br_2, bool br_3, bool br_4, bool br_5, bool br_6)
{
    if (br_1 ||br_2||br_3)
    {
        bt1=true;bt2=false;
    }
    else if (br_4||br_5||br_6)
    {
        bt1=false;bt2=true;
    }
    else{bt1=false;bt2=false;}
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "robot_6");
    ros::NodeHandle nh;

    // Publisher & Subscriber Definition
    ros::Subscriber sub2 = nh.subscribe("/ball_state",10, ballPos);
    ros::Subscriber sub3 = nh.subscribe("/robot_6/odom",100, newOdom6);
    ros::Subscriber sub4 = nh.subscribe("/robot_2/odom",100,newOdom2);
    ros::Subscriber sub5 = nh.subscribe("/ball_on_robot_2",1,br2Callback);
    ros::Subscriber sub6 = nh.subscribe("/ball_on_robot_4",1,br4Callback);
    ros::Subscriber sub7 = nh.subscribe("/ball_on_robot_5",1,br5Callback);
    ros::Subscriber sub8 = nh.subscribe("/ball_on_robot_3",1,br3Callback);
    ros::Subscriber sub9 = nh.subscribe("/ball_on_robot_1",1,br1Callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot_6/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int8>("/ball_on_robot_6", 1);

    set_ball_service = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::Rate loop_rate(100); //100 Hz

    one.data = 1;
    zero.data = 0;

    double inc_x, inc_y;

    while (ros::ok())
    {
        if (dribbling){pub2.publish(one);}
        else{pub2.publish(zero);}
        checkTeam(br1, br2, br3, br4, br5, br6);
        inc_x = goal.x -x6;
        inc_y = goal.y -y6;
        angle_to_goal = atan2(inc_y, inc_x);
        if (bt1)
        {
            if (angle_to_goal - theta6 > 0.2)
            {
                speed.linear.x = 0.0;
                if (angle_to_goal - theta6 > 0.5)
                {
                    speed.angular.z = 2.0;
                }
                else{speed.angular.z = 0.4;}
            }
            else if (angle_to_goal - theta6 < -0.2)
            {
                speed.linear.x = 0.0;
                if (angle_to_goal - theta6 < -0.5)
                {speed.angular.z = -2.0;}
                else{speed.angular.z = -0.4;}
            }
            else{speed.angular.z=0.0;speed.linear.x=0.6;}
        }
        else //!bt1
        {
            if (!isInEnemyPenalty(x6,y6) && dribbling)
            {
                if (angle_to_goal - theta6 > 0.2)
                {
                    speed.linear.x = 0.0;
                    if (angle_to_goal - theta6 > 0.5)
                    {
                        speed.angular.z = 2.0;
                    }
                    else {speed.angular.z = 0.4;}
                }
                else if (angle_to_goal - theta6 < -0.2)
                {
                    speed.linear.x = 0.0;
                    if (angle_to_goal - theta6 < -0.5)
                    {
                        speed.angular.z = -2.0;
                    }
                    else{speed.angular.z = -0.4;}
                }
                else{speed.angular.z = 0.0; speed.linear.x = 0.6;}
            }
            else
            {
                if (isInEnemyPenalty(x6,y6))
                {
                    if ((0.28 <= abs(theta6) && abs(theta6) <= 0.30) && speed.angular.z != 0)
                    {
                        speed.angular.z = 0.0;
                        speed.linear.x = 0.0;
                    }
                    else if ((0.28 <= abs(theta6) && abs(theta6) <= 0.30) && speed.angular.z == 0)
                    {
                        speed.angular.z = 0.0;
                        speed.linear.x = 0.0;
                    }
                    else if (y2>0)
                    {
                        speed.angular.z = -1.0;
                        speed.linear.x = 0.0;
                    }
                    else if (y2<0)
                    {
                        speed.angular.z = 1.0;
                        speed.linear.x = 0.0;
                    }
                }
                else{speed.linear.x=0.0;speed.angular.z=0.0;}
            }
        }
        pub.publish(speed);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}