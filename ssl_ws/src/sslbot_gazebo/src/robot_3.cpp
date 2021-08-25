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

double x3 = 0.0;
double y3 = 0.0;
double theta3 = 0.0;
geometry_msgs::Quaternion rot3_q;

void newOdom3(const nav_msgs::Odometry& msg)
{
    x3 = msg.pose.pose.position.x;
    y3 = msg.pose.pose.position.y;
    rot3_q = msg.pose.pose.orientation;

    tf::Quaternion q(rot3_q.x, rot3_q.y, rot3_q.z, rot3_q.w);
    tf::Matrix3x3 m(q);

    double R,P,Y;

    m.getRPY(R,P,Y);

    theta3 = Y;
}

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

bool isInEnemyPenalty(double a, double b)
{
    if ((-7.0<=a && a<=-4.0) && (-0.5<=b && b<=0.5))
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
    distance = sqrt(pow((x_ball-x3),2.0) + pow((y_ball-y3),2.0));
    dribbling = isDribbling(distance);
    if (bt2)
    {
        shooting = false;
        goal.x = 5.2;
        goal.y = 1.2;
    }
    else
    {
        if (dribbling)
        {
            goal.x = -4.50;
            goal.y = -0.35;

            if (!isInEnemyPenalty(x3,y3))
            {
                shooting = false;
                ballstate.request.model_state.model_name = "ssl_ball_1";
                ballstate.request.model_state.pose.position.x = x3 + (0.11*sin(((PI/2)-(theta3))));
                ballstate.request.model_state.pose.position.y = y3 + (0.11*cos(((PI/2)-(theta3))));
                ballstate.request.model_state.pose.position.z = 0.01;
                ballstate.request.model_state.pose.orientation = rot3_q;
                ballstate.request.model_state.reference_frame = "world";
                set_ball_service.call(ballstate); //call set_model_state to set ball in front of bot
            }
            else if (isInEnemyPenalty(x3,y3) && !shooting)
            {
                if (speed.linear.x==0 && speed.angular.z==0 && (2.90<=abs(theta3) && abs(theta3)<=2.92 ))
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
                    ballstate.request.model_state.pose.position.x = x3 + (0.11*sin(((PI/2)-(theta3))));
                    ballstate.request.model_state.pose.position.y = y3 + (0.11*cos(((PI/2)-(theta3))));
                    ballstate.request.model_state.pose.position.z = 0.01;
                    ballstate.request.model_state.pose.orientation = rot3_q;
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

void br6Callback(const std_msgs::Int8 msg)
{
    if (msg.data==1){br6=true;}
    else{br6=false;}
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
    ros::init(argc, argv, "robot_3");
    ros::NodeHandle nh;

    // Publisher & Subscriber Definition
    ros::Subscriber sub2 = nh.subscribe("/ball_state",10, ballPos);
    ros::Subscriber sub3 = nh.subscribe("/robot_3/odom",100, newOdom3);
    ros::Subscriber sub4 = nh.subscribe("/robot_5/odom",100,newOdom5);
    ros::Subscriber sub5 = nh.subscribe("/ball_on_robot_2",1,br2Callback);
    ros::Subscriber sub6 = nh.subscribe("/ball_on_robot_4",1,br4Callback);
    ros::Subscriber sub7 = nh.subscribe("/ball_on_robot_5",1,br5Callback);
    ros::Subscriber sub8 = nh.subscribe("/ball_on_robot_6",1,br6Callback);
    ros::Subscriber sub9 = nh.subscribe("/ball_on_robot_1",1,br1Callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int8>("/ball_on_robot_3", 1);

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
        inc_x = goal.x -x3;
        inc_y = goal.y -y3;
        angle_to_goal = atan2(inc_y, inc_x);
        if (bt2)
        {
            if (angle_to_goal - theta3 > 0.2)
            {
                speed.linear.x = 0.0;
                if (angle_to_goal - theta3 > 0.5)
                {
                    speed.angular.z = 2.0;
                }
                else{speed.angular.z = 0.4;}
            }
            else if (angle_to_goal - theta3 < -0.2)
            {
                speed.linear.x = 0.0;
                if (angle_to_goal - theta3 < -0.5)
                {speed.angular.z = -2.0;}
                else{speed.angular.z = -0.4;}
            }
            else{speed.angular.z=0.0;speed.linear.x=0.6;}
        }
        else //!bt2
        {
            if (!isInEnemyPenalty(x3,y3) && dribbling)
            {
                if (angle_to_goal - theta3 > 0.2)
                {
                    speed.linear.x = 0.0;
                    if (angle_to_goal - theta3 > 0.5)
                    {
                        speed.angular.z = 2.0;
                    }
                    else {speed.angular.z = 0.4;}
                }
                else if (angle_to_goal - theta3 < -0.2)
                {
                    speed.linear.x = 0.0;
                    if (angle_to_goal - theta3 < -0.5)
                    {
                        speed.angular.z = -2.0;
                    }
                    else{speed.angular.z = -0.4;}
                }
                else{speed.angular.z = 0.0; speed.linear.x = 0.6;}
            }
            else
            {
                if (isInEnemyPenalty(x3,y3))
                {
                    if ((2.90 <= abs(theta3) && abs(theta3) <= 2.92) && speed.angular.z != 0)
                    {
                        speed.angular.z = 0.0;
                        speed.linear.x = 0.0;
                    }
                    else if ((2.90 <= abs(theta3) && abs(theta3) <= 2.92) && speed.angular.z == 0)
                    {
                        speed.angular.z = 0.0;
                        speed.linear.x = 0.0;
                    }
                    else if (y5>0)
                    {
                        speed.angular.z = -1.0;
                        speed.linear.x = 0.0;
                    }
                    else if (y5<0)
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