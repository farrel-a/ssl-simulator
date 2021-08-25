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
bool dribbling = false;
bool passing = false;
double x_ball = 0.0;
double y_ball = 0.0;
double distance = 0.0;
double angle_to_goal;
double angle_to_friend;
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
    distance = sqrt(pow((x_ball-x4),2.0) + pow((y_ball-y4),2.0));
    dribbling = isDribbling(distance);
    if (bt1)
    {
        dribbling = false;
        passing = false;
        goal.x = -5.2;
        goal.y = 1.2;
    }
    else if (!bt1)
    {
        if (dribbling)
        {
            if ((speed.linear.x == 0.0) && (speed.angular.z == 0.0) && (abs(angle_to_friend-theta4) < 0.01) & (!passing))
            {
                ballstate.request.model_state.model_name = "ssl_ball_1";
                ballstate.request.model_state.pose.position.x = 0.0;
                ballstate.request.model_state.pose.position.y = 0.0;
                ballstate.request.model_state.pose.position.z = 0.0;
                ballstate.request.model_state.twist.linear.x = 2.0;
                ballstate.request.model_state.reference_frame = "ssl_ball_1";
                set_ball_service.call(ballstate);
                sleep(3);
                passing = true;   
            }

            else if(!passing)
            {
                ballstate.request.model_state.model_name = "ssl_ball_1";
                ballstate.request.model_state.pose.position.x = x4 + (0.11*sin(((PI/2)-(theta4))));
                ballstate.request.model_state.pose.position.y = y4 + (0.11*cos(((PI/2)-(theta4))));
                ballstate.request.model_state.pose.position.z = 0.01;
                ballstate.request.model_state.pose.orientation = rot4_q;
                ballstate.request.model_state.reference_frame = "world";
                set_ball_service.call(ballstate);
            }
        }

        else if (passing)
        {
            goal.x = 1.5;
            goal.y = 1.5;
        }

        else
        {
            dribbling = false;
            goal.x = msg.pose[0].position.x;
            goal.y = msg.pose[0].position.y;
        }
    }
}

//Publisher & Subscriber Declaration

ros::Publisher pub;

void br2Callback(const std_msgs::Int8& msg)
{
    if (msg.data == 1){br2 = true;}
    else {br2 = false;}
}

void br3Callback(const std_msgs::Int8 msg)
{
    if (msg.data==1){br3=true;}
    else{br3=false;}
}

void br1Callback(const std_msgs::Int8 msg)
{
    if (msg.data==1){br1=true;}
    else{br1=false;}
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
    ros::init(argc, argv, "robot_4");
    ros::NodeHandle nh;

    // Publisher & Subscriber Definition
    ros::Subscriber sub = nh.subscribe("/robot_4/odom",100, newOdom4);
    ros::Subscriber sub2 = nh.subscribe("/ball_state",1, ballPos);
    ros::Subscriber sub3 = nh.subscribe("/robot_6/odom",100, newOdom6);
    ros::Subscriber sub4 = nh.subscribe("/ball_on_robot_2",1,br2Callback);
    ros::Subscriber sub5 = nh.subscribe("/ball_on_robot_3",1,br3Callback);
    ros::Subscriber sub6 = nh.subscribe("/ball_on_robot_1",1,br1Callback);
    ros::Subscriber sub7 = nh.subscribe("/ball_on_robot_5",1,br5Callback);
    ros::Subscriber sub8 = nh.subscribe("/ball_on_robot_6",1,br6Callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot_4/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int8>("/ball_on_robot_4", 1);

    set_ball_service = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::Rate loop_rate(100); //100 Hz

    one.data = 1;
    zero.data = 0;

    double inc_x, inc_y, inc_x_friend, inc_y_friend;

    while(ros::ok())
    {
        if (dribbling){br4 = true;pub2.publish(one);}
        else{br4=false;pub2.publish(zero);}
        checkTeam(br1,br2,br3,br4,br5,br6);

        inc_x = goal.x - x4;
        inc_y = goal.y - y4;
        inc_x_friend = x6-x4;
        inc_y_friend = y6-y4;
        angle_to_goal = atan2(inc_y, inc_x);
        angle_to_friend = atan2(inc_y_friend, inc_x_friend);

        if (br5)
        {
            speed.linear.x = 0.0;
            speed.angular.z = 0.0;
        }

        else
        {
            if (!dribbling)
            {
                if ((angle_to_goal-theta4)>0.2)
                {
                    speed.linear.x = 0.0;
                    if (angle_to_goal-theta4 > 0.5){speed.angular.z=2.0;}
                    else{speed.angular.z=0.4;}
                }
                else if (angle_to_goal-theta4<-0.2)
                {
                    speed.linear.x=0.0;
                    if (angle_to_goal-theta4<-0.5){speed.angular.z=-2.0;}
                    else{speed.angular.z=-0.4;}
                }
                else{speed.angular.z=0.0;speed.linear.x=0.6;}
            }

            else if (dribbling && !passing)
            {
                if (angle_to_friend-theta4>0.01)
                {
                    speed.angular.z = 0.5;
                    speed.linear.x = 0.0;
                }
                else if (angle_to_friend-theta4<-0.01)
                {
                    speed.angular.z = -0.5;
                    speed.linear.x = 0.0;
                }
                else
                {
                    speed.angular.z = 0.0;
                    speed.linear.x = 0.0;
                    dribbling = false;
                }
            }
        }
        pub.publish(speed);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}