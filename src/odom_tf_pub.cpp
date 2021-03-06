#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <effort_controllers/odom_pub.h>
#include <effort_controllers/joint_velocity_controller.h>


double x = 0.0;
double y = 0.0;
double th = 0.0;
//默认机器人的起始位置是odom参考系下的0点
double vx = 0;
double vy = 0;
double vth = 0;
double dt ;

double flv=0;
double frv=0;
double blv=0;
double brv=0;


void fl_velocitysubscriber (const std_msgs::Float64ConstPtr &msg)
{
    flv= msg->data;
}
void bl_velocitysubscriber (const std_msgs::Float64ConstPtr &msg)
{
    blv= msg->data;
}
void fr_velocitysubscriber (const std_msgs::Float64ConstPtr &msg)
{
    frv= msg->data;
}
void br_velocitysubscriber (const std_msgs::Float64ConstPtr &msg)
{
    brv= msg->data;
}
void messageCallback(const hero_chassis_controller::odom_pub::ConstPtr& msg) //获取上篇文章串口功能包发来的数据
{
    vx = double(msg->front_left_velocity)/1000;
    vy = double(msg->back_left_velocity)/1000;
    vth =double(msg->front_right_velocity)/1000;
    ROS_INFO("Vx = [%f] Vth = [%f]", vx,  vth);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle n;
    ros::Subscriber front_left_velocity_controller_sub= n.subscribe<std_msgs::Float64>("/controller/front_left_velocity_controller/command", 1, fl_velocitysubscriber);
    ros::Subscriber front_right_velocity_controller_sub= n.subscribe<std_msgs::Float64>("/controller/front_right_velocity_controller/command", 1, fr_velocitysubscriber);
    ros::Subscriber back_left_velocity_controller_sub= n.subscribe<std_msgs::Float64>("/controller/back_left_velocity_controller/command", 1, bl_velocitysubscriber);
    ros::Subscriber back_right_velocity_controller_sub= n.subscribe<std_msgs::Float64>("/controller/back_right_velocity_controller/command", 1, br_velocitysubscriber);
    ros::Subscriber sub = n.subscribe("odom_param_pub", 1, messageCallback);//订阅上篇文章串口功能包的话题
    ros::Publisher odom_param_pub = n.advertise<hero_chassis_controller::odom_pub>("odom_param_pub", 1);//make a param publisher
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


    ros::Rate r(20);//以20Hz的速率发布里程信息，
    while(n.ok())
    {
        //param_pub init
        hero_chassis_controller::odom_pub odom_param_msg;
        odom_param_msg.front_left_velocity=flv;
        odom_param_msg.front_right_velocity=frv;
        odom_param_msg.back_left_velocity=blv;
        odom_param_msg.back_right_velocity=brv;




        ros::spinOnce();
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;
        //在这里，我们将根据我们设定的恒定速度更新我们的里程信息。

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        //我们通常会尝试在我们的系统中使用所有消息的3D版本，以允许2D和3D组件在适当的情况下协同工作，
        //并将消息数量保持在最低限度。因此，有必要将我们的yaw(偏航值)变为四元数。
        //tf提供的功能允许从yaw(偏航值)容易地创建四元数，并且可以方便地获取四元数的偏航值。

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        /*在这里，我们将创建一个TransformStamped消息，通过tf发送。
        我们要发布在current_time时刻的从"odom"坐标系到“base_link”坐标系的变换。
        因此，我们将相应地设置消息头和child_frame_id，
        确保使用“odom”作为父坐标系，将“base_link”作为子坐标系。*/

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
//将我们的里程数据中填入变换消息中，然后使用TransformBroadcaster发送变换。

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
//还需要发布nav_msgs/Odometry消息，以便导航包可以从中获取速度信息。
//将消息的header设置为current_time和“odom”坐标系。
        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
//这将使用里程数据填充消息，并发送出去。
//我们将消息的child_frame_id设置为“base_link”坐标系，
//因为我们要发送速度信息到这个坐标系。

        //publish the message
        odom_pub.publish(odom);
        odom_param_pub.publish(odom_param_msg);
        last_time = current_time;

        r.sleep();
    }
    // return 0;
}