#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iostream>
#include <effort_controllers/joint_velocity_controller.h>
#include <dynamic_tutorial/tutorialConfig.h>
#include <dynamic_reconfigure/client.h>
hardware_interface::JointHandle front_left_joint_,back_left_joint_,front_right_joint_,back_right_joint_;
double front_left_command_,front_right_command_,back_right_command_,back_left_command_,base,track,vx,vy,vth;
void callback(const geometry_msgs::Twist &cmd_vel)
{

    ROS_INFO("Received a /cmd_vel message!");
    vx=cmd_vel.linear.x;
    vy=cmd_vel.linear.y;
    vth=cmd_vel.angular.z;
    ROS_INFO("wheel_track=%f,wheel_base=%f",track,base);
    ROS_INFO("vx=:[%f],vy=:[%f],vth=:[%f]", vx,vy,vth);
    front_left_command_ = vx-vy-vth*(track+base)/2;
    front_right_command_ =vx+vy+vth*(track+base)/2;
    back_left_command_ = vx-vy+vth*(track+base)/2;
    back_right_command_ =vx+vy-vth*(track+base)/2;
    ROS_INFO("v1=%f,v2=%f,v3=%f,v4=%f",front_left_command_,front_right_command_,back_right_command_,back_left_command_);
//    front_left_joint_.setCommand(front_left_command_);
//    front_right_joint_.setCommand(front_right_command_);
//    back_left_joint_.setCommand(back_left_command_ );
//    back_right_joint_.setCommand(back_right_command_);

}
void dynCallBack(const dynamic_tutorial::tutorialConfig &config)
{
    ROS_INFO("wheel_track=%f,wheel_base=%f",config.wheel_track,config.wheel_base);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
    dynamic_reconfigure::Client<dynamic_tutorial::tutorialConfig> dynamic_client("dynamic_tutorial_node", dynCallBack);
    dynamic_tutorial::tutorialConfig config;
    dynamic_tutorial::tutorialConfig cmd_dynamic_client;
    cmd_dynamic_client.wheel_track=0.500;
    cmd_dynamic_client.wheel_base=0.475;
    base=cmd_dynamic_client.wheel_base;
    track=cmd_dynamic_client.wheel_track;
    ros::spin();
    return 1;
}
