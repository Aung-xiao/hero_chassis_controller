#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorial/tutorialConfig.h>

void callback(dynamic_tutorial::tutorialConfig &config,uint32_t level)
{
    ROS_INFO("Reconfigure request:%f,%f",
             config.wheel_base,config.wheel_track);
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"dynamic_tutorial_node");
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<dynamic_tutorial::tutorialConfig> server;
    dynamic_reconfigure::Server<dynamic_tutorial::tutorialConfig>::CallbackType f;
    f = boost::bind(&callback,_1,_2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
