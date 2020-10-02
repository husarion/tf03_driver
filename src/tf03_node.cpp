#include <ros/ros.h>
#include <TF03.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf03_node");
    ros::NodeHandle nh("~");
    ROS_INFO("Init TF03 object");
    TF03 tf03(nh);
    return 0;
}