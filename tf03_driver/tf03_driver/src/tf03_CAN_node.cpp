#include <ros/ros.h>
#include <TF03_CAN.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf03_CAN_node");
    ros::NodeHandle nh("~");
    ROS_INFO("Init TF03 object");
    TF03_CAN tf03(nh);
    return 0;
}