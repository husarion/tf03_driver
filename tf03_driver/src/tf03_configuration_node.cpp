#include <ros/ros.h>
#include <TF03_CAN.h>
#include <TF03_Serial.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf03_CAN_node");
    ros::NodeHandle nh("~");

    std::string sensor_interface;

    if (!(nh.param<std::string>("sensor_interface", sensor_interface, "")))
    {
        ROS_ERROR("sensor_interface not loaded correctly");
    }

    if (sensor_interface == "can")
    {
        ROS_INFO("Init TF03 CAN object");
        TF03_CAN tf03(nh);

        ROS_INFO("Start configuration");
        tf03.configureSensor();
    }
    else if (sensor_interface == "serial")
    {
        ROS_INFO("Init TF03 Serial object");
        TF03_Serial tf03(nh);

        ROS_INFO("Start configuration");
        tf03.configureSensor();
    }
    else
    {
        ROS_ERROR("sensor_interface not a corecct value. Choose \"serial\" or \"can\"");
    }

    return 0;
}