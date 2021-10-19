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
        TF03_CAN tf03(nh);
        tf03.mainLoop();
    }
    else if (sensor_interface == "serial")
    {
        TF03_Serial tf03(nh);
        tf03.mainLoop();
    }
    else
    {
        ROS_ERROR("sensor_interface not a corect value: %s. Choose \"serial\" or \"can\"", sensor_interface.c_str());
    }

    return 0;
}