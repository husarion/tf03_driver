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

        ROS_INFO("Begin While loop");
        while (ros::ok())
        {
            tf03.process_sensor_data();
        }
    }
    else if (sensor_interface == "serial")
    {
        ROS_INFO("Init TF03 Serial object");
        TF03_Serial tf03(nh);

        ROS_INFO("Begin While loop");
        while (ros::ok())
        {
            tf03.process_sensor_data();
        }
    }
    else
    {
        ROS_ERROR("sensor_interface not a corecct value. Choose \"serial\" or \"can\"");
    }

    return 0;
}