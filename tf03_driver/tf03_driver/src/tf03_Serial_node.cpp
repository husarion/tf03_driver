#include <ros/ros.h>
#include <TF03_Serial.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf03_Serial_node");
    ros::NodeHandle nh("~");
    ROS_INFO("Init TF03 object");
    TF03_Serial tf03(nh);

    bool do_configure;
    nh.param<bool>("configure", do_configure, false);

    if (do_configure)
    {
        ROS_INFO("Start configuration");
        tf03.configureSensor();
    }
    else
    {
        ROS_INFO("Begin While loop");
        while (ros::ok())
        {
            tf03.process_sensor_data();
        }
    }

    return 0;
}