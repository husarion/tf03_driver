#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include "TF03_Base.h"

#ifndef TF03_CAN_H
#define TF03_CAN_H

#define SET_OUTPUT_FORMAT_CAN 0x02


class TF03_CAN: public TF03_Base
{
public:
    TF03_CAN(ros::NodeHandle nh);
    virtual ~TF03_CAN();

    void process_sensor_data() override;

private:
    int init_sensor(std::string device_name) override;
    void write_command_data(std::vector<u_char> data) override;
    
    // Params
    std::string can_device;
    int can_receive_id;
    std::vector<int> can_transmit_id;
    std::vector<std::string> sensor_frame;
    std::map<int, std::string> sensors;

    struct can_frame *frame;
    int64_t can_socket;
};
#endif
