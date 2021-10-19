#include <math.h>
#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#ifndef TF03_BASE_H
#define TF03_BASE_H

#define SET_OUTPUT_FORMAT_SERIAL 0x01
#define SET_OUTPUT_FORMAT_CAN 0x02

enum tf_03_interface
{
    serial = 0,
    can = 1
};

enum tf_03_command_id
{
    version = 0x01,
    system_reset = 0x02,
    framerate = 0x03,
    factory_settings = 0x10,
    save_settings = 0x11,
    output_format = 0x45,
    transmit_can_id = 0x50,
    receive_can_id = 0x51,
};

struct parameter_config
{
    bool frame_sent;
    bool command_success;
    tf_03_command_id command;
    int64_t argument;
};

class TF03_Base
{
public:
    TF03_Base(ros::NodeHandle nh) {node_handle = nh;};
    virtual ~TF03_Base() {};

    u_char get_checksum(std::vector<u_char> data);
    void process_incoming_buffer(std::vector<u_char> data, int can_id = 0);
    
    void configureSensor();

    // ROS
    ros::NodeHandle node_handle;
    std::map<int, ros::Publisher> sensor_pub;
    std::map<int, sensor_msgs::Range> sensor_data;

    ros::Time command_timestamp;

    std::vector<parameter_config> parameters;
    std::vector<u_char> incoming_buffer;

    void mainLoop();

    void clear_incoming_buffer();
    bool is_buffer_correct(std::vector<u_char> *data);

private:
    virtual int init_sensor(std::string device_name) = 0;
    virtual void send_command(tf_03_command_id command_id, int64_t command_argument = 0) = 0;
    virtual void write_command_data(std::vector<u_char> data) = 0;
    virtual void process_sensor_data() = 0;
    bool verify_checksum(std::vector<u_char> data);
};

#endif
