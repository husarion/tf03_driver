#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#ifdef LIBSERIAL_0_X
#include <SerialPort.h>
#include <SerialStream.h>
#define SerialPortNotOpen SerialPort::NotOpen
#define SerialPortReadTimeout SerialPort::ReadTimeout
#define ReadByteFromSerial(port, buffer, timeout) (buffer = port->ReadByte(timeout))
#else
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#define SerialPortNotOpen LibSerial::NotOpen
#define SerialPortReadTimeout LibSerial::ReadTimeout
#define ReadByteFromSerial(port, buffer, timeout) (port->ReadByte(buffer, timeout))
#endif

#ifndef TF03_H
#define TF03_H

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

class TF03
{
public:
    TF03(ros::NodeHandle nh);
    ~TF03();

private:
    int init_sensor(tf_03_interface i, std::string device_name);
    void send_command(tf_03_interface i, tf_03_command_id command_id, int64_t command_argument = 0);
    void write_command_data(tf_03_interface i, std::vector<u_char> data);
    void clear_incoming_buffer();
    void process_incoming_buffer(std::vector<u_char> data, int can_id = 0);
    bool is_buffer_correct(std::vector<u_char> *data);
    bool verify_checksum(std::vector<u_char> data);
    u_char get_checksum(std::vector<u_char> data);
    void process_sensor_data();
    ros::NodeHandle node_handle;
    std::map<int, ros::Publisher> sensor_pub;
    std::map<int, sensor_msgs::Range> sensor_data;
    tf_03_interface interface;
    std::string sensor_interface;
    std::string can_device;
    int can_receive_id;
    std::vector<int> can_transmit_id;
    std::vector<std::string> sensor_frame;
    std::map<int, std::string> sensors;
    std::string serial_port;
    std::string line;
    u_char read_byte;
    uint32_t timeout_ms;
    std::vector<u_char> incoming_buffer;
    u_char low_byte;
    u_char high_byte;
    u_char checksum;
    u_char checksum_byte;
    uint16_t dist;
    float dist_meters;
    struct can_frame *frame;
    int64_t can_socket;
    bool print_version;
    std::string set_output_format;
    int set_transmit_can_id;
    int set_receive_can_id;
    bool reconfigure_sensor;
    ros::Time command_timestamp;
#ifdef LIBSERIAL_0_X
    SerialPort *tf03_serial_port;
#else
    LibSerial::SerialPort *tf03_serial_port;
#endif
    static std::map<tf_03_command_id, u_char> tf_03_command_len;
    std::vector<parameter_config> parameters;
};
#endif
