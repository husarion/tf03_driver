#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

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

#include <TF03_Base.h>

#ifndef TF03_Serial_H
#define TF03_Serial_H

class TF03_Serial: public TF03_Base
{
public:
    TF03_Serial(ros::NodeHandle nh);
    virtual ~TF03_Serial();

    void process_sensor_data() override;

private:
    int init_sensor(std::string device_name) override;
    void send_command(tf_03_command_id command_id, int64_t command_argument = 0) override;
    void write_command_data(std::vector<u_char> data) override;

    std::vector<std::string> sensor_frame;
    std::map<int, std::string> sensors;
    std::string serial_port;
    std::string line;
    u_char read_byte;
    uint32_t timeout_ms;
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
};
#endif
