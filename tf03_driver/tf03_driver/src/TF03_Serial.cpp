#include <TF03_Serial.h>

TF03_Serial::TF03_Serial(ros::NodeHandle nh): TF03_Base(nh)
{
    node_handle.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    node_handle.getParam("sensor_frame", sensor_frame);

    ROS_INFO("Initialize sensor");
    if (init_sensor(serial_port) != 0)
    {
        ROS_ERROR("Can not init sensor!");
        return;
    }

    for (auto s : sensors)
    {
        sensor_pub.insert(std::pair<int, ros::Publisher>(s.first, node_handle.advertise<sensor_msgs::Range>("sensor/" + s.second, 1)));
        sensor_data.insert(std::pair<int, sensor_msgs::Range>(s.first, sensor_msgs::Range()));
        sensor_data.at(s.first).header.frame_id = s.second;
        sensor_data.at(s.first).field_of_view = 0.00872664626;
        sensor_data.at(s.first).min_range = 0.1;
        sensor_data.at(s.first).max_range = 180;
    }
}

TF03_Serial::~TF03_Serial()
{
    if (tf03_serial_port->IsOpen())
    {
        tf03_serial_port->Close();
    }
}

void TF03_Serial::process_sensor_data()
{
    // ROS_INFO("Case serial");
    try
    {
        ReadByteFromSerial(tf03_serial_port, read_byte, timeout_ms);
    }
    catch (SerialPortNotOpen e)
    {
        ROS_ERROR("Serial port is not open");
    }
    catch (SerialPortReadTimeout e)
    {
        ROS_WARN("No data for %.2f s", (float)timeout_ms / 1000);
    }
    incoming_buffer.push_back(read_byte);
    if (is_buffer_correct(&incoming_buffer))
    {
        if (incoming_buffer[0] == 0x59)
        {
            process_incoming_buffer(std::vector<u_char>(&incoming_buffer[2], &incoming_buffer[8]));
        }
        else
        {
            process_incoming_buffer(incoming_buffer);
        }
        clear_incoming_buffer();
    }
    else
    {
        if (incoming_buffer.size() > 9)
        {
            ROS_ERROR("Buffer not correct and oversized, size=%ld", incoming_buffer.size());
        }
    }
}

int TF03_Serial::init_sensor(std::string device_name)
{

    // ROS_INFO("Create serial port object");
#ifdef LIBSERIAL_0_X
    // ROS_INFO("Using LIBSERIAL_0_x");
    tf03_serial_port = new SerialPort(device_name);
    if (!tf03_serial_port->IsOpen())
    {
        // ROS_INFO("Open serial port");
        try
        {
            tf03_serial_port->Open(
                SerialPort::BaudRate::BAUD_115200,
                SerialPort::CharacterSize::CHAR_SIZE_8,
                SerialPort::Parity::PARITY_NONE,
                SerialPort::StopBits::STOP_BITS_1,
                SerialPort::FlowControl::FLOW_CONTROL_NONE);
        }
        catch (SerialPort::OpenFailed)
        {
            ROS_ERROR("Can not open serial port");
            return 1;
        }
    }
#else
    // ROS_INFO("Using LIBSERIAL_1_x");
    tf03_serial_port = new LibSerial::SerialPort(
        device_name,
        LibSerial::BaudRate::BAUD_115200,
        LibSerial::CharacterSize::CHAR_SIZE_8,
        LibSerial::FlowControl::FLOW_CONTROL_NONE,
        LibSerial::Parity::PARITY_NONE,
        LibSerial::StopBits::STOP_BITS_1);
    if (!tf03_serial_port->IsOpen())
    {
        // ROS_INFO("Open serial port");
        try
        {
            tf03_serial_port->Open(device_name);
        }
        catch (LibSerial::OpenFailed)
        {
            ROS_ERROR("Can not open serial port");
            return 1;
        }
    }
#endif
    return 0;
}

void TF03_Serial::send_command(tf_03_command_id command_id, int64_t command_argument)
{
    std::vector<u_char> command = {0x5a};
    switch (command_id)
    {
    case tf_03_command_id::version:
        ROS_INFO("TF03_Serial::send_command: version");
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(command);
        break;
    case tf_03_command_id::system_reset:
        ROS_INFO("TF03_Serial::send_command: system_reset");
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(command);
        break;
    case tf_03_command_id::save_settings:
        ROS_INFO("TF03_Serial::send_command:save settings");
        // command: 5A 04 11 6F
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(command);
        break;
    case tf_03_command_id::output_format:
        ROS_INFO("TF03_Serial::send_command: output format");
        // Serial:  5A 05 45 01 A5
        // CAN:     5A 05 45 02 A6
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(command_argument);
        command.push_back(get_checksum(command));
        write_command_data(command);
        break;
    case tf_03_command_id::transmit_can_id:
    {
        ROS_INFO("TF03_Serial::send_command: transmit CAN ID");
        //          5A 08 50 H1 H2 H3 H4 SU
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        // ID=(H4 << 24)+(H3 << 16)+(H2 << 8)+H1
        uint32_t transmit_id = command_argument;
        u_char h1 = (transmit_id >> 0) & 0b00000000000000000000000011111111;
        u_char h2 = (transmit_id >> 8) & 0b00000000000000000000000011111111;
        u_char h3 = (transmit_id >> 16) & 0b00000000000000000000000011111111;
        u_char h4 = (transmit_id >> 24) & 0b00000000000000000000000011111111;
        command.push_back(h1);
        command.push_back(h2);
        command.push_back(h3);
        command.push_back(h4);
        command.push_back(get_checksum(command));
        write_command_data(command);
    }
    break;
    case tf_03_command_id::receive_can_id:
    {
        ROS_INFO("TF03_Serial::send_command: receive CAN ID");
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        // ID=(H4 << 24)+(H3 << 16)+(H2 << 8)+H1
        uint32_t transmit_id = command_argument;
        u_char h1 = (transmit_id >> 0) & 0b00000000000000000000000011111111;
        u_char h2 = (transmit_id >> 8) & 0b00000000000000000000000011111111;
        u_char h3 = (transmit_id >> 16) & 0b00000000000000000000000011111111;
        u_char h4 = (transmit_id >> 24) & 0b00000000000000000000000011111111;
        command.push_back(h1);
        command.push_back(h2);
        command.push_back(h3);
        command.push_back(h4);
        command.push_back(get_checksum(command));
        write_command_data(command);
    }
    break;
    default:
        ROS_ERROR("TF03_Serial::send_command: COMMAND NOT KNOWN");

        break;
    }
}

void TF03_Serial::write_command_data(std::vector<u_char> data)
{
    tf03_serial_port->Write(data);
}

std::map<tf_03_command_id, u_char> TF03_Serial::tf_03_command_len = {
    {version, 4},
    {system_reset, 4},
    {framerate, 6},
    {factory_settings, 4},
    {save_settings, 4},
    {output_format, 5},
    {transmit_can_id, 0x08},
    {receive_can_id, 0x08}};