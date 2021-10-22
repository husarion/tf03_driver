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

    sensors.insert(std::pair<int, std::string>(0, sensor_frame[0]));

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

void TF03_Serial::write_command_data(std::vector<u_char> data)
{
    ROS_INFO("SENDING DATA");
    tf03_serial_port->Write(data);
}