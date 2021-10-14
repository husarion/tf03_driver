#include <TF03.h>

TF03::TF03(ros::NodeHandle nh)
{
    node_handle = nh;
    timeout_ms = 1000;
    node_handle.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    node_handle.param<std::string>("can_device", can_device, "panther_can");
    node_handle.getParam("can_transmit_id", can_transmit_id);
    node_handle.getParam("sensor_frame", sensor_frame);
    node_handle.param<int>("can_receive_id", can_receive_id, 0x3003); //HEX 0x3003 = DEC 12291
    node_handle.param<std::string>("sensor_interface", sensor_interface, "serial");

    node_handle.param<bool>("print_version", print_version, false);
    node_handle.param<std::string>("set_output_format", set_output_format, "");
    node_handle.param<int>("set_transmit_can_id", set_transmit_can_id, 0);
    node_handle.param<int>("set_receive_can_id", set_receive_can_id, 0);
    reconfigure_sensor = false;

    if (print_version)
    {
        ROS_INFO("Print tf_03 version");
        reconfigure_sensor = true;
        parameters.push_back(parameter_config{false, false, tf_03_command_id::version, 0});
    }
    if (set_transmit_can_id != 0)
    {
        reconfigure_sensor = true;
        ROS_INFO("New value for set_transmit_can_id %#4x", set_transmit_can_id);
        parameters.push_back(parameter_config{false, false, tf_03_command_id::transmit_can_id, set_transmit_can_id});
    }
    if (set_receive_can_id != 0)
    {
        reconfigure_sensor = true;
        ROS_INFO("New value for set_receive_can_id %#4x", set_receive_can_id);
        parameters.push_back(parameter_config{false, false, tf_03_command_id::receive_can_id, set_receive_can_id});
    }
    if (set_output_format.compare("") != 0)
    {
        if (set_output_format.compare("serial") == 0)
        {
            ROS_INFO("New value for set_output_format %s", set_output_format.c_str());
            reconfigure_sensor = true;
            parameters.push_back(parameter_config{false, false, tf_03_command_id::output_format, SET_OUTPUT_FORMAT_SERIAL});
        }
        else if (set_output_format.compare("can") == 0)
        {
            ROS_INFO("New value for set_output_format %s", set_output_format.c_str());
            reconfigure_sensor = true;
            parameters.push_back(parameter_config{false, false, tf_03_command_id::output_format, SET_OUTPUT_FORMAT_CAN});
        }
        else
        {
            ROS_ERROR("Invalid value for parameter set_output_format. Valid are serial, can.");
        }
    }
    if (sensor_interface.compare("serial") == 0)
    {
        interface = tf_03_interface::serial;
    }
    else if (sensor_interface.compare("can") == 0)
    {
        interface = tf_03_interface::can;
        ROS_INFO("Will use CAN interface '%s' with device ID %#04x", can_device.c_str(), can_receive_id);
    }
    else
    {
        ROS_ERROR("Valid interfaces are: serial, can");
        return;
    }

    if (sensor_frame.size() == can_transmit_id.size() && can_transmit_id.size() > 0)
    {
        for (int i = 0; i < sensor_frame.size(); i++)
        {
            sensors.insert(std::pair<int, std::string>(can_transmit_id[i], sensor_frame[i]));
        }
    }
    else if (can_transmit_id.size() > 0)
    {
        for (auto id : can_transmit_id)
        {
            sensors.insert(std::pair<int, std::string>(id, "sensor_at_CAN_ID_" + std::to_string(id)));
        }
    }
    else
    {
        sensors.insert(std::pair<int, std::string>(0, "tf03_sensor"));
    }

    ROS_INFO("Initialize sensor");
    if (init_sensor(interface, (interface == tf_03_interface::serial) ? serial_port : can_device) != 0)
    {
        ROS_ERROR("Can not init sensor!");
        return;
    }

    if (reconfigure_sensor)
    {
        parameters.push_back(parameter_config{false, false, tf_03_command_id::save_settings, 0});

        ROS_INFO("Reconfiguring sensor");
        while (parameters.size() > 0)
        {
            if (parameters[0].frame_sent == false)
            {
                send_command(interface, parameters[0].command, parameters[0].argument);
                command_timestamp = ros::Time::now();
                parameters[0].frame_sent = true;
            }
            else if (command_timestamp + ros::Duration(2) < ros::Time::now())
            {
                ROS_ERROR("Timeout for command %#2x passed", parameters[0].command);
                ROS_ERROR("Configuation failed");
                return;
            }
            else if (parameters[0].command_success)
            {
                ROS_INFO("Command success");
                parameters.erase(parameters.begin());
            }
            process_sensor_data();
        }
    }
    else
    {
        for (auto s : sensors)
        {
            sensor_pub.insert(std::pair<int, ros::Publisher>(s.first, node_handle.advertise<sensor_msgs::Range>("sensor/" + s.second, 1)));
            sensor_data.insert(std::pair<int, sensor_msgs::Range>(s.first, sensor_msgs::Range()));
            sensor_data.at(s.first).header.frame_id = s.second;
            sensor_data.at(s.first).field_of_view = 0.00872664626;
            sensor_data.at(s.first).min_range = 0.1;
            sensor_data.at(s.first).max_range = 180;
        }
        ROS_INFO("Begin While loop");
        while (ros::ok())
        {
            process_sensor_data();
        }
    }
}

TF03::~TF03()
{
    if (interface == tf_03_interface::serial)
    {
        if (tf03_serial_port->IsOpen())
        {
            tf03_serial_port->Close();
        }
    }
    else
    {
        if (can_socket != -1)
        {
            close(can_socket);
        }
    }
}

void TF03::process_sensor_data()
{
    switch (interface)
    {
    case tf_03_interface::serial:
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
        break;
    case tf_03_interface::can:
        struct can_frame publish_frame;
        publish_frame.can_id = 0;
        publish_frame.__pad = 0;
        publish_frame.can_dlc = 0;
        for (auto d : publish_frame.data)
        {
            d = 0;
        }
        int nbytes = read(can_socket, &publish_frame, sizeof(struct can_frame));
        for (auto id : can_transmit_id)
        {
            if (id == publish_frame.can_id)
            {
                incoming_buffer.clear();
                for (int i = 0; i < publish_frame.can_dlc; i++)
                {
                    incoming_buffer.push_back(publish_frame.data[i]);
                }
                process_incoming_buffer(incoming_buffer, id);
            }
        }
        break;
    }
}

bool TF03::is_buffer_correct(std::vector<u_char> *data)
{
    if (data->at(0) == 0x59)
    {
        if (data->size() == 9 && data->at(1) == 0x59)
        {
            if (verify_checksum(*data))
            {
                return true;
            }
            // else
            // {
            //     ROS_WARN("Checksum not correct");
            // }
        }
        else if (data->size() > 9)
        {
            clear_incoming_buffer();
        }
    }
    else if (data->at(0) == 0x5a)
    {
        if (data->size() >= 2)
        {
            if (data->size() == data->at(1))
            {
                return true;
            }
            else if (data->size() > data->at(1))
            {
                clear_incoming_buffer();
            }
        }
    }
    else
    {
        // First byte is not equal to frame begin
        // need to remove it
        data->erase(data->begin());
    }
    return false;
}

void TF03::clear_incoming_buffer()
{
    incoming_buffer.clear();
}

void TF03::process_incoming_buffer(std::vector<u_char> data, int can_id)
{
    if (data.size() == 6 && data[4] == 0 && data[5] == 0 && !reconfigure_sensor)
    {
        // Distance measurement found
        u_char dist_low_byte = data[0];  // DIST_LOW
        u_char dist_high_byte = data[1]; // DIST_HIGH
        u_int dist = dist_high_byte * 256 + dist_low_byte;
        u_char strength_low_byte = data[2];  // STRENGTH_LOW
        u_char strength_high_byte = data[3]; // STRENGTH_HIGH
        u_int strength = strength_high_byte * 256 + strength_low_byte;
        float dist_meters = (float)dist / 100;
        sensor_data.at(can_id).range = dist_meters;
        //sensor_data.at(can_id).strength = strength;  // TODO: add "strength" to /tf03_driver/sensor/* ROS topics
        sensor_pub.at(can_id).publish(sensor_data.at(can_id));
    }
    else if (verify_checksum(data))
    {
        // Data frame structure
        // Head Len ID Payload Check sum
        if (data[2] == tf_03_command_id::version) // Command: Version nnumber
        {
            // Data is ordered as
            // 5A 07 01 V1 V2 V3 SU
            ROS_INFO("Version is %d.%d.%d", (uint16_t)data[5], (uint16_t)data[4], (uint16_t)data[3]);
            if (parameters[0].command == tf_03_command_id::version)
            {
                parameters[0].command_success = true;
            }
            else
            {
                ROS_WARN("Expected confirmation for %#2x, received %#2x instead", tf_03_command_id::version, parameters[0].command);
            }
        }
        else if (data[2] == tf_03_command_id::system_reset) // Command: reset
        {
            // Response is
            // 5A 05 02 00 61
            ROS_INFO("Reset staus is %d", (uint16_t)data[3]);
            if (parameters[0].command == tf_03_command_id::system_reset)
            {
                parameters[0].command_success = true;
            }
            else
            {
                ROS_WARN("Expected confirmation for %#2x, received %#2x instead", tf_03_command_id::system_reset, parameters[0].command);
            }
        }
        else if (data[2] == tf_03_command_id::output_format)
        {
            // Response is:
            // 5A 05 45 00 A4
            ROS_INFO("Output format status is %d", (uint16_t)data[3]);
            if (parameters[0].command == tf_03_command_id::output_format)
            {
                parameters[0].command_success = true;
            }
            else
            {
                ROS_WARN("Expected confirmation for %#2x, received %#2x instead", tf_03_command_id::output_format, parameters[0].command);
            }
        }
        else if (data[2] == tf_03_command_id::transmit_can_id)
        {
            // Response is:
            // 5A 05 50 00 AF
            ROS_INFO("Transmit CAN ID status is %d", (uint16_t)data[3]);
            if (parameters[0].command == tf_03_command_id::transmit_can_id)
            {
                parameters[0].command_success = true;
            }
            else
            {
                ROS_WARN("Expected confirmation for %#2x, received %#2x instead", tf_03_command_id::transmit_can_id, parameters[0].command);
            }
        }
        else if (data[2] == tf_03_command_id::receive_can_id)
        {
            // Response is:
            // 5A 05 51 00 B0
            ROS_INFO("Receive CAN ID status is %d", (uint16_t)data[3]);
            if (parameters[0].command == tf_03_command_id::receive_can_id)
            {
                parameters[0].command_success = true;
            }
            else
            {
                ROS_WARN("Expected confirmation for %#2x, received %#2x instead", tf_03_command_id::receive_can_id, parameters[0].command);
            }
        }
        else if (data[2] == tf_03_command_id::save_settings)
        {
            // success:     5A 05 11 00 70
            // fail:        5A 05 11 ER SU
            ROS_INFO("Save settings status is %d", (uint16_t)data[3]);
            if (parameters[0].command == tf_03_command_id::save_settings)
            {
                parameters[0].command_success = true;
            }
            else
            {
                ROS_WARN("Expected confirmation for %#2x, received %#2x instead", tf_03_command_id::save_settings, parameters[0].command);
            }
        }
        else if (data[2] == tf_03_command_id::factory_settings)
        {
            // success      5A 05 10 00 6F
            // fail         5A 05 10 ER SU
            ROS_INFO("Factory reset status is %d", (uint16_t)data[3]);
            if (parameters[0].command == tf_03_command_id::factory_settings)
            {
                parameters[0].command_success = true;
            }
            else
            {
                ROS_WARN("Expected confirmation for %#2x, received %#2x instead", tf_03_command_id::factory_settings, parameters[0].command);
            }
        }
    }
    else
    {
        // ROS_INFO("Data not correct");
    }
}

int TF03::init_sensor(tf_03_interface i, std::string device_name)
{
    if (i == tf_03_interface::serial)
    {
        ROS_INFO("Create serial port object");
#ifdef LIBSERIAL_0_X
        ROS_INFO("Using LIBSERIAL_0_x");
        tf03_serial_port = new SerialPort(device_name);
        if (!tf03_serial_port->IsOpen())
        {
            ROS_INFO("Open serial port");
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
        ROS_INFO("Using LIBSERIAL_1_x");
        tf03_serial_port = new LibSerial::SerialPort(
            device_name,
            LibSerial::BaudRate::BAUD_115200,
            LibSerial::CharacterSize::CHAR_SIZE_8,
            LibSerial::FlowControl::FLOW_CONTROL_NONE,
            LibSerial::Parity::PARITY_NONE,
            LibSerial::StopBits::STOP_BITS_1);
        if (!tf03_serial_port->IsOpen())
        {
            ROS_INFO("Open serial port");
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
    }
    else
    {
        can_socket = 0;
        can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket == -1)
        {
            ROS_ERROR("Can not open CAN socket");
            return 1;
        }
        else
        {
            // ROS_INFO("CAN socket ID: %ld", can_socket);
        }
        struct sockaddr_can addr;
        struct ifreq ifr;
        strcpy(ifr.ifr_name, device_name.c_str());
        ioctl(can_socket, SIOCGIFINDEX, &ifr);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        bind(can_socket, (struct sockaddr *)&addr, sizeof(addr));
        // ROS_INFO("Init frame");
        frame = new struct can_frame;
        frame->can_id = can_receive_id;
        frame->__pad = 0;
        frame->can_dlc = 0;
        for (auto d : frame->data)
        {
            d = 0;
        }
    }
    return 0;
}

void TF03::send_command(tf_03_interface i, tf_03_command_id command_id, int64_t command_argument)
{
    std::vector<u_char> command = {0x5a};
    switch (command_id)
    {
    case tf_03_command_id::version:
        ROS_INFO("TF03::send_command: version");
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(i, command);
        break;
    case tf_03_command_id::system_reset:
        ROS_INFO("TF03::send_command: system_reset");
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(i, command);
        break;
    case tf_03_command_id::save_settings:
        ROS_INFO("TF03::send_command:save settings");
        // command: 5A 04 11 6F
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(i, command);
        break;
    case tf_03_command_id::output_format:
        ROS_INFO("TF03::send_command: output format");
        // Serial:  5A 05 45 01 A5
        // CAN:     5A 05 45 02 A6
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(command_argument);
        command.push_back(get_checksum(command));
        write_command_data(i, command);
        break;
    case tf_03_command_id::transmit_can_id:
    {
        ROS_INFO("TF03::send_command: transmit CAN ID");
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
        write_command_data(i, command);
    }
    break;
    case tf_03_command_id::receive_can_id:
    {
        ROS_INFO("TF03::send_command: receive CAN ID");
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
        write_command_data(i, command);
    }
    break;
    default:
        ROS_ERROR("TF03::send_command: COMMAND NOT KNOWN");

        break;
    }
}

void TF03::write_command_data(tf_03_interface i, std::vector<u_char> data)
{
    switch (i)
    {
    case tf_03_interface::serial:
        tf03_serial_port->Write(data);
        break;
    case tf_03_interface::can:
        frame->can_dlc = data.size();
        for (int cnt = 0; cnt < data.size(); cnt++)
        {
            frame->data[cnt] = data[cnt];
        }
        int nbytes = write(can_socket, frame, sizeof(struct can_frame));
        // ROS_INFO("Sent %d bytes", nbytes);
        break;
    }
}

bool TF03::verify_checksum(std::vector<u_char> data)
{
    // ROS_INFO("verify_checksum() %d", data[data.size()]);
    // if (data.size() == 7)
    // {
    // ROS_INFO("%d, %d, %d, %d, %d, %d, %d", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
    // }
    if (get_checksum(std::vector<u_char>(&data[0], &data[data.size() - 1])) == data[data.size() - 1])
    {
        // ROS_INFO("verify_checksum: True");
        return true;
    }
    // ROS_INFO("verify_checksum: False");
    return false;
}

u_char TF03::get_checksum(std::vector<u_char> data)
{
    // ROS_INFO("get_checksum()");
    u_char checksum = 0;
    for (auto c : data)
    {
        // ROS_INFO("Byte %d", (int)c);
        checksum += c;
    }
    // ROS_INFO("return checksum: %d", checksum);
    return checksum;
}

std::map<tf_03_command_id, u_char> TF03::tf_03_command_len = {
    {version, 4},
    {system_reset, 4},
    {framerate, 6},
    {factory_settings, 4},
    {save_settings, 4},
    {output_format, 5},
    {transmit_can_id, 0x08},
    {receive_can_id, 0x08}};