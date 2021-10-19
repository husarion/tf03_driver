#include <TF03_CAN.h>

TF03_CAN::TF03_CAN(ros::NodeHandle nh): TF03_Base(nh)
{
    int sum=0;
    sum += node_handle.param<std::string>("can_device", can_device, "panther_can");

    int num_of_params = 1;
    if (sum != num_of_params)
    {
        ROS_WARN("j%d parameters loaded incorrectly", (num_of_params - sum));
    }

    sum=0;
    sum += node_handle.getParam("can_transmit_id", can_transmit_id);

    if (sum != num_of_params)
    {
        ROS_WARN("a%d parameters loaded incorrectly", (num_of_params - sum));
    }
    sum=0;
    sum += node_handle.getParam("sensor_frame", sensor_frame);

    if (sum != num_of_params)
    {
        ROS_WARN("j%d parameters loaded incorrectly", (num_of_params - sum));
    }
    sum=0;
    sum += node_handle.param<int>("can_receive_id", can_receive_id, 0x11); //HEX 0x11 = DEC 17


    if (sum != num_of_params)
    {
        ROS_WARN("b%d parameters loaded incorrectly", (num_of_params - sum));
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

    ROS_INFO("Initialize sensor with CAN interface");
    if (init_sensor(can_device) != 0)
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

TF03_CAN::~TF03_CAN()
{
    close(can_socket);
}

void TF03_CAN::process_sensor_data()
{
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
}

int TF03_CAN::init_sensor(std::string device_name)
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
    return 0;
}

void TF03_CAN::send_command(tf_03_command_id command_id, int64_t command_argument)
{
    std::vector<u_char> command = {0x5a};
    switch (command_id)
    {
    case tf_03_command_id::version:
        ROS_INFO("TF03_CAN::send_command: version");
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(command);
        break;
    case tf_03_command_id::system_reset:
        ROS_INFO("TF03_CAN::send_command: system_reset");
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(command);
        break;
    case tf_03_command_id::save_settings:
        ROS_INFO("TF03_CAN::send_command:save settings");
        // command: 5A 04 11 6F
        command.push_back(tf_03_command_len.at(command_id));
        command.push_back(command_id);
        command.push_back(get_checksum(command));
        write_command_data(command);
        break;
    case tf_03_command_id::output_format:
        ROS_INFO("TF03_CAN::send_command: output format");
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
        ROS_INFO("TF03_CAN::send_command: transmit CAN ID");
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
        ROS_INFO("TF03_CAN::send_command: receive CAN ID");
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
        ROS_ERROR("TF03_CAN::send_command: COMMAND NOT KNOWN");

        break;
    }
}

void TF03_CAN::write_command_data(std::vector<u_char> data)
{
    frame->can_dlc = data.size();
    for (int cnt = 0; cnt < data.size(); cnt++)
    {
        frame->data[cnt] = data[cnt];
    }
    int nbytes = write(can_socket, frame, sizeof(struct can_frame));
    // ROS_INFO("Sent %d bytes", nbytes);
}

std::map<tf_03_command_id, u_char> TF03_CAN::tf_03_command_len = {
    {version, 4},
    {system_reset, 4},
    {framerate, 6},
    {factory_settings, 4},
    {save_settings, 4},
    {output_format, 5},
    {transmit_can_id, 0x08},
    {receive_can_id, 0x08}};
