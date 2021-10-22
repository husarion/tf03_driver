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


