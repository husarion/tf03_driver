#include "TF03_Base.h"

void TF03_Base::clear_incoming_buffer()
{
    incoming_buffer.clear();
}


void TF03_Base::process_incoming_buffer(std::vector<u_char> data, int can_id)
{
    if (data.size() == 6 && data[4] == 0 && data[5] == 0)
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

bool TF03_Base::is_buffer_correct(std::vector<u_char> *data)
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

bool TF03_Base::verify_checksum(std::vector<u_char> data)
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

u_char TF03_Base::get_checksum(std::vector<u_char> data)
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