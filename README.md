# tf03_driver

The tf03_driver provides ROS interface for **TF 03** range sensor produced by [Benewake](http://en.benewake.com/).

Node supports both serial and CAN interfaces. It also allows changing sensor configuration.

### Parameters

- `sensor_interface`

   Allow to choose communication between serial or CAN. Possible values [`serial`, `can`], default is `serial`

- `serial_port`

   Used when `sensor_interface` is set to `serial`. Serial port where sensor is connected, default value is `/dev/ttyUSB0`

- `can_device`

   Used when `sensor_interface` is set to `can`. CAN device where sensor is connected, default value is `panther_can`

- `can_transmit_id`

   Used when `sensor_interface` is set to `can`. CAN ID which sensor is using to transmit data, default value is `3` (HEX `0x03`)

- `can_receive_id`

   Used when `sensor_interface` is set to `can`. CAN ID which sensor is using to receive data, default value is `12291` (HEX `0x3003`)

Below parameters are used to reconfigure the sensor, they are used once, then application exits. Node will start in continuous mode only when these parameters are set to default values. Default values means that they are not saved to sensor.

- `print_version`

   Print the sensor version, default value false

- `set_transmit_can_id`

   Set new CAN ID value used by sensor to transmit data, default value `0`

- `set_receive_can_id`

   Set new CAN ID value used by sensor to receive data, default value `0`