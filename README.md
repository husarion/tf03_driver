# tf03_driver

The tf03_driver provides ROS interface for **TF 03** range sensor produced by [Benewake](http://en.benewake.com/).

Node supports both serial and CAN interfaces. It also allows changing sensor configuration.

## First setup (Sensor in factory settings)

First setup is done only when the sensor is configured to factory settings.
In order to set up the sensors correctly you need to connect only one to your device.

Cloning repo:
```
git clone https://github.com/husarion/tf03_driver.git
```

Nex go to first setup example. Default can_send is 1 and receive is 17.
If you wish to modify it, you can do so in docker-compose.yml file.

```
cd tf03_driver/examples/first_setup
```

In order to run the setup, execute in first setup directory:

```
docker-compose up --build
```

## Sensor communication with can

**Important note:** You need to have can_utils package installed, instructions [here](#can-utils-instalation)

**After first setup!**
If you have changed can send or receive id make sure to modify it in docker-compose
```
cd repo-path/examples/one_sensor_with_can
./can_setup.sh 
docker-compose up --build
```

to check if the data is received:

```
rostopic echo /tf03_driver/sensor/front
```

### Parameters

- `sensor_interface`

   Allow to choose communication between serial or CAN. Possible values [`serial`, `can`], default is `serial`

- `serial_port`

   Used when `sensor_interface` is set to `serial`. Serial port where sensor is connected, default value is `/dev/ttyUSB0`

- `can_device`

   Used when `sensor_interface` is set to `can`. CAN device where sensor is connected, default value is `panther_can`

- `can_transmit_id`

   Used when `sensor_interface` is set to `can`. Array of CAN IDs defining which sensors are used, default is array of one element with value `3` (HEX `0x03`)

- `sensor_frame`

   Used when `sensor_interface` is set to `can`. Array of TF frame names for CAN IDs. It will be used only when Array size is equal to `can_transmit_id`, otherwise frame names will be created according to pattern: `sensor_at_CAN_ID_{can_transmit_id}`

- `can_receive_id`

   Used when `sensor_interface` is set to `can`. CAN ID which sensor is using to receive data, default value is `12291` (HEX `0x3003`)

Below parameters are used to reconfigure the sensor, they are used once, then application exits. Node will start in continuous mode only when these parameters are set to default values. Default values means that they are not saved to sensor.

- `print_version`

   Print the sensor version, default value false

- `set_transmit_can_id`

   Set new CAN ID value used by sensor to transmit data, default value `0`

- `set_receive_can_id`

   Set new CAN ID value used by sensor to receive data, default value `0`

- `set_output_format`

   Set interface to publish measurements and accept new configuration commands. Possible values are [`serial`, `can`], default is empty


### Published topics

Driver publishes sensor measurements on topic according to pattern: `/{node_name}/sensor/{sensor_frame}` with message type `sensor_msgs::Range`.
Node will create as many publishers as sensors defined in parameters. In serial mode, frame name is set to `tf03_sensor`.

## Can utils instalation

Simply run

```
sudo apt-get install can-utils
```
