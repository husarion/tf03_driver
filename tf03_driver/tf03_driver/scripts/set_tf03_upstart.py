#!/usr/bin/python3

import sys
import subprocess
import os

possible_arg_count = [2, 5]
if (len(sys.argv) not in possible_arg_count) or len(sys.argv) == 1:
    sys.exit("""
    ###Mismatch argument count. Expected 4.###

This script set up autostart of tf03 over can sensors
USAGE: 
    sudo python3 set_tf03_upstart.py <username> <local_ip_addr> <ros_master_ip/ros_master_hostname>
    
Example for default panther configuration: 
    sudo python3 set_tf03_upstart.py husarion 10.15.20.3 10.15.20.3 four_sensors

Uninstall: 
    sudo python3 set_tf03_upstart.py uninstall
""")


def prompt_sudo():
    ret = 0
    if os.geteuid() != 0:
        msg = "[sudo] password for %u:"
        ret = subprocess.check_call("sudo -v -p '%s'" % msg, shell=True)
    return ret


if prompt_sudo() != 0:
    sys.exit("The user wasn't authenticated as a sudouser, exiting")

if str(sys.argv[1]) == "uninstall":
    subprocess.call("rm /usr/sbin/can_setup.sh", shell=True)
    subprocess.call("rm /usr/sbin/tf03_script.sh", shell=True)
    subprocess.call("rm /etc/ros/env.sh", shell=True)
    subprocess.call("rm /etc/systemd/system/tf03_can.service", shell=True)
    subprocess.call(
        "rm /etc/systemd/system/tf03_sensors.service", shell=True)
    subprocess.call("systemctl disable tf03_can.service", shell=True)
    subprocess.call("systemctl disable tf03_sensors.service", shell=True)
    sys.exit("All removed")

HOSTNAME = str(sys.argv[1])
ROS_IP = str(sys.argv[2])
ROS_MASTER_URI = str(sys.argv[3])
SENSOR_LAUNCH = str(sys.argv[4])

launches = ["four_sensors", "single_sensor", "two_sensors", "four_sensors_rx_9"]
if SENSOR_LAUNCH not in launches:
    sys.exit("""
Wrong panther_type. Expected values:
    "four_sensors", "single_sensor", "two_sensors", "four_sensors_rx_9"
    """)

print("Configuration ->", "Hostname:", HOSTNAME, "ROS_IP:", ROS_IP,
      "ROS_MASTER_URI:", ROS_MASTER_URI,"SENSOR_LAUNCH:", SENSOR_LAUNCH)
subprocess.call("mkdir /etc/ros", shell=True)


#
# /etc/ros/env.sh
#

env_msg = """#!/bin/sh
export ROS_IP={rip} 
export ROS_MASTER_URI=http://{rmu}:11311
""".format(rip=ROS_IP, rmu=ROS_MASTER_URI)

subprocess.Popen(['echo "{}" > /etc/ros/env.sh'.format(env_msg)],  shell=True)


#
# /etc/systemd/system/tf03_can.service
#

can_setup = """#!/bin/bash
modprobe can
modprobe can-raw
modprobe slcan
echo "slcan_attach..."
slcan_attach -o -s8 -n benewake_can /dev/ttyACM0
sleep 5
echo "slcand..."
slcand -o -f -s8 -F /dev/ttyACM0 benewake_can &
sleep 5
echo "ifconfig..."
ifconfig benewake_can up"""

can_service = """[Unit]
Description=Enable CAN port for Panther robot
After=syslog.target network.target multi-user.target nodm.service user@1000.service

[Service]
Type=simple
ExecStartPre=/bin/sleep 15
ExecStart=/usr/sbin/can_setup.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
"""

subprocess.Popen(
    ['echo "{}" > /usr/sbin/can_setup.sh'.format(can_setup)],  shell=True)
subprocess.call("touch /etc/systemd/system/tf03_can.service", shell=True)
subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/tf03_can.service'.format(can_service)],  shell=True)


#
# /usr/sbin/tf03_sensors.sh
#

tf03_script = """#!/bin/bash
source ~/husarion_ws/devel/setup.bash
source /etc/ros/env.sh
export ROS_HOME=$(echo ~{hn})/.ros
roslaunch tf03_driver {sl}.launch &
PID=$!
wait "$PID"
""".format(hn=HOSTNAME,sl=SENSOR_LAUNCH)

subprocess.Popen(
    ['echo "{}" > /usr/sbin/tf03_script.sh'.format(tf03_script)],  shell=True)


#
# tf03_service 
#

tf03_service = """[Unit]
Requires=tf03_can.service
PartOf=tf03_can.service
After=NetworkManager.service time-sync.target tf03_can.service
[Service]
Type=simple
User={hn}
ExecStart=/usr/sbin/tf03_script.sh
[Install]
WantedBy=multi-user.target
""".format(hn=HOSTNAME)

subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/tf03_sensors.service'.format(tf03_service)],  shell=True)


subprocess.call("systemctl enable tf03_can.service", shell=True)
subprocess.call("chmod +x /usr/sbin/can_setup.sh", shell=True)
subprocess.call("systemctl enable tf03_sensors.service", shell=True)
subprocess.call("chmod +x /usr/sbin/tf03_script.sh", shell=True)


print("Done!")
