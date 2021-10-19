#!/bin/bash
sudo modprobe can
sudo modprobe can-raw
sudo modprobe slcan
echo "slcan_attach..."
sudo slcan_attach -o -s8 -n benewake_can /dev/ttyACM0
sleep 5
echo "slcand..."
sudo slcand -o -f -s8 -F /dev/ttyACM0 benewake_can &
sleep 5
echo "ifconfig..."
sudo ifconfig benewake_can up
