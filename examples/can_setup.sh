#!/bin/bash
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
ifconfig benewake_can up
