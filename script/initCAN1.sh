#!/bin/bash


echo "Enabling CAN1"
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 up
#sudo ifconfig can1 txqueuelen 1000

#echo "can0 up. Dumping (ctrl+c to close):"
#candump -c -t z can1,080~111111 #Filter out 080 sync messages