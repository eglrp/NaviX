#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo ""

echo "copy rules in udev"
sudo cp ./96-hokuyo.rules /etc/udev/rules.d/
sudo cp ./99-BCB.rules /etc/udev/rules.d/
sudo cp ./99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger