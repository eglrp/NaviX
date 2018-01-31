#!/bin/bash

#source /opt/ros/kinetic/setup.sh
#python /home/wellcasa/catkin_ws/js/init.py 

basepath=$(cd `dirname $0`; pwd)
python $basepath/shut_down_all_node.py
