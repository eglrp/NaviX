#!/bin/bash

basepath=$(cd `dirname $0`; pwd)
python $basepath/shut_down_all_node.py 
roslaunch $basepath/start_scan_mode.launch &

