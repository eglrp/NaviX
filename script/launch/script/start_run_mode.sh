#!/bin/bash


basepath=$(cd `dirname $0`; pwd)
python $basepath/shut_down_all_node.py 
roslaunch $basepath/start_run_mode.launch &

