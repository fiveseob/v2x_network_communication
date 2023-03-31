#!/bin/bash
sleep 2s

trap /home/fiveseob/ros_ws/src/relaysw/scripts/stop.sh SIGINT
python3 /home/fiveseob/ros_ws/src/relaysw/scripts/v2x_intersections.py
