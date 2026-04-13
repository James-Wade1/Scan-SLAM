#!/bin/bash

cd /home/jwade19/ros_ws

source /opt/ros/jazzy/setup.bash

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install

ln -sf build/scan_slam/compile_commands.json compile_commands.json

tail -f /dev/null