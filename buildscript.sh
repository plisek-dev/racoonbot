#!/bin/bash
echo source /opt/ros/humble/setup.bash && colcon build  --symlink-install && find . -wholename './src/*.py'


