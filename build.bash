#!/bin/bash
source /opt/ros/humble/setup.bash

cd ~/vrx_ws && colcon build --merge-install
source ~/vrx_ws/install/setup.bash