#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run domain_bridge domain_bridge \
  ~/hospital-robot-ROS2/src/hospital_control/config/bridge_config.yaml
