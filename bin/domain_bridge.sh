#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run domain_bridge domain_bridge \
  ~/hospital-kym/src/hospital_control/config/bridge_config.yaml
