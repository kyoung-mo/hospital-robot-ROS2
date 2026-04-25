export ROS_DOMAIN_ID=5
source ../install/setup.bash
ros2 run yolo_node yolo_node --ros-args -p robot_id:=robot_1 -r __node:=yolo_node_robot1
