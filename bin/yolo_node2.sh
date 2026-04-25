export ROS_DOMAIN_ID=5
source ../install/setup.bash
ros2 run yolo_node yolo_node --ros-args -p robot_id:=robot_2 -r __node:=yolo_node_robot2


