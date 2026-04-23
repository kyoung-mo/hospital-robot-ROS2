cmake_minimum_required(VERSION 3.8)
project(mini_project_hp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(Qt6_DIR "/user/lib/x86_64-linux-gnu/cmake/Qt6")
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

include_directories(src)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/mainwindow.cpp
  resources.qrc
  src/ros_thread.h
)

target_link_libraries(${PROJECT_NAME} Qt6::Widgets)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
