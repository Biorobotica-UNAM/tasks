cmake_minimum_required(VERSION 3.8)
project(tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Busca las dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(faces_interfaces REQUIRED)
find_package(take_interfaces REQUIRED)
find_package(manipulation_planner_interfaces REQUIRED)
find_package(manipulation_controller_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manipulation_tools REQUIRED)
find_package(human_interfaces REQUIRED)
find_package(tf2 REQUIRED)             
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(vision_tools REQUIRED)

add_executable(find_seat_node src/find_seat_node.cpp)
add_executable(find_person_node src/find_person_node.cpp)
add_executable(find_object_node src/find_object_node.cpp)
add_executable(take_node src/take_node.cpp)
add_executable(human_follower_node src/human_follower_node.cpp)
add_executable(find_waving_node src/find_waving_node.cpp)
add_executable(table_align_node src/table_align_node.cpp)

ament_target_dependencies(find_seat_node
 rclcpp std_srvs std_msgs vision_interfaces)

 ament_target_dependencies(find_person_node
 rclcpp std_srvs std_msgs vision_interfaces faces_interfaces)

ament_target_dependencies(find_object_node
 rclcpp std_srvs std_msgs vision_interfaces geometry_msgs)

ament_target_dependencies(take_node
 rclcpp std_srvs std_msgs geometry_msgs take_interfaces manipulation_planner_interfaces manipulation_controller_interfaces manipulation_tools)
ament_target_dependencies(human_follower_node
 rclcpp std_srvs std_msgs vision_interfaces geometry_msgs)

ament_target_dependencies(table_align_node
  rclcpp std_srvs geometry_msgs vision_tools)

ament_target_dependencies(find_waving_node
  rclcpp 
  std_srvs 
  std_msgs 
  human_interfaces 
  geometry_msgs
)

install(TARGETS
  find_seat_node
  find_person_node
  find_object_node
  take_node
  human_follower_node
  find_waving_node
  table_align_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
