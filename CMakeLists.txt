cmake_minimum_required(VERSION 3.8)
project(tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(find_seat_action src/find_seat_action.cpp)
ament_target_dependencies(find_seat_action
 rclcpp std_srvs std_msgs)

install(TARGETS
  find_seat_action
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
