cmake_minimum_required(VERSION 3.8)
project(my_turtle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_turtle_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(turtle_controller SHARED src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp rclcpp_lifecycle rclcpp_action rclcpp_components turtlesim my_turtle_interfaces geometry_msgs)
rclcpp_components_register_nodes(turtle_controller "my_turtle_controller::TurtleController")

install(TARGETS
  turtle_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()
