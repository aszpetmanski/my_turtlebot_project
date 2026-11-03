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
find_package(turtlesim REQUIRED)
find_package(my_turtle_interfaces REQUIRED)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp rclcpp_lifecycle rclcpp_action turtlesim my_turtle_interfaces)

install(TARGETS
  
    turtle_controller
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
