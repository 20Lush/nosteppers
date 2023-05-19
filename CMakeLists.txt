cmake_minimum_required(VERSION 3.5)
project(nosteppers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(JetsonGPIO REQUIRED)

include_directories(include/nosteppers/)

add_executable(dynamixel_sync_read_write src/dynamixel_sync_read_write.cpp)
ament_target_dependencies(dynamixel_sync_read_write 
    dynamixel_sdk_custom_interfaces
    dynamixel_sdk
    rclcpp
)

add_executable(dynamixel_srw_tester src/dynamixel_srw_tester.cpp)
ament_target_dependencies(dynamixel_srw_tester
    dynamixel_sdk_custom_interfaces
    dynamixel_sdk
    rclcpp
)

add_executable(joint_server src/joint_server.cpp)
ament_target_dependencies(joint_server
    dynamixel_sdk_custom_interfaces
    std_msgs
    rclcpp
    JetsonGPIO
)

add_executable(joint_planned_path src/joint_planned_path.cpp)
ament_target_dependencies(joint_planned_path
    std_msgs
    rclcpp
)

add_executable(target_loader src/target_loader.cpp)
ament_target_dependencies(target_loader
    std_msgs
    rclcpp
)


install(TARGETS 
  dynamixel_sync_read_write
  dynamixel_srw_tester
  joint_server
  joint_planned_path
  target_loader
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
