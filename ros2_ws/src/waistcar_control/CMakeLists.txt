cmake_minimum_required(VERSION 3.8)
project(waistcar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(waistcar_msgs REQUIRED)
# find_package(Boost REQUIRED)
# link_libraries(${Boost_LIBRARY_DIRS})



include_directories(
  include
  /usr/include/eigen3
  )
link_directories(lib)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(motor_control_node src/motor_control_node.cpp)
target_link_libraries(motor_control_node unitreeMotorSDK_Arm64)
ament_target_dependencies(
  motor_control_node
  "rclcpp"
  "waistcar_msgs"
)

add_executable(motor_command_node src/motor_command_node.cpp)
ament_target_dependencies(
  motor_command_node
  "rclcpp"
  "waistcar_msgs"
)

add_executable(imu_read_node src/imu_read_node.cpp)
ament_target_dependencies(
  imu_read_node
  "rclcpp"
  "waistcar_msgs"
)
add_executable(remoter_read_node src/remoter_read_node.cpp)
ament_target_dependencies(
  remoter_read_node
  "rclcpp"
  "waistcar_msgs"
)
add_executable(waistcar_control_node src/waistcar_control_node.cpp src/car_jump.cpp src/data_logger.cpp src/lqr_inst.cpp)
ament_target_dependencies(
  waistcar_control_node
  "rclcpp"
  "waistcar_msgs"
)

add_executable(lidar_serial_node src/lidar_serial_node.cpp)
ament_target_dependencies(lidar_serial_node rclcpp)


# target_link_libraries(imu_read_node ${Boost_LIBRARIES})

install(TARGETS
  motor_control_node
  motor_command_node
  imu_read_node
  remoter_read_node
  waistcar_control_node
  lidar_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)
# install(TARGETS 
#   unitreeMotorSDK_Linux64
#   LIBRARY DESTINATION lib)



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
