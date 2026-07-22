cmake_minimum_required(VERSION 3.8)
project(phantom_panda_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(franka_msgs REQUIRED)

add_executable(phantom_panda_teleop_node src/phantom_panda_teleop_node.cpp)
add_executable(servo_trajectory_accumulator_node src/servo_trajectory_accumulator_node.cpp)
add_executable(sim_haptic_node src/sim_haptic_node.cpp)
add_library(realtime_servo_velocity_controller SHARED
  src/realtime_servo_velocity_controller.cpp)
target_include_directories(phantom_panda_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(phantom_panda_teleop_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  phantom_panda_teleop_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "trajectory_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "moveit_msgs"
  "control_msgs"
  "controller_manager_msgs"
  "franka_msgs"
)
target_link_libraries(phantom_panda_teleop_node Eigen3::Eigen)

ament_target_dependencies(
  sim_haptic_node
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
)

ament_target_dependencies(
  servo_trajectory_accumulator_node
  "rclcpp"
  "trajectory_msgs"
  "control_msgs"
)

target_include_directories(realtime_servo_velocity_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(realtime_servo_velocity_controller PUBLIC cxx_std_17)
ament_target_dependencies(
  realtime_servo_velocity_controller
  "controller_interface"
  "hardware_interface"
  "pluginlib"
  "realtime_tools"
  "rclcpp"
  "rclcpp_lifecycle"
  "trajectory_msgs"
  "control_msgs"
)

pluginlib_export_plugin_description_file(
  controller_interface phantom_panda_teleop_controllers.xml)

install(TARGETS
  phantom_panda_teleop_node
  servo_trajectory_accumulator_node
  sim_haptic_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS realtime_servo_velocity_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_rcm_geometry test/test_rcm_geometry.cpp)
  target_include_directories(test_rcm_geometry PUBLIC include)
  target_link_libraries(test_rcm_geometry Eigen3::Eigen)
endif()

ament_package()
