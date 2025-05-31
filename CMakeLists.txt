cmake_minimum_required(VERSION 3.8)
project(tools_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(odometry src/odometry.cpp)
add_executable(localization src/localization.cpp)
add_executable(test_tf_tree src/test_tf_tree.cpp)
add_executable(gps_interface src/gps_interface.cpp src/ublox_parser.cpp)
# add_executable(gps_test src/gps_test.cpp src/gps_interface.cpp src/ublox_parser.cpp)
#add_executable(imu src/imu_interface.cpp)
ament_target_dependencies(odometry
rclcpp
nav_msgs)

ament_target_dependencies(localization
rclcpp
tf2
tf2_ros
tf2_geometry_msgs
tf2_sensor_msgs
geometry_msgs
builtin_interfaces
nav_msgs
sensor_msgs)

ament_target_dependencies(test_tf_tree
rclcpp
tf2
tf2_ros
tf2_geometry_msgs
tf2_sensor_msgs
geometry_msgs
builtin_interfaces
nav_msgs
message_filters
sensor_msgs)

ament_target_dependencies(gps_interface
rclcpp
nav_msgs
sensor_msgs)

# ament_target_dependencies(gps_test
# rclcpp
# nav_msgs
# sensor_msgs)

#ament_target_dependencies(imu
#rclcpp
#nav_msgs)

target_include_directories(odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(test_tf_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(gps_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# target_include_directories(gps_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

#target_include_directories(imu PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

target_compile_features(odometry PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_compile_features(localization PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(test_tf_tree PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(gps_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_compile_features(gps_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#target_compile_features(imu PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
localization
#imu
# gps_test
test_tf_tree
gps_interface
odometry
    DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
