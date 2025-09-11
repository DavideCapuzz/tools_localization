cmake_minimum_required(VERSION 3.12)
project(tools_localization)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bring in the UKF submodule library (non-ROS2, local)
add_subdirectory(libs/UKF)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)

# ukf eigen and and json dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED) 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ---- Nodes ----
add_executable(localization
        src/localization_node.cpp
)

add_executable(gps_interface
        src/interfaces/gps_interface.cpp
        src/interfaces/ublox_parser.cpp
)

# ---- Dependencies ----
ament_target_dependencies(localization
        rclcpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_sensor_msgs
        geometry_msgs
        builtin_interfaces
        nav_msgs
        sensor_msgs
)

target_link_libraries(localization
        UKF   # link the pure library
)

ament_target_dependencies(gps_interface
rclcpp
nav_msgs
sensor_msgs)

# Eigen and JSON dep
target_link_libraries(localization Eigen3::Eigen nlohmann_json::nlohmann_json)
target_link_libraries(gps_interface Eigen3::Eigen nlohmann_json::nlohmann_json)

# ament_target_dependencies(gps_test
# rclcpp
# nav_msgs
# sensor_msgs)

#ament_target_dependencies(imu
#rclcpp
#nav_msgs)

# ---- Include paths ----
target_include_directories(localization PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tools_localization/libraries>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tools_localization/filters>
        ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(gps_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# target_include_directories(gps_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ---- Compile features ----
target_compile_features(localization PUBLIC c_std_99 cxx_std_17)
target_compile_features(gps_interface PUBLIC c_std_99 cxx_std_17)

# ---- Install ----
install(TARGETS
        localization
        gps_interface
        DESTINATION lib/${PROJECT_NAME})

# (optional) install scripts
install(PROGRAMS
        DESTINATION lib/${PROJECT_NAME}
)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
