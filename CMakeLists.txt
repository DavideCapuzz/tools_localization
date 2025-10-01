cmake_minimum_required(VERSION 3.12)
project(tools_localization)

# ------------------------------
# Compiler warnings / features
# ------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------
# Submodules / local libs
# ------------------------------
add_subdirectory(libs/UKF)

# ------------------------------
# ROS2 dependencies
# ------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)

# ------------------------------
# Other dependencies
# ------------------------------
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)
find_package(mcap REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# =====================================================
# =============== Library (implementation) ============
# =====================================================
add_library(localization_node_lib
        src/localization_node.cpp
)

target_include_directories(localization_node_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tools_localization/libraries>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tools_localization/filters>
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(localization_node_lib
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

target_link_libraries(localization_node_lib
        UKF
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

target_compile_features(localization_node_lib PUBLIC cxx_std_17)

# =====================================================
# ================= Node Executable ===================
# =====================================================
add_executable(localization
        src/localization_main.cpp   # <-- only main() lives here
)

target_link_libraries(localization
        localization_node_lib
)

# =====================================================
# ================ GPS Interface Node =================
# =====================================================
add_executable(gps_interface
        src/interfaces/gps_interface.cpp
        src/interfaces/ublox_parser.cpp
)

ament_target_dependencies(gps_interface
        rclcpp
        nav_msgs
        sensor_msgs
)

target_link_libraries(gps_interface
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

target_include_directories(gps_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(gps_interface PUBLIC cxx_std_17)

# =====================================================
# ==================== Tests ==========================
# =====================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_localization_node
          test/gtest/test_localization_node.cpp
  )

  target_link_libraries(test_localization_node
          localization_node_lib
          mcap
  )

  ament_target_dependencies(test_localization_node
          rclcpp
          tf2
          tf2_ros
          geometry_msgs
          sensor_msgs
          nav_msgs
          rosbag2_cpp
  )
endif()

# =====================================================
# ================== Install ==========================
# =====================================================
install(TARGETS
        localization
        gps_interface
        localization_node_lib
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

ament_package()
