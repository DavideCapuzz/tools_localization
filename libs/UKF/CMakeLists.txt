cmake_minimum_required(VERSION 3.12)
project(UKF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(UKF STATIC
        src/IMU.cpp
        src/UKF.cpp
)

target_link_libraries(UKF PRIVATE Eigen3::Eigen)

target_include_directories(UKF PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
