cmake_minimum_required(VERSION 3.12)
project(EKF_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(EKF_basic STATIC
        src/EKF_basic.cpp
)

target_link_libraries(EKF_basic PRIVATE Eigen3::Eigen)

target_include_directories(EKF_basic PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
