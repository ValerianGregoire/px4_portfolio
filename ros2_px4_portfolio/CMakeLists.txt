cmake_minimum_required(VERSION 3.8)
project(ros2_px4_portfolio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Add include
include_directories(include)

add_library(pid_lib src/pid_.cpp)

target_include_directories(pid_lib 
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>") # /${PROJECT_NAME}
    
ament_target_dependencies(pid_lib px4_msgs rclcpp std_msgs)

# Add executables
add_executable(controller src/controller.cpp)

# Link libraries
target_link_libraries(controller pid_lib)

# Install targets
install(TARGETS
  controller
  pid_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
