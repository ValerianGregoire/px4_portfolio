cmake_minimum_required(VERSION 3.8)
project(missions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add include
# include_directories(include)
add_library(pid_lib src/pid_.cpp)
target_include_directories(pid_lib 
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# Add executables
add_executable(controller src/controller.cpp)
target_link_libraries(controller pid_lib)
ament_target_dependencies(controller rclcpp aruco_opencv_msgs px4_msgs std_msgs)

add_executable(line_detector src/line_detector.cpp)
target_include_directories(line_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(line_detector rclcpp cv_bridge OpenCV geometry_msgs sensor_msgs std_msgs)
target_link_libraries(line_detector 
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# Install targets
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  controller
  line_detector
  pid_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
