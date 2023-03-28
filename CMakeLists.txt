cmake_minimum_required(VERSION 3.8)
project(lsm9ds1_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(lsm9ds1_handler SHARED
    src/lsm9ds1.cpp
    src/lsm9ds1_device.cpp
)

target_include_directories(lsm9ds1_handler
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lsm9ds1_handler rclcpp sensor_msgs)
target_link_libraries(lsm9ds1_handler i2c)

add_executable(lsm9ds1_node src/lsm9ds1_node.cpp)
target_link_libraries(lsm9ds1_node lsm9ds1_handler)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  lsm9ds1_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  lsm9ds1_handler
  DESTINATION lib
) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
