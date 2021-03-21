cmake_minimum_required(VERSION 3.5)
project(phoenix)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(phoenix_msgs REQUIRED)

# Add include path
include_directories(${PROJECT_SOURCE_DIR}/include)

# adc3
add_executable(adc3 src/adc3_main.cpp src/i2c.cpp src/ads1015.cpp)
ament_target_dependencies(adc3 rclcpp std_msgs sensor_msgs)

# stream (standalone)
add_executable(stream src/stream_main.cpp src/uart.cpp src/avalon_st.cpp)
ament_target_dependencies(stream rclcpp sensor_msgs phoenix_msgs)

# stream (library)
#add_library(stream SHARED src/stream_main.cpp src/uart.cpp src/avalon_st.cpp)
#target_compile_definitions(stream PUBLIC PHOENIX_BUILD_AS_LIBRARY)
#ament_target_dependencies(stream rclcpp rclcpp_components sensor_msgs phoenix_msgs)
#rclcpp_components_register_nodes(stream "phoenix::StreamPublisherNode")

# command (standalone)
add_executable(command src/command_main.cpp src/gpio.cpp src/spi.cpp src/avalon_st.cpp src/avalon_mm.cpp src/epcq.cpp)
ament_target_dependencies(command rclcpp std_msgs phoenix_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  adc3
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  stream
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  command
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
