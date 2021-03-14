cmake_minimum_required(VERSION 3.5)
project(phoenix_gui)

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

# Set QT5_DIR enviroment variable to install directory of Qt5
# for example : C:\dev\Qt\5.15.2\msvc2019_64

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# add include path
#include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(phoenix_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
find_package(OpenGL)

# gui
#qt5_wrap_ui(UI_HEADERS src/main_window.ui)
add_executable(gui
  src/main.cpp
  src/main_window.cpp
  src/node_thread.cpp
  src/image_viewer.cpp
  src/gamepad_thread.cpp
  src/cintelhex/ihex_copy.c
  src/cintelhex/ihex_parse.c
  src/cintelhex/ihex_record.c
)
ament_target_dependencies(gui
  rclcpp
  std_msgs
  sensor_msgs
  phoenix_msgs
)
target_link_libraries(gui
  Qt5::Widgets
  Qt5::OpenGL
  ${OPENGL_LIBRARIES}
)

# install
install(TARGETS gui DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
