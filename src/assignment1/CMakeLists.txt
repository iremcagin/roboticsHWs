cmake_minimum_required(VERSION 3.5)
project(assignment1)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp Directory
include_directories(include)

# Install Python Modules
ament_python_install_package(${PROJECT_NAME})

# Install Python Executables
install(PROGRAMS
  scripts/cleaner.py
  scripts/tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
