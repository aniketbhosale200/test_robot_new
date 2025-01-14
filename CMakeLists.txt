cmake_minimum_required(VERSION 3.8)
project(test_robot_new)

# Find the necessary ROS 2 packages
find_package(ament_cmake REQUIRED)

# Set C and C++ standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # Updated to C++17 for modern ROS2 compatibility
endif()

# Add compiler options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install directories if they exist
foreach(dir config launch meshes urdf)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(
      DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
    )
  endif()
endforeach()

# ament package declaration
ament_package()

