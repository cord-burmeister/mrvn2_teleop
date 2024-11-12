cmake_minimum_required(VERSION 3.8)
project(mrvn2_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Add compile options to enable warnings
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED)

# Check for missing dependencies
if(NOT ament_cmake_FOUND)
  message(FATAL_ERROR "ament_cmake not found")
endif()
if(NOT rclcpp_FOUND)
  message(FATAL_ERROR "rclcpp not found")
endif()
if(NOT rcutils_FOUND)
  message(FATAL_ERROR "rcutils not found")
endif()
if(NOT std_msgs_FOUND)
  message(FATAL_ERROR "std_msgs not found")
endif()
if(NOT geometry_msgs_FOUND)
  message(FATAL_ERROR "geometry_msgs not found")
endif()
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Boost not found")
endif()

add_executable(mrvn2_teleop_key_node src/m2_teleop_key_node.cpp)
ament_target_dependencies(mrvn2_teleop_key_node rclcpp std_msgs geometry_msgs rcutils)
target_include_directories(mrvn2_teleop_key_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mrvn2_teleop_key_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mrvn2_teleop_key_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
