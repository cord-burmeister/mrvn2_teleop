cmake_minimum_required(VERSION 3.8)
project(mrvn2_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


find_package(Boost REQUIRED)

add_executable(mrvn2_teleop_key_node src/m2_teleop_key_node.cpp)
ament_target_dependencies(mrvn2_teleop_key_node rclcpp std_msgs geometry_msgs rcutils)
target_include_directories(mrvn2_teleop_key_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# include_directories(include ${Boost_INCLUDE_DIRS})
# target_link_libraries(m2_teleop_key_node Boost)


target_compile_features(mrvn2_teleop_key_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mrvn2_teleop_key_node
  DESTINATION lib/${PROJECT_NAME})

 install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

 install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
