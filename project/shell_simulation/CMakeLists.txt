cmake_minimum_required(VERSION 3.8)
project(shell_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)


add_executable(example_control src/example_control.cpp)
ament_target_dependencies(example_control
  "rclcpp"
  "std_msgs"
)
target_include_directories(example_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install launch directory
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

# Install C++ nodes
install(
  TARGETS 
    example_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes
install(
  PROGRAMS
    scripts/example_control.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

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
