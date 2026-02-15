cmake_minimum_required(VERSION 3.8)
project(c_example_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables
add_executable(dummy_publisher src/dummy_publisher.cpp)
ament_target_dependencies(dummy_publisher rclcpp std_msgs)

add_executable(dummy_subscriber src/dummy_subscriber.cpp)
ament_target_dependencies(dummy_subscriber rclcpp std_msgs)

# Install executables
install(TARGETS
  dummy_publisher
  dummy_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory (if you create launch files later)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

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
