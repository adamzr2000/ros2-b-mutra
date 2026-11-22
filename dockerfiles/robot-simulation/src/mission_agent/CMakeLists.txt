cmake_minimum_required(VERSION 3.8)
project(mission_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Mission agent executable. All control logic lives in this binary's own .text
# (no behaviour-bearing shared library) so a single-byte runtime tamper of the
# tick loop is both measured by the attestation sidecar and operationally
# effective. Keep the frame pointer so the tampered hot-path offset stays
# stable and easy to resolve with objdump/nm.
add_executable(mission_agent src/mission_agent.cpp)
target_compile_options(mission_agent PRIVATE -fno-omit-frame-pointer)
ament_target_dependencies(mission_agent
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# Install executable
install(TARGETS
  mission_agent
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
