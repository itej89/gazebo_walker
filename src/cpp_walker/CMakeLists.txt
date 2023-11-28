cmake_minimum_required(VERSION 3.8)
project(cpp_walker)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_action
    sensor_msgs
    geometry_msgs
)

# Build service node.
add_executable(cpp_walker src/cpp_walker.cpp)
ament_target_dependencies(cpp_walker rclcpp sensor_msgs geometry_msgs)

# Install targets.
install(TARGETS
  cpp_walker
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch models world
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # un-comment the following lines skip linters (TRUE = skip that test)
  # set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  # Add a gtest executable
  set(TESTFILES
    test/basic_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
