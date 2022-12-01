cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(service src/service.cpp)
ament_target_dependencies(service rclcpp std_msgs example_interfaces)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs tf2 tf2_ros example_interfaces)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs example_interfaces)

install(TARGETS
  talker
  listener
  service
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  # the following lines skip linters
  #set(ament_cmake_cppcheck_FOUND TRUE)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #set(ament_cmake_flake8_FOUND TRUE)
  #set(ament_cmake_uncrustify_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  # Run all lint tests in package.xml except those listed above
  #ament_lint_auto_find_test_dependencies()

  # Add a gtest executable
  set(TESTFILES
    test/basic_test.cpp
    test/main.cpp)
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs)

  install(TARGETS
  test_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
