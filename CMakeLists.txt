cmake_minimum_required(VERSION 3.8)
project(cpp_led)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(led_pub src/led_pub.cpp)
add_executable(led_sub src/led_sub.cpp)
add_executable(led_key src/led_key.cpp)

ament_target_dependencies(led_pub
  rclcpp
  std_msgs
)
ament_target_dependencies(led_sub
  rclcpp
  std_msgs
)
ament_target_dependencies(led_key
  rclcpp
  std_msgs
)
target_link_libraries(led_pub
  gpiod
)

install(TARGETS
  led_key
  led_pub
  led_sub
  DESTINATION lib/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
