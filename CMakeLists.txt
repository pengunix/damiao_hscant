cmake_minimum_required(VERSION 3.8)
project(damiao_hscant)
set(ENABLE_DAMIAO on)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DmCommand.msg"
  "msg/DmState.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

if(${ENABLE_DAMIAO})
  add_executable(dm_motor_ros src/dm_node.cpp src/damiao.cpp)
else()
  add_executable(dm_motor_ros src/rs_node.cpp src/damiao.cpp)
endif()

target_include_directories(dm_motor_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dm_motor_ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dm_motor_ros
  "rclcpp"
  "std_msgs"
)

rosidl_target_interfaces(dm_motor_ros ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS dm_motor_ros
  DESTINATION lib/${PROJECT_NAME})

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
