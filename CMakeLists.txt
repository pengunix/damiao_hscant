cmake_minimum_required(VERSION 3.23)
project(motor_ros)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

# 这个奇怪的变量居然在ROS库里也存在，打开就没办法找到共享库
# set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(sockcanpp_CONCEPT_SUPPORT OFF CACHE BOOL "" FORCE)
add_subdirectory(lib/libsockcanpp)

# Generate ROS2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Command.msg"
  "msg/State.msg"
  DEPENDENCIES std_msgs
)

add_definitions(-DUSE_ROS)

add_executable(motor_node src/motor_node.cpp src/motor.cpp)

target_include_directories(motor_node PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/lib/libsockcanpp/include
)

ament_target_dependencies(motor_node rclcpp std_msgs)

# Link generated message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(motor_node
  ${cpp_typesupport_target}
  sockcanpp
  yaml-cpp
  pthread
)

install(TARGETS motor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY conf/
  DESTINATION share/${PROJECT_NAME}/conf
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/setup_can.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
