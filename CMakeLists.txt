cmake_minimum_required(VERSION 3.25)
project(jaka_robot_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Debug)

# 查找 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jaka_robot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
# 头文件路径（SDK头文件 + ROS2头）
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/jaka_robot_driver
)

# 库文件路径
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/out/shared # libjakaAPI_2_3_0_7.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib # libprotobuf.so 等
)

# 添加你的源文件（示例）
add_executable(jaka_driver_node
  src/jaka_driver_node.cpp
)


# 链接库
target_link_libraries(jaka_driver_node
  jakaAPI_2_3_0_7
  protobuf
  pthread
)



# 设置运行时库路径（否则运行时找不到 .so）
set_target_properties(jaka_driver_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib:${CMAKE_CURRENT_SOURCE_DIR}/out/shared"
  INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib:${CMAKE_CURRENT_SOURCE_DIR}/out/shared"
)


# 依赖 ROS2
ament_target_dependencies(jaka_driver_node
  rclcpp
  std_msgs
  jaka_robot_interfaces
  std_srvs
)



# 安装头文件（如果你要编写 wrapper）
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
