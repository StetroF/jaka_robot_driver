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

# 公共头文件路径
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/jaka_robot_driver
)

# 公共库路径
set(LIBRARY_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/out/shared  # libjakaAPI_2_3_0_7.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib         # libprotobuf.so 等
)
link_directories(${LIBRARY_DIRS})

# 公共链接库
set(COMMON_LIBS
  jakaAPI_2_3_0_7
  protobuf
  pthread
)

# 公共 ROS2 依赖
set(ROS2_DEPS
  rclcpp
  std_msgs
  jaka_robot_interfaces
  std_srvs
)

# 定义函数：添加可执行文件并配置
function(add_jaka_executable target_name source_file)
  add_executable(${target_name} ${source_file})
  target_link_libraries(${target_name} ${COMMON_LIBS})
  set_target_properties(${target_name} PROPERTIES
    BUILD_RPATH "${LIBRARY_DIRS}"
    INSTALL_RPATH "${LIBRARY_DIRS}"
  )
  ament_target_dependencies(${target_name} ${ROS2_DEPS})
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endfunction()

# 添加节点
add_jaka_executable(jaka_driver_node src/jaka_driver_node.cpp)
add_jaka_executable(test_servo_move src/test_servo_move.cpp)
# 安装头文件
install(DIRECTORY include/ DESTINATION include/)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
