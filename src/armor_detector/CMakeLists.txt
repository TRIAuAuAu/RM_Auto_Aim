cmake_minimum_required(VERSION 3.10)
project(armor_detector)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

#######################
## Find dependencies ##
#######################
# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 查找 ROS 2 核心依赖
find_package(ament_cmake_auto REQUIRED)
# 查找 fmt 库
find_package(fmt REQUIRED)
# 查找 auto_aim_interfaces 包

# 找到 auto_aim_interfaces 包的路径:
# 自定义工作空间路径变量
set(WORKSPACE_DIR "/home/auauau/RM_Auto_Aim")
# 构建 auto_aim_interfaces_DIR 路径
set(auto_aim_interfaces_DIR "${WORKSPACE_DIR}/install/auto_aim_interfaces/share/auto_aim_interfaces/cmake")
find_package(auto_aim_interfaces REQUIRED)




ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${auto_aim_interfaces_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)
# 添加可执行文件
add_executable(armor_detector_test
    test/test.cpp
)

# 链接动态库到可执行文件
target_link_libraries(armor_detector_test
    ${PROJECT_NAME}
    fmt::fmt
)
# 链接 ROS 2 相关库到可执行文件
ament_target_dependencies(armor_detector_test
    rclcpp
    geometry_msgs
    auto_aim_interfaces
)
# 链接 ROS 2 相关库到动态库
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    auto_aim_interfaces
)
# 安装可执行文件
install(TARGETS armor_detector_test
  DESTINATION lib/${PROJECT_NAME}
)


# 添加测试
enable_testing()
add_test(NAME DetectorNumberClassifierTest COMMAND armor_detector_test)

# 导出包
ament_auto_package(
  INSTALL_TO_SHARE
  model
)
