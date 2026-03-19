cmake_minimum_required(VERSION 3.8)
project(controller_communicate_type_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # msg 生成工具

# 2. 声明要编译的 msg 文件（核心）
set(msg_files
  "msg/JointStates.msg"  # 你的自定义 msg 文件路径
  # 可添加多个："msg/ControllerReference.msg"
)

# 3. 生成 msg 对应的 C++/Python 代码
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # 依赖的其他 msg 包（必须与 package.xml 对应）
  DEPENDENCIES std_msgs sensor_msgs
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
