
cmake_minimum_required(VERSION 3.16)
project(my_position_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)  # 必须在 generate_parameter_library 命令前
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_communicate_type_pkg REQUIRED)  # 自定义接口类型包

# Add my_position_controller library related compile commands
generate_parameter_library(my_position_controller_parameters
  src/my_position_controller.yaml
  include/my_position_controller/validate_my_position_controller_parameters.hpp
)
add_library(
  my_position_controller
  SHARED
  src/my_position_controller.cpp
)
target_include_directories(my_position_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(my_position_controller my_position_controller_parameters)
# ament_target_dependencies(my_position_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(my_position_controller 
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  std_srvs
  controller_communicate_type_pkg
)
target_compile_definitions(my_position_controller PRIVATE "MY_POSITION_CONTROLLER_BUILDING_DLL")

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)

#   ament_add_gmock(test_load_my_position_controller test/test_load_my_position_controller.cpp)
#   target_include_directories(test_load_my_position_controller PRIVATE include)
#   ament_target_dependencies(
#     test_load_my_position_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   add_rostest_with_parameters_gmock(test_my_position_controller test/test_my_position_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/my_position_controller_params.yaml)
#   target_include_directories(test_my_position_controller PRIVATE include)
#   target_link_libraries(test_my_position_controller my_position_controller)
#   ament_target_dependencies(
#     test_my_position_controller
#     controller_interface
#     hardware_interface
#   )

#   add_rostest_with_parameters_gmock(test_my_position_controller_preceeding test/test_my_position_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/my_position_controller_preceeding_params.yaml)
#   target_include_directories(test_my_position_controller_preceeding PRIVATE include)
#   target_link_libraries(test_my_position_controller_preceeding my_position_controller)
#   ament_target_dependencies(
#     test_my_position_controller_preceeding
#     controller_interface
#     hardware_interface
#   )
# endif()
ament_export_libraries(
  my_position_controller
)

install(
  TARGETS my_position_controller
  EXPORT export_my_position_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(
  controller_interface
  my_position_controller.xml
)
ament_package()
