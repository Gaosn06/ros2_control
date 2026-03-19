// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "forward_command_controller/forward_command_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace forward_command_controller
{
ForwardCommandController::ForwardCommandController() : ForwardControllersBase() {}

void ForwardCommandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());//创建参数监听器，监听控制器参数的变化 ParamListener 是一个辅助类，用于监听参数的变化并提供访问参数的接口 由 generate_parameter_library 生成
  /*
  **get_node()获取的节点名称是在controller_manager中定义的名称
  **ParamListener类在forward_command_controller.hpp中定义，由cmakelist里面的generate_parameter_library生成
  **ParamListener类的构造函数会在内部调用 declare_params() 方法，
  **声明控制器需要使用的参数，这些参数通常在控制器的 YAML 配置文件中定义，并从controller_manager的yaml文件中获取这些参数
  **在 configure 阶段会被读取和使用
  */
}
/*
**读取从yaml文件拿到的参数的变量名称存入 params_ 结构体中，由param_listener_赋值给params_
**根据 joints 和 interface_name 参数构建 command_interface_types_ 向量，
**command_interface_types_ 向量存储了控制器需要使用的接口类型，
**这些接口类型会在 controller_manager 中被用来获取对应的接口对象，
**并在 update() 循环中使用这些接口对象来发送控制命令
*/
controller_interface::CallbackReturn ForwardCommandController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interface_types_.clear();
  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/" + params_.interface_name);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::ForwardCommandController, controller_interface::ControllerInterface)
