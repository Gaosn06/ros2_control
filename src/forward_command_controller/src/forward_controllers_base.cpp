// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "forward_command_controller/forward_controllers_base.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace forward_command_controller
{
ForwardControllersBase::ForwardControllersBase()
: controller_interface::ControllerInterface(),//初始化 ROS2 控制器接口，创建 controller 生命周期结构
  rt_command_ptr_(nullptr),//存储实时控制命令
  joints_command_subscriber_(nullptr)//订阅器，用于接收控制命令
{
}

/*
controller_manager load
        ↓
on_init()
        ↓
on_configure()
        ↓
on_activate()
        ↓
update() 循环运行
*/ 
//controller_interface::CallbackReturn 这个是一个 枚举类型，表示执行结果 如果返回 ERROR：控制器 不会继续进入 configure 阶段
//
controller_interface::CallbackReturn ForwardControllersBase::on_init()//控制器初始化回调函数，在控制器被加载时调用
{
  try
  {
    // 调用 declare_parameters() 方法，声明控制器需要使用的参数，这些参数通常在控制器的 YAML 配置文件中定义，并且在 configure 阶段会被读取和使用
    declare_parameters();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
//
controller_interface::CallbackReturn ForwardControllersBase::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }
  // 创建订阅器，订阅 "~/commands" 话题，接收控制命令，并将命令写入实时缓冲区 rt_command_ptr_ 中，
  // 以供 update() 循环使用
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

/* 
** 这个函数定义了控制器需要使用的命令接口和状态接口的配置，
** 在这个函数中，控制器指定了它需要使用的命令接口类型，
** 这些接口类型会在 controller_manager 中被用来获取对应的接口对象，
** 并在 update() 循环中使用这些接口对象来发送控制命令。
*/
controller_interface::InterfaceConfiguration
ForwardControllersBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

// 这个函数定义了控制器需要使用的状态接口的配置，当前控制器不需要使用任何状态接口，因此返回 NONE
controller_interface::InterfaceConfiguration ForwardControllersBase::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ForwardControllersBase::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
    // controller_interface::get_ordered_interfaces() 这个函数会根据 command_interface_types_ 中定义的接口类型，从 controller_manager 中获取对应的接口，并按照顺序存储在 ordered_interfaces 中
  if (
    //将hardware_interface返回出来的command_interfaces_接口对象按照command_interface_types_的顺序存储在ordered_interfaces中，如果command_interface_types_中的接口类型在controller_manager中没有对应的接口对象，或者ordered_interfaces中的接口对象数量与command_interface_types_中的接口类型数量不匹配，就会返回错误
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) || //command_interfaces_的类型是 std::vector<hardware_interface::LoanedCommandInterface>，存储了控制器需要使用的接口对象LoanedCommandInterface继承自 hardware_interface::CommandInterface，CommandInterface 是一个抽象类，将数据给到接口对象，接口对象会将数据传递给底层的硬件资源 
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardControllersBase::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

// 这个函数是控制器的核心，在 update() 循环中被调用，用于获取最新的控制命令，并将其应用到硬件接口上，实现对机器人的控制
controller_interface::return_type ForwardControllersBase::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace forward_command_controller

/*
command_interfaces_ ：在 ForwardControllersBase 类中，command_interfaces_ 是一个 std::vector<hardware_interface::LoanedCommandInterface> 类型的成员变量，
用于存储控制器需要使用的接口对象 LoanedCommandInterface。LoanedCommandInterface 继承自 hardware_interface::CommandInterface，
CommandInterface 是一个抽象类，定义了控制器与底层硬件资源之间的接口。通过 command_interfaces_，控制器可以将命令数据传递给对应的硬件资源，实现对机器人的控制。
将命令数据写入 command_interfaces_ 中的接口对象，接口对象会将数据传递给底层的硬件资源，实现对机器人的控制。这是控制器与硬件资源之间的关键连接点，使得控制器能够通过接口对象与底层硬件进行通信和控制。
*/

/*
rt_command_ptr_ ：在 ForwardControllersBase 类中，rt_command_ptr_ 是一个 realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> 类型的成员变量，
用于存储实时控制命令。RealtimeBuffer 是一个线程安全的缓冲区，允许在非实时线程中写入数据，并在实时线程中读取数据。通过 rt_command_ptr_，
控制器可以在 update() 循环中获取最新的控制命令，并将其应用到硬件接口上。
将命令数据从订阅回调函数写入 rt_command_ptr_，使得 update() 循环能够在每次执行时获取到最新的命令数据，实现对机器人的实时控制。
*/  