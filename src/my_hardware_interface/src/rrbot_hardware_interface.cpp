// Copyright (c) 2022-2026, b»robotized group (template)
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

#include <limits>
#include <vector>

#include "my_hardware_interface/rrbot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_hardware_interface
{
  hardware_interface::CallbackReturn RRBotHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    // TODO(anyone): read parameters and initialize the hardware
    // hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    
    hw_positions_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_efforts_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    tp_.resize(1, std::numeric_limits<double>::quiet_NaN());
    tp1_.resize(1, std::numeric_limits<double>::quiet_NaN());
    tp2_.resize(1, std::numeric_limits<double>::quiet_NaN());
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
    // 开一个数组joint_states_，用来存储机器人关节的状态，并将其初始化为NaN 这个数组就模拟底层硬件
    // joint_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    joint_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    joint_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    joint_efforts_[0] = 0.3; // 模拟一个初始力矩值，方便调试
    joint_efforts_[1] = 0.6; // 模拟一个初始力矩值，方便调试
    tp_[0] = 0.2; // 模拟一个初始力矩值，方便调试
    tp1_[0] = 0.7; // 模拟一个初始力矩值，方便调试
    tp2_[0] = 0.9; // 模拟一个初始力矩值，方便调试
    return CallbackReturn::SUCCESS;
  }

  // std::vector<hardware_interface::StateInterface> RRBotHardwareInterface::export_state_interfaces()
  // {
  //   std::vector<hardware_interface::StateInterface> state_interfaces;
  //   for (size_t i = 0; i < info_.joints.size(); ++i)
  //   {
  //     //初始化关节状态为参数中定义的初始值 将配置文件 / 参数中的字符串型初始值转换为双精度浮点数
  //     hw_states_[i] = std::stod(info_.joints[i].state_interfaces[0].initial_value);

  //     state_interfaces.emplace_back(
  //       hardware_interface::StateInterface(
  //         // TODO(anyone): insert correct interfaces
  //         info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_states_[i]));
  //   }
  //   return state_interfaces;
  // }

  std::vector<hardware_interface::StateInterface> RRBotHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    /*********************关节状态接口***************************/
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      // 遍历当前关节的所有状态接口，按名称匹配初始值
      for (const auto &state_if : info_.joints[i].state_interfaces)
      {
        // 1. 处理位置（position）接口的初始值
        if (state_if.name == hardware_interface::HW_IF_POSITION)
        {
          // 将配置文件中字符串型初始值转为双精度浮点数，赋值给位置状态变量
          if (!state_if.initial_value.empty())
          {
            hw_positions_states_[i] = std::stod(state_if.initial_value);
          }
          else
          {
            // 配置无初始值时，默认设为 0.0（避免 NaN）
            hw_positions_states_[i] = 0.0;
          }
          // 导出位置状态接口
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.joints[i].name,
                  hardware_interface::HW_IF_POSITION,
                  &hw_positions_states_[i]));
        }

        // 2. 处理速度（velocity）接口的初始值
        else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
        {
          if (!state_if.initial_value.empty())
          {
            hw_velocities_states_[i] = std::stod(state_if.initial_value);
          }
          else
          {
            hw_velocities_states_[i] = 0.0; // 默认 0.0
          }
          // 导出速度状态接口
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.joints[i].name,
                  hardware_interface::HW_IF_VELOCITY,
                  &hw_velocities_states_[i]));
        }
        // 3. 处理速度（effort）接口的初始值
        else if (state_if.name == hardware_interface::HW_IF_EFFORT)
        {
          if (!state_if.initial_value.empty())
          {
            hw_efforts_states_[i] = std::stod(state_if.initial_value);
          }
          else
          {
            hw_efforts_states_[i] = 0.0; // 默认 0.0
          }
          // 导出力矩状态接口
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.joints[i].name,
                  hardware_interface::HW_IF_EFFORT,
                  &hw_efforts_states_[i]));
        }
      }
    }
    /*****************传感器状态接口***************************/
    for(size_t i = 0; i < info_.sensors.size(); ++i)
    {
      for(const auto &state_if : info_.sensors[i].state_interfaces)
      {
        if(state_if.name == "tp")
        {
          if (!state_if.initial_value.empty())
          {
            tp_[i] = std::stod(state_if.initial_value);
          }
          else
          {
            tp_[i] = 0.0; // 默认 0.0
          }
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.sensors[i].name,
                  "tp",
                  &tp_[i]));
        }
        if(state_if.name == "tp1")
        {
          if (!state_if.initial_value.empty())
          {
            tp1_[i] = std::stod(state_if.initial_value);
          }
          else
          {
            tp1_[i] = 0.0; // 默认 0.0
          }
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.sensors[i].name,
                  "tp1",
                  &tp1_[i]));
        }
        if(state_if.name == "tp2")
        {
          if (!state_if.initial_value.empty())
          {
            tp2_[i] = std::stod(state_if.initial_value);
          }
          else
          {
            tp2_[i] = 0.0; // 默认 0.0
          }
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.sensors[i].name,
                  "tp2",
                  &tp2_[i]));
        }
      }
    }
    return state_interfaces;
  }

  // std::vector<hardware_interface::CommandInterface> RRBotHardwareInterface::export_command_interfaces()
  // {
  //   std::vector<hardware_interface::CommandInterface> command_interfaces;
  //   for (size_t i = 0; i < info_.joints.size(); ++i)
  //   {
  //     command_interfaces.emplace_back(
  //       hardware_interface::CommandInterface(
  //         // TODO(anyone): insert correct interfaces
  //         // info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i])
  //         info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &hw_commands_[i])
  //       );
  //   }
  //   return command_interfaces;
  // }

  std::vector<hardware_interface::CommandInterface> RRBotHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      for(const auto &cmd_if : info_.joints[i].command_interfaces)
      {
        // 1. 导出位置命令接口
        if (cmd_if.name == hardware_interface::HW_IF_POSITION)
        {
          command_interfaces.emplace_back(
              hardware_interface::CommandInterface(
                  info_.joints[i].name,               // 关节名（如 joint1）
                  hardware_interface::HW_IF_POSITION, // 接口名：position
                  &hw_position_commands_[i]));       // 绑定位置命令变量
        }
        // 2. 导出速度命令接口
        else if (cmd_if.name == hardware_interface::HW_IF_VELOCITY)
        {
          command_interfaces.emplace_back(
              hardware_interface::CommandInterface(
                  info_.joints[i].name,               // 关节名（如 joint1）
                  hardware_interface::HW_IF_VELOCITY, // 接口名：velocity
                  &hw_velocity_commands_[i]));       // 绑定速度命令变量
        }
      }
    }
    return command_interfaces;
  }

  // hardware_interface::CallbackReturn RRBotHardwareInterface::on_activate(
  //   const rclcpp_lifecycle::State & /*previous_state*/)
  // {
  //   // TODO(anyone): prepare the robot to receive commands
  //   //将机器人状态和命令初始化为参数中定义的初始值
  //   hw_commands_ = hw_states_;
  //   joint_states_ = hw_states_;
  //   return CallbackReturn::SUCCESS;
  // }

  hardware_interface::CallbackReturn RRBotHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // 激活时将命令初始化为当前状态值，避免突变
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      hw_position_commands_[i] = hw_positions_states_[i];
      hw_velocity_commands_[i] = hw_velocities_states_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), "Hardware activated!");
    return CallbackReturn::SUCCESS;
  }

  // hardware_interface::CallbackReturn RRBotHardwareInterface::on_deactivate(
  //   const rclcpp_lifecycle::State & /*previous_state*/)
  // {
  //   // TODO(anyone): prepare the robot to stop receiving commands

  //   return CallbackReturn::SUCCESS;
  // }

  hardware_interface::CallbackReturn RRBotHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // 停用时清空命令
    std::fill(hw_position_commands_.begin(), hw_position_commands_.end(), 0.0);
    std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
    RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"), "Hardware deactivated!");
    return CallbackReturn::SUCCESS;
  }

  // hardware_interface::return_type RRBotHardwareInterface::read(
  //   const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  // {
  //   // TODO(anyone): read robot states
  //   hw_states_ = joint_states_;
  //   return hardware_interface::return_type::OK;
  // }

  // hardware_interface::return_type RRBotHardwareInterface::write(
  //   const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  // {
  //   // TODO(anyone): write robot's commands'
  //   //将机器人命令写入joint_states_中，并将其打印出来
  //   joint_states_ = hw_commands_;
  //   return hardware_interface::return_type::OK;
  // }

  hardware_interface::return_type RRBotHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // 模拟从硬件读取状态：将模拟关节状态同步到硬件层状态变量
    // 真实硬件中，这里应该从串口/总线读取编码器/速度计数据
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      hw_positions_states_[i]   = joint_positions_[i];
      hw_velocities_states_[i]  = joint_velocities_[i];
      hw_efforts_states_[i]     = joint_efforts_[i];
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RRBotHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // 模拟向硬件写入命令：将硬件层命令变量同步到模拟关节状态
    // 真实硬件中，这里应该将命令下发给电机驱动器
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      joint_positions_[i]   = hw_position_commands_[i];
      joint_velocities_[i]  = hw_velocity_commands_[i];
      // 可选：打印命令值，方便调试
      // RCLCPP_INFO(rclcpp::get_logger("RRBotHardwareInterface"),
      //             "Joint %d: pos_cmd=%.2f, vel_cmd=%.2f",
      //             i, hw_position_commands_[i], hw_velocity_commands_[i]);
    }
    return hardware_interface::return_type::OK;
  }

} // namespace my_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    my_hardware_interface::RRBotHardwareInterface, hardware_interface::SystemInterface)
