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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/b-robotized/ros_team_workspace) repository.
//

#include "my_position_controller/my_position_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{ // utility

  // TODO(destogl): remove this when merged upstream
  // Changed services history QoS to keep all so we don't lose any client service calls
  static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1, // message queue depth
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false};

  using ControllerReferenceMsg = my_position_controller::MyPositionController::ControllerReferenceMsg;

  // called from RT control loop
  void reset_controller_reference_msg(
      std::shared_ptr<ControllerReferenceMsg> &msg, const std::vector<std::string> &joint_names)
  {
    msg->joint_names = joint_names;
    msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN()); // 后面参数是给定displacements值为未初始化标记
    msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    msg->duration = std::numeric_limits<double>::quiet_NaN();
  }

} // namespace

namespace my_position_controller
{
  MyPositionController::MyPositionController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn MyPositionController::on_init()
  {
    control_mode_.initRT(control_mode_type::FAST);

    try
    {
      param_listener_ = std::make_shared<my_position_controller::ParamListener>(get_node());
      // RCLCPP_INFO(get_node()->get_logger(), "I am comming!!!!");
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MyPositionController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    params_ = param_listener_->get_params();

    if (!params_.state_joints.empty())
    {
      state_joints_ = params_.state_joints;
    }
    else
    {
      state_joints_ = params_.joints;
    }

    if (params_.joints.size() != state_joints_.size())
    {
      RCLCPP_FATAL(
          get_node()->get_logger(),
          "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
          params_.joints.size(), state_joints_.size());
      return CallbackReturn::FAILURE;
    }

    // topics QoS 话题通信质量相关参数配置
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
        "~/reference", subscribers_qos,
        std::bind(&MyPositionController::reference_callback, this, std::placeholders::_1));

    std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
    reset_controller_reference_msg(msg, params_.joints); // 初始化msg内容
    input_ref_.writeFromNonRT(msg);

    try
    {
      // State publisher
      s_publisher_ =
          get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
      state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  void MyPositionController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
  {
    if (msg->joint_names.size() == params_.joints.size())
    {
      input_ref_.writeFromNonRT(msg);
    }
    else
    {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Received %zu , but expected %zu joints in command. Ignoring message.",
          msg->joint_names.size(), params_.joints.size());
    }
  }

  controller_interface::InterfaceConfiguration MyPositionController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(params_.joints.size());
    for (const auto &joint : params_.joints)
    {
      for (const auto &command_interface : params_.command_interfaces)
      {
        command_interfaces_config.names.push_back(joint + "/" + command_interface);
      }

      // command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
    }

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration MyPositionController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // state_interfaces_config.names.reserve(state_joints_.size());
    state_interfaces_config.names.reserve(state_joints_.size() * params_.state_interfaces.size() + 
                                       params_.sensors.size() * params_.sensor_state_interfaces.size());
    for (const auto &joint : state_joints_)
    {
      for (const auto &state_interfaces : params_.state_interfaces)
      {
        state_interfaces_config.names.push_back(joint + "/" + state_interfaces);
      }
      // state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
    }

  // --------------------------
  // 2. 添加传感器的状态接口（增加空值检查）
  // --------------------------
  for(const auto &sensor : params_.sensors)
  {
    // 跳过空传感器名
    if (sensor.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "忽略空的传感器名");
      continue;
    }
    // 修正：循环变量名改为 sensor_state_interface（明确是传感器接口）
    for (const auto &sensor_state_interface : params_.sensor_state_interfaces)
    {
      // 跳过空接口名
      if (sensor_state_interface.empty()) {
        RCLCPP_WARN(get_node()->get_logger(), "忽略空的传感器状态接口名（传感器：%s）", sensor.c_str());
        continue;
      }
      state_interfaces_config.names.push_back(sensor + "/" + sensor_state_interface);
    }
  }

    return state_interfaces_config;
  }

  controller_interface::CallbackReturn MyPositionController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
    // `on_activate` method in `JointTrajectoryController` for exemplary use of
    // `controller_interface::get_ordered_interfaces` helper function

    // Set default value in command
    reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MyPositionController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
    // instead of a loop
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // controller_interface::return_type MyPositionController::update(
  //   const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
  // {
  //   auto current_ref = input_ref_.readFromRT();

  //   // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  //   // instead of a loop
  //   for (size_t i = 0; i < command_interfaces_.size(); ++i)
  //   {
  //     if (!std::isnan((*current_ref)->displacements[i]))
  //     {
  //       command_interfaces_[i].set_value((*current_ref)->displacements[i]);
  //       (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
  //     }
  //   }

  //   if (state_publisher_ && state_publisher_->trylock())
  //   {
  //     // TODO(anyone): depending on the message type, fill the state message with data from state interfaces
  //     state_publisher_->msg_.joint_names.clear();
  //     state_publisher_->msg_.displacements.clear();
  //     for(size_t i = 0; i < state_interfaces_.size(); ++i)
  //     {
  //       state_publisher_->msg_.joint_names.push_back(state_interfaces_[i].get_name());
  //       state_publisher_->msg_.displacements.push_back(state_interfaces_[i].get_value());
  //     }
  //     state_publisher_->msg_.header.stamp = time;
  //     state_publisher_->unlockAndPublish();
  //   }

  //   return controller_interface::return_type::OK;
  // }

  controller_interface::return_type MyPositionController::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    // ========== 1. 安全读取参考指令（双重检查 + 深度拷贝） ==========
    std::shared_ptr<ControllerReferenceMsg> ref_msg;
    {
      auto current_ref = input_ref_.readFromRT();
      // 双重空指针检查
      if (!current_ref || !(*current_ref))
      {
        RCLCPP_WARN(get_node()->get_logger(), "未接收到有效参考指令（空指针）");
        return controller_interface::return_type::OK;
      }
      // 深度拷贝，避免修改原始消息
      ref_msg = std::make_shared<ControllerReferenceMsg>(**current_ref);
    }

    // 基础长度校验
    if (ref_msg->joint_names.size() != params_.joints.size() ||
        ref_msg->displacements.size() != params_.joints.size() ||
        ref_msg->velocities.size() != params_.joints.size())
    {
      RCLCPP_WARN(get_node()->get_logger(),
                  "参考指令长度不匹配：关节数=%zu, 位置指令数=%zu, 速度指令数=%zu",
                  params_.joints.size(), ref_msg->displacements.size(), ref_msg->velocities.size());
      return controller_interface::return_type::OK;
    }

    // ========== 2. 指令透传到底层硬件接口（健壮的接口索引逻辑） ==========
    size_t iface_idx = 0;
    for (size_t joint_idx = 0; joint_idx < params_.joints.size(); ++joint_idx)
    {
      const std::string &joint_name = params_.joints[joint_idx];
      // 读取参考指令（保留NaN判断，与原逻辑一致）
      const double pos_cmd = ref_msg->displacements[joint_idx];
      const double vel_cmd = ref_msg->velocities[joint_idx];

      // 遍历该关节的所有命令接口（适配任意数量的接口配置）
      for (auto &iface_name : params_.command_interfaces)
      {
        // 越界检查（关键修复：>= 而非 +1 >=）
        if (iface_idx >= command_interfaces_.size())
        {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "关节%s的%s接口索引越界（总接口数：%zu）",
                       joint_name.c_str(), iface_name.c_str(), command_interfaces_.size());
          goto end_command_write; // 跳出双层循环，避免后续错误
        }

        // 核心修复：移除 const 限制 → 改为引用（auto&），允许修改接口值
        auto &cmd_iface = command_interfaces_[iface_idx];

        // 校验接口名称（避免写入错误接口）
        if (cmd_iface.get_interface_name() != iface_name)
        {
          RCLCPP_WARN(get_node()->get_logger(),
                      "关节%s的接口名称不匹配：预期=%s, 实际=%s",
                      joint_name.c_str(), iface_name.c_str(), cmd_iface.get_interface_name().c_str());
          iface_idx++;
          continue;
        }

        // 根据接口名称写入对应指令（透传逻辑）
        if (iface_name == "position" && !std::isnan(pos_cmd))
        {
          cmd_iface.set_value(pos_cmd);
          // 重置原始消息的NaN（避免重复下发）
          (*input_ref_.readFromRT())->displacements[joint_idx] = std::numeric_limits<double>::quiet_NaN();
        }
        else if (iface_name == "velocity" && !std::isnan(vel_cmd))
        {
          cmd_iface.set_value(vel_cmd);
          (*input_ref_.readFromRT())->velocities[joint_idx] = std::numeric_limits<double>::quiet_NaN();
        }

        RCLCPP_DEBUG(get_node()->get_logger(),
                     "关节%s的%s接口写入值：%.4f",
                     joint_name.c_str(), iface_name.c_str(),
                     (iface_name == "position" ? pos_cmd : vel_cmd));
        iface_idx++;
      }
    }
  end_command_write:

    // ========== 3. 安全发布状态（实时线程安全逻辑） ==========
    if (state_publisher_ && state_publisher_->trylock())
    {
      auto &state_msg = state_publisher_->msg_;
      // 重置消息（清空旧数据）
      state_msg.header.stamp = time;
      state_msg.joint_names.clear();
      state_msg.displacements.clear();
      state_msg.velocities.clear();
      state_msg.efforts.clear();

      state_msg.sensor_names.clear();
      state_msg.sensor_tp.clear();

      size_t state_iface_idx = 0;
      for (size_t joint_idx = 0; joint_idx < state_joints_.size(); ++joint_idx)
      {
        const std::string &joint_name = state_joints_[joint_idx];
        double current_pos = std::numeric_limits<double>::quiet_NaN();
        double current_vel = std::numeric_limits<double>::quiet_NaN();
        double current_effort = std::numeric_limits<double>::quiet_NaN();
        // 遍历该关节的所有状态接口（适配配置）
        for (auto &iface_name : params_.state_interfaces)
        {
          if (state_iface_idx >= state_interfaces_.size())
          {
            RCLCPP_WARN(get_node()->get_logger(),
                        "关节%s的%s状态接口索引越界", joint_name.c_str(), iface_name.c_str());
            break;
          }

          const auto &state_iface = state_interfaces_[state_iface_idx];
          if (state_iface.get_interface_name() != iface_name)
          {
            state_iface_idx++;
            continue;
          }

          // 读取状态值
          if (iface_name == "position")
          {
            current_pos = state_iface.get_value();
          }
          else if (iface_name == "velocity")
          {
            current_vel = state_iface.get_value();
          }
          else if (iface_name == "effort")
          {
            current_effort = state_iface.get_value();
          }
          state_iface_idx++;
        }

        // 填充状态消息（与state_joints_一致）
        state_msg.joint_names.push_back(joint_name);
        state_msg.displacements.push_back(current_pos);
        state_msg.velocities.push_back(current_vel);
        state_msg.efforts.push_back(current_effort);
        // RCLCPP_INFO(get_node()->get_logger(),
        //              "关节%s状态：位置=%.4f, 速度=%.4f, 力矩=%.4f, 温度=%.4f",
        //              joint_name.c_str(), current_pos, current_vel, current_effort, current_tp);
      }
      
          // 3.2 读取单个TP传感器值（核心：仅处理一个TP值）
    // 继续遍历剩余的状态接口，查找TP接口
    double tp_value = std::numeric_limits<double>::quiet_NaN();
    double tp1_value = std::numeric_limits<double>::quiet_NaN();
    double tp2_value = std::numeric_limits<double>::quiet_NaN();
    for(int sensor_idx = 0; sensor_idx < params_.sensors.size(); ++sensor_idx)
    {
        const std::string &sensor_name = params_.sensors[sensor_idx];
        RCLCPP_DEBUG(get_node()->get_logger(), "正在处理传感器：%s", sensor_name.c_str());
        for (state_iface_idx = 0; state_iface_idx < state_interfaces_.size(); ++state_iface_idx)
        {
          const auto &sensor_iface = state_interfaces_[state_iface_idx];
          // 匹配TP接口（名称可根据实际配置调整，如"sensor1/tp"）
          if (sensor_iface.get_interface_name() == "tp")
          {
            tp_value = sensor_iface.get_value();
            // 校验TP值有效性
            if (std::isnan(tp_value))
            {
              RCLCPP_WARN(get_node()->get_logger(), "TP传感器值为NaN（无效）");
            }
            else
            {
              RCLCPP_DEBUG(get_node()->get_logger(), "读取到TP传感器值：%.4f", tp_value);
            }
          }
          if (sensor_iface.get_interface_name() == "tp1")
          {
            tp1_value = sensor_iface.get_value();
            // 校验TP1值有效性
            if (std::isnan(tp1_value))
            {
              RCLCPP_WARN(get_node()->get_logger(), "TP1传感器值为NaN（无效）");
            }
            else
            {
              RCLCPP_DEBUG(get_node()->get_logger(), "读取到TP1传感器值：%.4f", tp1_value);
            }
          }
          if (sensor_iface.get_interface_name() == "tp2")
          {
            tp2_value = sensor_iface.get_value();
            // 校验TP2值有效性
            if (std::isnan(tp2_value))
            {
              RCLCPP_WARN(get_node()->get_logger(), "TP2传感器值为NaN（无效）");
            }
            else
            {
              RCLCPP_DEBUG(get_node()->get_logger(), "读取到TP2传感器值：%.4f", tp2_value);
            }
      
          }
        }
        // 3.3 填充TP值到状态消息（需确保StateMsg中定义了tp_value字段）
        state_msg.sensor_names.push_back(sensor_name); // 假设StateMsg中有sensor_names字段
        state_msg.sensor_tp.push_back(tp_value); // 假设StateMsg中有tp_value字段
        state_msg.sensor_tp.push_back(tp1_value); // 假设StateMsg中有tp1_value字段
        state_msg.sensor_tp.push_back(tp2_value); // 假设StateMsg中有tp2_value字段
    }
    

    

      // 解锁并发布（关键修复：实时安全发布）
      state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

} // namespace my_position_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    my_position_controller::MyPositionController, controller_interface::ControllerInterface)
