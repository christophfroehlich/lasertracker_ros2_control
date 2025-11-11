// Copyright (c) 2025 AIT
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

#include "pose_calculator/pose_calculator.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace pose_calculator
{

controller_interface::CallbackReturn PoseCalculator::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PoseCalculator::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration PoseCalculator::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_names_;

  return state_interfaces_config;
}

std::vector<hardware_interface::StateInterface> PoseCalculator::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> exported_state_interfaces;

  std::string export_prefix = get_node()->get_name() + std::string("/target");

  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "orientation.x", &pose_msg_.pose.orientation.x));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "orientation.y", &pose_msg_.pose.orientation.y));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "orientation.z", &pose_msg_.pose.orientation.z));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "orientation.w", &pose_msg_.pose.orientation.w));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "position.x", &pose_msg_.pose.position.x));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "position.y", &pose_msg_.pose.position.y));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "position.z", &pose_msg_.pose.position.z));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "measurement_ok", "double"));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "measure_mode_3d", "double"));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "measure_mode_6d", "double"));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "bad_accuracy", "double"));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "angle_out_of_range", "double"));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "distance_out_of_range", "double"));

  return exported_state_interfaces;
}

controller_interface::CallbackReturn PoseCalculator::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_interface_names_ = {
    "target/angle_hz",
    "target/angle_vt",
    "target/distance",
    "target/quaternion_0",
    "target/quaternion_1",
    "target/quaternion_2",
    "target/quaternion_3",
    "target/status",
  };

  // pre-reserve command interfaces
  state_interfaces_.reserve(state_interface_names_.size());

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseCalculator::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseCalculator::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PoseCalculator::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  update_data_from_interfaces();

  // Convert to Cartesian coordinates
  pose_msg_.pose.position.x = data_[2] * std::sin(-data_[1]) * std::cos(-data_[0]);
  pose_msg_.pose.position.y = data_[2] * std::sin(-data_[1]) * std::sin(-data_[0]);
  pose_msg_.pose.position.z = data_[2] * std::cos(data_[1]);
  // fix order of quaternion components
  pose_msg_.pose.orientation.x = data_[4];
  pose_msg_.pose.orientation.y = data_[5];
  pose_msg_.pose.orientation.z = data_[6];
  pose_msg_.pose.orientation.w = data_[3];

  return controller_interface::return_type::OK;
}

}  // namespace pose_calculator

PLUGINLIB_EXPORT_CLASS(
  pose_calculator::PoseCalculator, controller_interface::ChainableControllerInterface)
