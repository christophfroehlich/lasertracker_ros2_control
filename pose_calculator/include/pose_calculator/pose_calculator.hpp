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

#ifndef POSE_CALCULATOR__POSE_CALCULATOR_HPP_
#define POSE_CALCULATOR__POSE_CALCULATOR_HPP_

// system
#include <array>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pose_calculator
{
class PoseCalculator : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

protected:
  std::vector<std::string> state_interface_names_;

  geometry_msgs::msg::PoseStamped pose_msg_;

  mutable std::array<double, 7> data_{{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};

private:
  /**
   * @brief Update the data array from the state interfaces.
   * @note This method is thread-safe and non-blocking.
   * @note This method might return stale data if the data is not updated. This is to ensure that
   * the data from the sensor is not discontinuous.
   */
  void update_data_from_interfaces() const
  {
    for (auto i = 0u; i < data_.size(); ++i) {
      const auto data = state_interfaces_[i].get_optional();
      if (data.has_value()) {
        data_[i] = data.value();
      }
    }
  }

};
}  // namespace pose_calculator

#endif  // POSE_CALCULATOR__POSE_CALCULATOR_HPP_
