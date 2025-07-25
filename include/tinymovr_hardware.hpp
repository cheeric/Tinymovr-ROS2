#pragma once

#include <string>
#include <vector>

// ROS2 control headers
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

// For exporting the class as a plugin
#include "pluginlib/class_list_macros.hpp"

// The Tinymovr C++ library
#include <tinymovr/tinymovr.hpp>

// For managing library symbol visibility
#include "visibility_control.hpp"

namespace tinymovr_ros2
{

class TinymovrHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(TinymovrHardware)

    TINYMOVR_ROS2_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    TINYMOVR_ROS2_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    TINYMOVR_ROS2_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    TINYMOVR_ROS2_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    TINYMOVR_ROS2_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    TINYMOVR_ROS2_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    TINYMOVR_ROS2_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::vector<double> hw_position_states_;
    std::vector<double> hw_velocity_states_;
    std::vector<double> hw_effort_states_;

    std::vector<double> hw_position_commands_;
    std::vector<double> hw_velocity_commands_;
    std::vector<double> hw_effort_commands_;
    
    std::vector<Tinymovr> servos_;
    std::vector<double> rads_to_ticks_;
    std::string can_interface_name_;
};

} // namespace tinymovr_ros2