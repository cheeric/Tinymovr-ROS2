#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdexcept>
#include <algorithm>

#include "tinymovr_hardware.hpp"
#include "socketcan_cpp/socketcan_cpp.hpp"

scpp::SocketCan socket_can;

const char* SocketCanErrorToString(scpp::SocketCanStatus status) {
    switch (status) {
        case scpp::STATUS_OK:
            return "No error";
        case scpp::STATUS_SOCKET_CREATE_ERROR:
            return "SocketCAN socket creation error";
        case scpp::STATUS_INTERFACE_NAME_TO_IDX_ERROR:
            return "SocketCAN interface name to index error";
        case scpp::STATUS_MTU_ERROR:
            return "SocketCAN maximum transfer unit error";
        case scpp::STATUS_CANFD_NOT_SUPPORTED:
            return "SocketCAN flexible data-rate not supported on this interface";
        case scpp::STATUS_ENABLE_FD_SUPPORT_ERROR:
            return "Error enabling SocketCAN flexible-data-rate support";
        case scpp::STATUS_WRITE_ERROR:
            return "SocketCAN write error";
        case scpp::STATUS_READ_ERROR:
            return "SocketCAN read error";
        case scpp::STATUS_BIND_ERROR:
            return "SocketCAN bind error";
        default:
            return "Unknown SocketCAN error";
    }
}

void send_cb(uint32_t arbitration_id, uint8_t *data, uint8_t data_length, bool rtr)
{
    RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "Attempting to write CAN frame with arbitration_id: %d", arbitration_id);
    scpp::CanFrame cf_to_write;
    cf_to_write.id = arbitration_id;
    if (rtr) cf_to_write.id |= CAN_RTR_FLAG;
    cf_to_write.len = data_length;
    std::copy(data, data + data_length, cf_to_write.data);
    auto write_status = socket_can.write(cf_to_write);
    if (write_status != scpp::STATUS_OK)
    {
        throw std::runtime_error(SocketCanErrorToString(write_status));
    }
    else
    {
        RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "CAN frame with arbitration_id: %d written successfully.", arbitration_id);
    }
}

bool recv_cb(uint32_t *arbitration_id, uint8_t *data, uint8_t *data_length)
{
    RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "Attempting to read CAN frame...");
    scpp::CanFrame fr;
    scpp::SocketCanStatus read_status = socket_can.read(fr);
    if (read_status == scpp::STATUS_OK)
    {
        *arbitration_id = fr.id & CAN_EFF_MASK;
        *data_length = fr.len;
        std::copy(fr.data, fr.data + fr.len, data);
        RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "CAN frame with arbitration_id: %d read successfully.", *arbitration_id);
        return true;
    }
    return false;
}

void delay_us_cb(uint32_t us)
{
    rclcpp::sleep_for(std::chrono::microseconds(us));
}

namespace tinymovr_ros2
{

hardware_interface::CallbackReturn TinymovrHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "on_init() running...");

    try {
        can_interface_name_ = info_.hardware_parameters.at("can_interface_name");
    } catch (const std::out_of_range &ex) {
        RCLCPP_FATAL(rclcpp::get_logger("TinymovrHardware"), "Required hardware parameter 'can_interface_name' not set in URDF.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    size_t num_joints = info_.joints.size();
    hw_position_states_.resize(num_joints, 0.0);
    hw_velocity_states_.resize(num_joints, 0.0);
    hw_effort_states_.resize(num_joints, 0.0);
    hw_position_commands_.resize(num_joints, 0.0);
    hw_velocity_commands_.resize(num_joints, 0.0);
    hw_effort_commands_.resize(num_joints, 0.0);
    rads_to_ticks_.resize(num_joints, 0.0);

    for (const auto & joint : info_.joints)
    {
        int joint_idx = &joint - &info_.joints[0];
        try {
            int id = std::stoi(joint.parameters.at("id"));
            int delay_us = std::stoi(joint.parameters.at("delay_us"));
            rads_to_ticks_[joint_idx] = std::stod(joint.parameters.at("rads_to_ticks"));
            
            RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "Initializing joint '%s' (id: %d, delay: %d us, rads_to_ticks: %f)", 
                joint.name.c_str(), id, delay_us, rads_to_ticks_[joint_idx]);

            servos_.emplace_back(id, &send_cb, &recv_cb, &delay_us_cb, delay_us);
        } catch (const std::out_of_range &ex) {
            RCLCPP_FATAL(rclcpp::get_logger("TinymovrHardware"), "Missing required parameter for joint '%s': %s", joint.name.c_str(), ex.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "on_init() finished successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TinymovrHardware::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "export_state_interfaces() running...");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TinymovrHardware::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "export_command_interfaces() running...");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn TinymovrHardware::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "Activating Hardware...");
    if (socket_can.open(can_interface_name_.c_str()) != scpp::STATUS_OK)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TinymovrHardware"), "Could not open CAN interface '%s'", can_interface_name_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "Socketcan opened successfully on '%s'.", can_interface_name_.c_str());

    RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "Asserting spec compatibility...");
    for (auto& servo : servos_)
    {
        if (servo.get_protocol_hash() != avlos_proto_hash) {
            RCLCPP_FATAL(rclcpp::get_logger("TinymovrHardware"), "Protocol hash mismatch for servo ID %d", servo.get_uid());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "Spec compatibility OK.");
    
    RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "Asserting calibrated...");
    for (auto& servo : servos_)
    {
        if (!servo.get_calibrated()) {
            RCLCPP_FATAL(rclcpp::get_logger("TinymovrHardware"), "Servo ID %d is not calibrated.", servo.get_uid());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "Calibration OK.");

    for (auto& servo : servos_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("TinymovrHardware"), "Setting state and mode for servo ID %d", servo.get_uid());
        servo.controller.set_state(2);
        servo.controller.set_mode(2);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
        if (servo.controller.get_state() != 2 || servo.controller.get_mode() != 2) {
             RCLCPP_ERROR(rclcpp::get_logger("TinymovrHardware"), "Failed to set mode for servo ID %d.", servo.get_uid());
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "Hardware activated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TinymovrHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "Deactivating Hardware...");
    try {
        for (auto& servo : servos_)
        {
            servo.controller.set_state(0);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("TinymovrHardware"), "Error during deactivation: %s", e.what());
    }
    socket_can.close();
    RCLCPP_INFO(rclcpp::get_logger("TinymovrHardware"), "Hardware deactivated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TinymovrHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    try {
        for (size_t i = 0; i < servos_.size(); ++i)
        {
            const double ticks_to_rads = 1.0 / rads_to_ticks_[i];
            hw_position_states_[i] = servos_[i].sensors.user_frame.get_position_estimate() * ticks_to_rads;
            hw_velocity_states_[i] = servos_[i].sensors.user_frame.get_velocity_estimate() * ticks_to_rads;
            hw_effort_states_[i] = servos_[i].controller.current.get_Iq_estimate();
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("TinymovrHardware"), "Error during read: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type TinymovrHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    try {
        for (size_t i = 0; i < servos_.size(); ++i)
        {
            servos_[i].controller.position.set_setpoint(hw_position_commands_[i] * rads_to_ticks_[i]);
            servos_[i].controller.velocity.set_setpoint(hw_velocity_commands_[i] * rads_to_ticks_[i]);
            servos_[i].controller.current.set_Iq_setpoint(hw_effort_commands_[i]);
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("TinymovrHardware"), "Error during write: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(
  tinymovr_ros2::TinymovrHardware,
  hardware_interface::SystemInterface
)