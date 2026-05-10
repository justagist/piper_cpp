#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace piper_cpp
{
class PiperInterfaceV2;
}

namespace piper_cpp_ros
{

/// ros2_control hardware interface for the AgileX Piper 6-DOF arm.
///
/// Lifecycle:
///   - on_init      : parse joint list + hardware params (`can_interface`, `speed_pct`,
///                    `go_to_zero_on_activate`); allocate state/command buffers.
///   - on_configure : open the CAN port and start the piper_cpp read thread.
///   - on_activate  : enable joint motors, set motion mode (CanCmd/MoveJ/PositionVelocity),
///                    optionally drive all joints to zero, then seed the command buffer
///                    from the live feedback so the first write does not jump the arm.
///   - on_deactivate: stop streaming targets. Joints stay enabled, holding their last
///                    commanded pose -- disabling here would drop the arm.
///   - on_cleanup   : disconnect the CAN port.
///
/// State interfaces : position (rad), velocity (rad/s), effort (N*m) per joint. Velocity
///                    and effort come from the per-joint high-speed feedback (~200 Hz).
///                    Effort is derived by piper_cpp from motor current via the
///                    manufacturer's per-joint coefficient table.
/// Command interfaces: position (rad) per joint.
///
/// Joint ordering: the URDF's `<ros2_control>` block must declare exactly six joints. The
/// first joint maps to motor 1, the second to motor 2, ..., regardless of joint name. The
/// joint names themselves are passed straight through to ros2_control.
class PiperHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PiperHardware)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params
    ) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    static constexpr std::size_t kNumJoints = 6;

    std::unique_ptr<piper_cpp::PiperInterfaceV2> piper_;

    // Configurable via <hardware><param ...> in the URDF's <ros2_control> block.
    std::string can_interface_ = "can0";
    int speed_pct_ = 30;
    bool go_to_zero_on_activate_ = true;

    std::vector<double> hw_states_position_;   ///< rad
    std::vector<double> hw_states_velocity_;   ///< rad/s
    std::vector<double> hw_states_effort_;     ///< N*m
    std::vector<double> hw_commands_position_; ///< rad

    std::atomic<bool> active_{false};
};

} // namespace piper_cpp_ros
