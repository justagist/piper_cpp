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

/// ros2_control hardware interface for the AgileX Piper 6-DOF arm and its optional
/// parallel-jaw gripper.
///
/// Lifecycle:
///   - on_init      : parse joint list + hardware params (`can_interface`, `speed_pct`,
///                    `go_to_zero_on_activate`, `with_gripper`, `gripper_max_effort`,
///                    `home_gripper_on_activate`); allocate state/command buffers.
///   - on_configure : open the CAN port and start the piper_cpp read thread.
///   - on_activate  : enable joint motors, set motion mode (CanCmd/MoveJ/PositionVelocity),
///                    optionally drive all joints to zero, then seed the command buffer
///                    from the live feedback so the first write does not jump the arm.
///                    If `with_gripper` is true, also seed the gripper command from
///                    feedback and (optionally) set its current position as zero.
///   - on_deactivate: stop streaming targets. Joints stay enabled, holding their last
///                    commanded pose -- disabling here would drop the arm.
///   - on_cleanup   : disconnect the CAN port.
///
/// Arm state interfaces : position (rad), velocity (rad/s), effort (N*m) per joint.
///                        Velocity and effort come from the per-joint high-speed feedback
///                        (~200 Hz). Effort is derived by piper_cpp from motor current via
///                        the manufacturer's per-joint coefficient table.
/// Arm command interfaces: position (rad) per joint.
///
/// Gripper (if `with_gripper:=true`) appears as a 7th prismatic joint. State interfaces:
///                        position (m, jaw stroke) and effort (N*m). Command interface:
///                        position (m). The gripper's max effort is fixed at
///                        `gripper_max_effort` (N*m, hardware param) and re-sent every
///                        cycle along with `Enable`.
///
/// Joint ordering: the URDF's `<ros2_control>` block must declare exactly six arm joints
/// (mapped to motors 1..6 in declaration order, regardless of joint name) followed by one
/// gripper joint when `with_gripper` is true. Joint names themselves are passed straight
/// through to ros2_control.
class PiperHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PiperHardware)

    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    static constexpr std::size_t kArmJoints = 6;

    std::unique_ptr<piper_cpp::PiperInterfaceV2> piper_;

    // Configurable via <hardware><param ...> in the URDF's <ros2_control> block.
    std::string can_interface_ = "can0";
    int speed_pct_ = 30;
    bool go_to_zero_on_activate_ = true;
    bool with_gripper_ = false;
    double gripper_max_effort_nm_ = 1.0;
    bool home_gripper_on_activate_ = false;

    // Cached u16 form of gripper_max_effort_nm_ in 0.001 N*m units, range 0..5000.
    uint16_t gripper_effort_milli_nm_ = 1000;

    std::vector<double> hw_states_position_;   ///< rad
    std::vector<double> hw_states_velocity_;   ///< rad/s
    std::vector<double> hw_states_effort_;     ///< N*m
    std::vector<double> hw_commands_position_; ///< rad

    double hw_state_gripper_position_ = 0.0;   ///< m
    double hw_state_gripper_effort_ = 0.0;     ///< N*m
    double hw_command_gripper_position_ = 0.0; ///< m

    std::atomic<bool> active_{false};
};

} // namespace piper_cpp_ros
