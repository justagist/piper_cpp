#include "piper_cpp_ros/piper_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include "piper_cpp/interface/piper_interface_v2.h"
#include "piper_cpp/types/control.h"

namespace piper_cpp_ros
{

namespace
{
constexpr double kRadToMd = 1000.0 * 180.0 / M_PI;
constexpr double kMdToRad = 1.0 / kRadToMd;

// piper_cpp's high-speed feedback reports motor_speed in 0.001 rad/s and effort in 0.001 N*m.
constexpr double kMilliToUnit = 1.0e-3;

// Gripper feedback / command: jaw stroke is in 0.001 mm = micrometers (1 um = 1e-6 m).
constexpr double kUmToM = 1.0e-6;
constexpr double kMToUm = 1.0e6;

inline int32_t radToMd(double rad) { return static_cast<int32_t>(std::lround(rad * kRadToMd)); }
inline double mdToRad(int32_t md) { return static_cast<double>(md) * kMdToRad; }

constexpr auto kLogger = "PiperHardware";
} // namespace

hardware_interface::CallbackReturn
    PiperHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto get_param = [&](const std::string& key, const std::string& def) -> std::string
    {
        auto it = info_.hardware_parameters.find(key);
        return it != info_.hardware_parameters.end() ? it->second : def;
    };
    auto parse_bool = [](const std::string& v, bool def) -> bool
    {
        if (v == "true" || v == "True" || v == "1")
            return true;
        if (v == "false" || v == "False" || v == "0")
            return false;
        return def;
    };

    can_interface_ = get_param("can_interface", "can0");
    try
    {
        speed_pct_ = std::stoi(get_param("speed_pct", "30"));
    }
    catch (const std::exception&)
    {
        speed_pct_ = 30;
    }
    if (speed_pct_ < 0)
        speed_pct_ = 0;
    if (speed_pct_ > 100)
        speed_pct_ = 100;
    go_to_zero_on_activate_ = parse_bool(get_param("go_to_zero_on_activate", "true"), true);
    with_gripper_ = parse_bool(get_param("with_gripper", "false"), false);
    home_gripper_on_activate_ = parse_bool(get_param("home_gripper_on_activate", "false"), false);
    try
    {
        gripper_max_effort_nm_ = std::stod(get_param("gripper_max_effort", "1.0"));
    }
    catch (const std::exception&)
    {
        gripper_max_effort_nm_ = 1.0;
    }
    // Gripper firmware accepts 0..5000 milli-N*m (0..5 N*m).
    {
        long milli = std::lround(gripper_max_effort_nm_ * 1000.0);
        if (milli < 0)
            milli = 0;
        if (milli > 5000)
            milli = 5000;
        gripper_effort_milli_nm_ = static_cast<uint16_t>(milli);
    }

    const std::size_t expected_joints = kArmJoints + (with_gripper_ ? 1u : 0u);
    if (info_.joints.size() != expected_joints)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLogger), "Expected %zu joints in <ros2_control> (with_gripper=%s), got %zu",
            expected_joints, with_gripper_ ? "true" : "false", info_.joints.size()
        );
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_position_.assign(kArmJoints, 0.0);
    hw_states_velocity_.assign(kArmJoints, 0.0);
    hw_states_effort_.assign(kArmJoints, 0.0);
    hw_commands_position_.assign(kArmJoints, 0.0);

    for (std::size_t i = 0; i < kArmJoints; ++i)
    {
        const auto& joint = info_.joints[i];
        if (joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLogger),
                "Arm joint '%s' must declare exactly one command interface, of type 'position'.", joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        bool has_pos_state = false;
        for (const auto& iface : joint.state_interfaces)
        {
            if (iface.name == hardware_interface::HW_IF_POSITION)
                has_pos_state = true;
            else if (iface.name == hardware_interface::HW_IF_VELOCITY || iface.name == hardware_interface::HW_IF_EFFORT)
                continue;
            else
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger(kLogger),
                    "Arm joint '%s' declares unsupported state interface '%s'. Supported: position, velocity, effort.",
                    joint.name.c_str(), iface.name.c_str()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        if (!has_pos_state)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLogger), "Arm joint '%s' must declare a 'position' state interface.",
                joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    if (with_gripper_)
    {
        const auto& gj = info_.joints[kArmJoints];
        if (gj.command_interfaces.size() != 1 || gj.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLogger),
                "Gripper joint '%s' must declare exactly one command interface, of type 'position'.", gj.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        bool has_pos_state = false;
        for (const auto& iface : gj.state_interfaces)
        {
            if (iface.name == hardware_interface::HW_IF_POSITION)
                has_pos_state = true;
            else if (iface.name == hardware_interface::HW_IF_EFFORT)
                continue;
            else
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger(kLogger),
                    "Gripper joint '%s' declares unsupported state interface '%s'. Supported: position, effort.",
                    gj.name.c_str(), iface.name.c_str()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        if (!has_pos_state)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLogger), "Gripper joint '%s' must declare a 'position' state interface.",
                gj.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(
        rclcpp::get_logger(kLogger),
        "Configured for can_interface='%s', speed_pct=%d, go_to_zero_on_activate=%s, with_gripper=%s",
        can_interface_.c_str(), speed_pct_, go_to_zero_on_activate_ ? "true" : "false", with_gripper_ ? "true" : "false"
    );
    if (with_gripper_)
    {
        RCLCPP_INFO(
            rclcpp::get_logger(kLogger), "Gripper: joint='%s', max_effort=%.3f N*m, home_on_activate=%s",
            info_.joints[kArmJoints].name.c_str(), gripper_max_effort_nm_, home_gripper_on_activate_ ? "true" : "false"
        );
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_configure(const rclcpp_lifecycle::State&)
{
    piper_ = std::make_unique<piper_cpp::PiperInterfaceV2>(can_interface_);
    if (!piper_->connectPort(/*can_init=*/true, /*piper_init=*/true, /*start_threads=*/true))
    {
        RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Failed to open CAN port '%s'", can_interface_.c_str());
        piper_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "CAN port '%s' opened.", can_interface_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_activate(const rclcpp_lifecycle::State&)
{
    using namespace std::chrono_literals;
    using namespace std::chrono;

    if (!piper_)
    {
        RCLCPP_ERROR(rclcpp::get_logger(kLogger), "on_activate() called before on_configure().");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger(kLogger), "Enabling joint motors...");
    piper_->enableRobot();
    auto t0 = steady_clock::now();
    while (!piper_->isEnabled())
    {
        if (steady_clock::now() - t0 > 5s)
        {
            RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Timed out waiting for robot to enable.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(50ms);
    }

    piper_->sendMotionCtrl2(
        piper_cpp::CtrlMode::CanCmd, piper_cpp::MoveMode::MoveJ, static_cast<uint8_t>(speed_pct_),
        piper_cpp::MitMode::PositionVelocity
    );
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "Motion mode: CanCmd / MoveJ / %d%% / PositionVelocity.", speed_pct_);

    if (go_to_zero_on_activate_)
    {
        RCLCPP_INFO(
            rclcpp::get_logger(kLogger), "Driving joints to zero before handing over to the controller manager..."
        );
        const auto zero_dt = 20ms;
        const auto zero_duration = 3s;
        const auto zero_t0 = steady_clock::now();
        while (steady_clock::now() - zero_t0 < zero_duration)
        {
            piper_->controlJoints(0, 0, 0, 0, 0, 0);
            std::this_thread::sleep_for(zero_dt);
        }
    }

    // Sample current pose into both state and command buffers so the first write() doesn't
    // jolt the joints away from their current position (mostly defensive).
    auto js = piper_->getArmJointStates();
    auto wait_t0 = steady_clock::now();
    while (!js.is_valid)
    {
        if (steady_clock::now() - wait_t0 > 2s)
        {
            RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Timed out waiting for joint feedback.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(20ms);
        js = piper_->getArmJointStates();
    }
    hw_states_position_[0] = mdToRad(js.value.joint_1);
    hw_states_position_[1] = mdToRad(js.value.joint_2);
    hw_states_position_[2] = mdToRad(js.value.joint_3);
    hw_states_position_[3] = mdToRad(js.value.joint_4);
    hw_states_position_[4] = mdToRad(js.value.joint_5);
    hw_states_position_[5] = mdToRad(js.value.joint_6);
    hw_commands_position_ = hw_states_position_;

    if (with_gripper_)
    {
        // Optionally latch the current jaw position as zero. This re-calibrates the gripper
        // at whatever pose the jaws happen to be in right now -- only enable if you always
        // activate from a known reference (typically fully closed).
        if (home_gripper_on_activate_)
        {
            RCLCPP_INFO(
                rclcpp::get_logger(kLogger),
                "home_gripper_on_activate=true -- latching current jaw position as gripper zero."
            );
            piper_->setGripperZero();
            std::this_thread::sleep_for(50ms);
        }

        auto gs = piper_->getGripperStates();
        auto g_t0 = steady_clock::now();
        while (!gs.is_valid)
        {
            if (steady_clock::now() - g_t0 > 2s)
            {
                RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Timed out waiting for gripper feedback.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            std::this_thread::sleep_for(20ms);
            gs = piper_->getGripperStates();
        }
        hw_state_gripper_position_ = static_cast<double>(gs.value.grippers_angle) * kUmToM;
        hw_state_gripper_effort_ = static_cast<double>(gs.value.grippers_effort) * kMilliToUnit;
        hw_command_gripper_position_ = hw_state_gripper_position_;
        if (!gs.value.foc_status.homing_status)
        {
            RCLCPP_WARN(
                rclcpp::get_logger(kLogger),
                "Gripper has not been homed (homing_status=false). Position commands will be ignored "
                "by the firmware until setGripperZero() is sent. Set home_gripper_on_activate:=true "
                "to home automatically (only if jaws start from a known reference)."
            );
        }
    }

    active_.store(true, std::memory_order_release);
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "Activated. Ready to stream joint targets.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
    // Stop streaming targets. The arm firmware holds the last commanded pose against gravity;
    // do NOT disable the joint motors here -- that would drop the arm.
    active_.store(false, std::memory_order_release);
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "Deactivated. Joints left enabled and holding last commanded pose.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_cleanup(const rclcpp_lifecycle::State&)
{
    if (piper_)
    {
        piper_->disconnectPort(std::chrono::milliseconds{200});
        piper_.reset();
        RCLCPP_INFO(rclcpp::get_logger(kLogger), "CAN port closed.");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PiperHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifaces;
    ifaces.reserve(kArmJoints * 3 + (with_gripper_ ? 2u : 0u));
    for (std::size_t i = 0; i < kArmJoints; ++i)
    {
        for (const auto& iface : info_.joints[i].state_interfaces)
        {
            if (iface.name == hardware_interface::HW_IF_POSITION)
                ifaces.emplace_back(info_.joints[i].name, iface.name, &hw_states_position_[i]);
            else if (iface.name == hardware_interface::HW_IF_VELOCITY)
                ifaces.emplace_back(info_.joints[i].name, iface.name, &hw_states_velocity_[i]);
            else if (iface.name == hardware_interface::HW_IF_EFFORT)
                ifaces.emplace_back(info_.joints[i].name, iface.name, &hw_states_effort_[i]);
        }
    }
    if (with_gripper_)
    {
        const auto& gj = info_.joints[kArmJoints];
        for (const auto& iface : gj.state_interfaces)
        {
            if (iface.name == hardware_interface::HW_IF_POSITION)
                ifaces.emplace_back(gj.name, iface.name, &hw_state_gripper_position_);
            else if (iface.name == hardware_interface::HW_IF_EFFORT)
                ifaces.emplace_back(gj.name, iface.name, &hw_state_gripper_effort_);
        }
    }
    return ifaces;
}

std::vector<hardware_interface::CommandInterface> PiperHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    ifaces.reserve(kArmJoints + (with_gripper_ ? 1u : 0u));
    for (std::size_t i = 0; i < kArmJoints; ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]);
    }
    if (with_gripper_)
    {
        const auto& gj = info_.joints[kArmJoints];
        ifaces.emplace_back(gj.name, hardware_interface::HW_IF_POSITION, &hw_command_gripper_position_);
    }
    return ifaces;
}

hardware_interface::return_type PiperHardware::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (!piper_)
        return hardware_interface::return_type::ERROR;
    auto js = piper_->getArmJointStates();
    if (js.is_valid)
    {
        hw_states_position_[0] = mdToRad(js.value.joint_1);
        hw_states_position_[1] = mdToRad(js.value.joint_2);
        hw_states_position_[2] = mdToRad(js.value.joint_3);
        hw_states_position_[3] = mdToRad(js.value.joint_4);
        hw_states_position_[4] = mdToRad(js.value.joint_5);
        hw_states_position_[5] = mdToRad(js.value.joint_6);
    }
    auto hs = piper_->getArmHighSpeedFeedbacks();
    if (hs.is_valid)
    {
        for (std::size_t i = 0; i < kArmJoints; ++i)
        {
            const auto& fb = hs.value.high_spd_feedbacks[i];
            hw_states_velocity_[i] = static_cast<double>(fb.motor_speed) * kMilliToUnit;
            hw_states_effort_[i] = static_cast<double>(fb.effort) * kMilliToUnit;
        }
    }
    if (with_gripper_)
    {
        auto gs = piper_->getGripperStates();
        if (gs.is_valid)
        {
            hw_state_gripper_position_ = static_cast<double>(gs.value.grippers_angle) * kUmToM;
            hw_state_gripper_effort_ = static_cast<double>(gs.value.grippers_effort) * kMilliToUnit;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiperHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (!piper_)
        return hardware_interface::return_type::ERROR;
    if (!active_.load(std::memory_order_acquire))
        return hardware_interface::return_type::OK;

    piper_->controlJoints(
        radToMd(hw_commands_position_[0]), radToMd(hw_commands_position_[1]), radToMd(hw_commands_position_[2]),
        radToMd(hw_commands_position_[3]), radToMd(hw_commands_position_[4]), radToMd(hw_commands_position_[5])
    );
    if (with_gripper_)
    {
        const int32_t angle_um = static_cast<int32_t>(std::lround(hw_command_gripper_position_ * kMToUm));
        piper_->controlGripper(angle_um, gripper_effort_milli_nm_, piper_cpp::GripperStatusCode::Enable);
    }
    return hardware_interface::return_type::OK;
}

} // namespace piper_cpp_ros

PLUGINLIB_EXPORT_CLASS(piper_cpp_ros::PiperHardware, hardware_interface::SystemInterface)
