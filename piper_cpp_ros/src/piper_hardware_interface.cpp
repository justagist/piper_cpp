#include "piper_cpp_ros/piper_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include "piper_cpp/interface/piper_interface_v2.h"

namespace piper_cpp_ros
{

namespace
{
constexpr double kRadToMd = 1000.0 * 180.0 / M_PI;
constexpr double kMdToRad = 1.0 / kRadToMd;

// piper_cpp's high-speed feedback reports motor_speed in 0.001 rad/s and effort in 0.001 N*m.
constexpr double kMilliToUnit = 1.0e-3;

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

    if (info_.joints.size() != kNumJoints)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLogger), "Expected %zu joints in <ros2_control>, got %zu", kNumJoints,
            info_.joints.size()
        );
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto get_param = [&](const std::string& key, const std::string& def) -> std::string
    {
        auto it = info_.hardware_parameters.find(key);
        return it != info_.hardware_parameters.end() ? it->second : def;
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
    {
        auto v = get_param("go_to_zero_on_activate", "true");
        go_to_zero_on_activate_ = !(v == "false" || v == "False" || v == "0");
    }

    hw_states_position_.assign(kNumJoints, 0.0);
    hw_states_velocity_.assign(kNumJoints, 0.0);
    hw_states_effort_.assign(kNumJoints, 0.0);
    hw_commands_position_.assign(kNumJoints, 0.0);

    for (const auto& joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1
            || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLogger),
                "Joint '%s' must declare exactly one command interface, of type 'position'.", joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        bool has_pos_state = false;
        for (const auto& iface : joint.state_interfaces)
        {
            if (iface.name == hardware_interface::HW_IF_POSITION)
                has_pos_state = true;
            else if (iface.name == hardware_interface::HW_IF_VELOCITY
                     || iface.name == hardware_interface::HW_IF_EFFORT)
                continue;
            else
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger(kLogger),
                    "Joint '%s' declares unsupported state interface '%s'. Supported: position, velocity, effort.",
                    joint.name.c_str(), iface.name.c_str()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        if (!has_pos_state)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLogger), "Joint '%s' must declare a 'position' state interface.",
                joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(
        rclcpp::get_logger(kLogger), "Configured for can_interface='%s', speed_pct=%d, go_to_zero_on_activate=%s",
        can_interface_.c_str(), speed_pct_, go_to_zero_on_activate_ ? "true" : "false"
    );

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
            rclcpp::get_logger(kLogger),
            "Driving joints to zero before handing over to the controller manager..."
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
    ifaces.reserve(kNumJoints * 3);
    for (std::size_t i = 0; i < kNumJoints; ++i)
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
    return ifaces;
}

std::vector<hardware_interface::CommandInterface> PiperHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    ifaces.reserve(kNumJoints);
    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]);
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
        for (std::size_t i = 0; i < kNumJoints; ++i)
        {
            const auto& fb = hs.value.high_spd_feedbacks[i];
            hw_states_velocity_[i] = static_cast<double>(fb.motor_speed) * kMilliToUnit;
            hw_states_effort_[i] = static_cast<double>(fb.effort) * kMilliToUnit;
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
    return hardware_interface::return_type::OK;
}

} // namespace piper_cpp_ros

PLUGINLIB_EXPORT_CLASS(piper_cpp_ros::PiperHardware, hardware_interface::SystemInterface)
