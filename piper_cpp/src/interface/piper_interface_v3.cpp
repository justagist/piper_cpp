#include "piper_cpp/interface/piper_interface_v3.h"
#include "piper_cpp/protocol/piper_parser_v3.h"

namespace piper_cpp
{

PiperInterfaceV3::PiperInterfaceV3(
    std::string can_name, bool judge_flag, bool auto_init, bool dh_offset, bool sdk_joint_limit, bool sdk_gripper_limit
)
    : PiperInterfaceV2(
          std::make_unique<PiperParserV3>(), std::move(can_name), judge_flag, auto_init, dh_offset, sdk_joint_limit,
          sdk_gripper_limit
      )
{
}

PiperInterfaceV3::PiperInterfaceV3(
    std::unique_ptr<StdCanInterface> can, bool dh_offset, bool sdk_joint_limit, bool sdk_gripper_limit
)
    : PiperInterfaceV2(std::make_unique<PiperParserV3>(), std::move(can), dh_offset, sdk_joint_limit, sdk_gripper_limit)
{
}

bool PiperInterfaceV3::controlGripperWidth(int32_t width_um, uint16_t effort, GripperStatusCode status_code)
{
    return sendGripperCtrl(GripperMode::Width, width_um, effort, status_code);
}

bool PiperInterfaceV3::controlGripperAngle(int32_t angle_millideg, uint16_t effort, GripperStatusCode status_code)
{
    return sendGripperCtrl(GripperMode::Angle, angle_millideg, effort, status_code);
}

} // namespace piper_cpp
