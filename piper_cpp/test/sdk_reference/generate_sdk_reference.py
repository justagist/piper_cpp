#!/usr/bin/env python3
"""Generate sdk_reference_frames.h from the official Python piper_sdk.

Test tooling only: the library never runs Python, and the generated header is committed, so
the tests need neither this script nor network access. Regenerate only when moving the pin.

    git clone https://github.com/agilexrobotics/piper_sdk.git /tmp/piper_sdk
    git -C /tmp/piper_sdk checkout b847e5393a7ce6ff8508d713eca7bdd3a42421b4
    python3 -m venv /tmp/sdk_venv && /tmp/sdk_venv/bin/pip install python-can
    /tmp/sdk_venv/bin/python generate_sdk_reference.py --sdk /tmp/piper_sdk

Commands go through the SDK's public interface methods with the CAN bus replaced by a
recorder, so the frames include its unit conversion, range handling and packing. A call the
SDK rejects with ValueError/TypeError is recorded as rejected.
"""

import argparse
import logging
import math
import subprocess
import sys
from pathlib import Path

PINNED_COMMIT = "b847e5393a7ce6ff8508d713eca7bdd3a42421b4"
OUTPUT = Path(__file__).with_name("sdk_reference_frames.h")


class RecordingCan:
    """Stands in for the SDK's CAN wrapper and keeps every frame it is asked to send."""

    class CAN_STATUS:
        SEND_MESSAGE_SUCCESS = object()

    def __init__(self):
        self.sent = []

    def SendCanMessage(self, arbitration_id, data):
        self.sent.append((arbitration_id, bytes(data)))
        return self.CAN_STATUS.SEND_MESSAGE_SUCCESS


def mit_values():
    """Physical MIT inputs: zero, both ends, negatives, mid-range, clamping and code boundaries."""
    values = [
        (0.0, 0.0, 10.0, 0.8, 0.0),
        (-12.5, -45.0, 0.0, -5.0, -16.0),
        (12.5, 45.0, 500.0, 5.0, 16.0),
        (-1.0, -3.0, 5.0, -0.5, -2.5),
        (0.7, 1.3, 33.3, 1.1, 3.3),
        (-6.25, 22.5, 250.0, 2.5, -8.0),
        (-1e-9, -1e-9, 1e-9, -1e-9, -1e-9),
        (13.0, 50.0, 600.0, 6.0, 17.0),
        (-13.0, -50.0, -1.0, -6.0, -17.0),
    ]
    # Inputs exactly on a code boundary and one ulp below it, where truncation changes code.
    for k in (1, 2047, 2048, 4094):
        t = -16.0 + k * 32.0 / 4095
        values += [(0.0, 0.0, 10.0, 0.8, t), (0.0, 0.0, 10.0, 0.8, math.nextafter(t, -math.inf))]
    for k in (1, 32767, 32768, 65534):
        p = -12.5 + k * 25.0 / 65535
        values += [(p, 0.0, 10.0, 0.8, 0.0), (math.nextafter(p, -math.inf), 0.0, 10.0, 0.8, 0.0)]
    return values


def command_cases():
    """(protocols, api, args, sdk_call). `api` names the C++ entry point the test calls."""
    both = (2, 3)
    cases = []
    for motor in range(1, 7):
        for v in mit_values():
            cases.append(((3,), "controlJointMit", (motor, *v), lambda p, *a: p.JointMitCtrl(*a)))
    for bad in (math.nan, math.inf, -math.inf):
        cases.append(((3,), "controlJointMit", (1, bad, 0.0, 10.0, 0.8, 0.0), lambda p, *a: p.JointMitCtrl(*a)))
        cases.append(((3,), "controlJointMit", (6, 0.0, 0.0, 10.0, 0.8, bad), lambda p, *a: p.JointMitCtrl(*a)))

    # Raw packed fields straight into the parser: isolates the bit layout from the scaling.
    for protocol, t_bits in ((2, 8), (3, 12)):
        t_max = (1 << t_bits) - 1
        for motor, packed in (
            (1, (0x1234, 0xABC, 0xDEF, 0x345, 0x678 & t_max)),
            (2, (0, 0, 0, 0, 0)),
            (3, (0xFFFF, 0xFFF, 0xFFF, 0xFFF, t_max)),
            (6, (0x8000, 0x800, 0x052, 0x947, (t_max + 1) // 2)),
        ):
            cases.append(((protocol,), "encodeJointMit", (motor, *packed), None))

    # 0x151, args as the SDK takes them (wire move_mode). V2's own rejections are not compared:
    # the C++ V2 encoder keeps its existing permissive behaviour.
    for args in ((1, 1, 50, 0x00, 0, 0), (1, 4, 100, 0xAD, 0, 0), (1, 0, 30, 0x00, 0, 3), (7, 5, 10, 0x00, 255, 1),
                 (1, 2, 0, 0x00, 0, 2), (1, 1, 50, 0x00, 0, 4)):
        cases.append(((2,), "sendMotionCtrl2", args, lambda p, *a: p.MotionCtrl_2(*a)))
    for args in ((1, 6, 100, 0xAD, 0, 0), (1, 1, 50, 0x00, 0, 4), (1, 0, 30, 0x00, 0, 3), (7, 5, 10, 0x00, 255, 1),
                 (1, 2, 0, 0xFF, 0, 2), (1, 1, 50, 0x00, 0, 5), (2, 1, 50, 0x00, 0, 0)):
        cases.append(((3,), "sendMotionCtrl2", args, lambda p, *a: p.MotionCtrl_2(*a)))

    for args in ((30000, 1000, 0x01), (0, 0, 0x00), (-5000, 5000, 0x03), (70000, 2000, 0x02)):
        cases.append((both, "controlGripper", args, lambda p, *a: p.GripperCtrl(*a, 0x00)))
    # The SDK rejects efforts above 5000 under both protocols; only V3 here does the same.
    cases.append(((3,), "controlGripper", (1000, 5001, 0x01), lambda p, *a: p.GripperCtrl(*a, 0x00)))
    cases.append((both, "setGripperZero", (), lambda p: p.GripperCtrl(0, 0, 0x00, 0xAE)))
    for args in ((30000, 1000, 0x01), (25000, 500, 0x03), (-45000, 1000, 0x05), (90000, 5000, 0x07),
                 (0, 0, 0x04), (12345, 0, 0x06), (1000, 5001, 0x01), (1000, 5001, 0x05)):
        cases.append(((3,), "controlGripperV3", args, lambda p, *a: p.GripperCtrl_V3(*a, 0x00)))

    unchanged = [
        ("controlJoints", (10000, -20000, 30000, -40000, 50000, -60000), lambda p, *a: p.JointCtrl(*a)),
        ("controlEndPose", (150000, -20000, 300000, -179000, 85000, -90000), lambda p, *a: p.EndPoseCtrl(*a)),
        ("enableMotor", (7,), lambda p, m: p.EnableArm(m)),
        ("enableMotor", (3,), lambda p, m: p.EnableArm(m)),
        ("disableMotor", (7,), lambda p, m: p.DisableArm(m)),
        ("emergencyStop", (0x01,), lambda p, s: p.EmergencyStop(s)),
        ("emergencyStop", (0x02,), lambda p, s: p.EmergencyStop(s)),
        ("moveCAxisUpdate", (0x02,), lambda p, n: p.MoveCAxisUpdateCtrl(n)),
        ("sendJointConfig", (3, 0xAE, 0xAE, 300, 0xAE), lambda p, *a: p.JointConfig(*a)),
        ("setMotorAngleLimitsMaxSpeed", (2, 1500, -1500, 3000), lambda p, *a: p.MotorAngleLimitMaxSpdSet(*a)),
        ("setEndVelAccLimits", (1000, 2000, 3000, 4000), lambda p, *a: p.EndSpdAndAccParamSet(*a)),
        ("setCrashProtectionLevels", (1, 2, 3, 4, 5, 8), lambda p, *a: p.CrashProtectionConfig(*a)),
        ("sendArmParamEnquiryAndConfig", (0x01, 0x00, 0x02, 0xAE, 0x01), lambda p, *a: p.ArmParamEnquiryAndConfig(*a)),
        ("setGripperTeachingPendantParam", (150, 100, 5), lambda p, *a: p.GripperTeachingPendantParamConfig(*a)),
        ("setMasterSlaveConfig", (0xFC, 0x00, 0x00, 0x00), lambda p, *a: p.MasterSlaveConfig(*a)),
    ]
    cases += [(both, api, args, call) for api, args, call in unchanged]
    return cases


def encode_packed_mit(sdk, protocol, motor, pos, vel, kp, kd, t):
    import can

    if protocol == 2:
        msgs, parser = sdk["msg_v2"], sdk["parser_v2"]
        mit = msgs.ArmMsgJointMitCtrl(pos, vel, kp, kd, t)
    else:
        msgs, parser = sdk["msg_v3"], sdk["parser_v3"]
        mit = msgs.ArmMsgJointMitCtrl(pos, vel, kp, kd, t)
    msg = msgs.PiperMessage(type_=getattr(msgs.ArmMsgType, f"PiperMsgJointMitCtrl_{motor}"), arm_joint_mit_ctrl=mit)
    frame = can.Message(arbitration_id=0, data=[], is_extended_id=False)
    assert parser.EncodeMessage(msg, frame)
    return [(frame.arbitration_id, bytes(frame.data))]


def decode_cases():
    """(protocols, kind, can_id, data). Every frame is a full 8-byte frame."""
    both = (2, 3)
    cases = []
    for mode in (0x00, 0x01, 0x02, 0x03, 0x05):
        cases.append((both, "status", 0x2A1, [0x01, 0x00, mode, 0x00, 0x01, 0x02, 0x01, 0x02]))
    for mode in (0x04, 0x06, 0x07, 0xFF):
        cases.append(((3,), "status", 0x2A1, [0x01, 0x00, mode, 0x00, 0x01, 0x02, 0x01, 0x02]))
    for mode in (0x00, 0x01, 0x02):
        cases.append(((3,), "gripper", 0x2A8, [0x00, 0x00, 0x75, 0x30, 0xFC, 0x18, 0xC1, mode]))
    cases.append((both, "gripper", 0x2A8, [0xFF, 0xFF, 0xEC, 0x78, 0x03, 0xE8, 0x40, 0x00]))
    cases.append(((3,), "ik", 0x2AA, [0x00, 0x00, 0x27, 0x10, 0xFF, 0xFF, 0xB1, 0xE0]))
    cases.append(((3,), "ik", 0x2AB, [0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF]))
    cases.append(((3,), "ik", 0x2AC, [0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]))
    cases.append((both, "joint", 0x2A6, [0x00, 0x00, 0x27, 0x10, 0xFF, 0xFF, 0xB1, 0xE0]))
    cases.append((both, "high_spd", 0x253, [0xFF, 0x38, 0xFF, 0x9C, 0x00, 0x01, 0x86, 0xA0]))
    cases.append((both, "low_spd", 0x265, [0x00, 0xF0, 0xFF, 0xF6, 0xE2, 0x41, 0x01, 0x2C]))
    return cases


def decode(sdk, protocol, kind, can_id, data):
    import can

    msgs = sdk["msg_v2"] if protocol == 2 else sdk["msg_v3"]
    parser = sdk["parser_v2"] if protocol == 2 else sdk["parser_v3"]
    msg = msgs.PiperMessage()
    frame = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    assert parser.DecodeMessage(frame, msg)
    if kind == "status":
        s = msg.arm_status_msgs
        return [int(s.ctrl_mode), int(s.arm_status), int(s.mode_feed), int(s.teach_status), int(s.motion_status),
                s.trajectory_num, s.err_code]
    if kind == "gripper":
        g = msg.gripper_feedback
        # The V2 SDK has no gripper mode; -1 tells the test there is nothing to compare.
        mode = int(msg.gripper_feedback_v3.mode) if protocol == 3 else -1
        return [g.grippers_angle, g.grippers_effort, g.status_code, mode]
    if kind == "ik":
        ik = msg.arm_ik_joint_feedback
        return {0x2AA: [ik.ik_joint_1, ik.ik_joint_2], 0x2AB: [ik.ik_joint_3, ik.ik_joint_4],
                0x2AC: [ik.ik_joint_5, ik.ik_joint_6]}[can_id]
    if kind == "joint":
        return [msg.arm_joint_feedback.joint_3, msg.arm_joint_feedback.joint_4]
    if kind == "high_spd":
        h = msg.arm_high_spd_feedback_3
        return [h.motor_speed, h.current, h.pos]
    if kind == "low_spd":
        lo = msg.arm_low_spd_feedback_5
        return [lo.vol, lo.foc_temp, lo.motor_temp, lo.foc_status_code, lo.bus_current]
    raise ValueError(kind)


def cpp_double(x):
    if math.isnan(x):
        return "kNaN"
    if math.isinf(x):
        return "kInf" if x > 0 else "-kInf"
    return repr(float(x))  # shortest form that reads back as the same double


def cpp_frames(frames):
    out = []
    for can_id, data in frames:
        out.append("{0x%03X, %d, {%s}}" % (can_id, len(data), ", ".join("0x%02X" % b for b in data)))
    return "{" + ", ".join(out) + "}"


def load_sdk(sdk_path):
    head = subprocess.run(["git", "-C", str(sdk_path), "rev-parse", "HEAD"], check=True, capture_output=True,
                          text=True).stdout.strip()
    if head != PINNED_COMMIT:
        sys.exit(f"piper_sdk checkout is at {head}, expected {PINNED_COMMIT}")
    sys.path.insert(0, str(sdk_path))
    from piper_sdk.interface.piper_interface_v2 import C_PiperInterface_V2
    from piper_sdk.interface.piper_interface_v3 import C_PiperInterface_V3
    from piper_sdk.piper_msgs import msg_v2, msg_v3
    from piper_sdk.protocol.protocol_v2 import C_PiperParserV2
    from piper_sdk.protocol.protocol_v3 import C_PiperParserV3

    # The SDK logs a warning for every clamped MIT input; the frames are what matter here.
    logging.disable(logging.WARNING)
    return {
        "interface_v2": C_PiperInterface_V2("sdk-reference-v2", can_auto_init=False),
        "interface_v3": C_PiperInterface_V3("sdk-reference-v3", can_auto_init=False),
        "msg_v2": msg_v2, "msg_v3": msg_v3,
        "parser_v2": C_PiperParserV2(), "parser_v3": C_PiperParserV3(),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sdk", type=Path, required=True, help="piper_sdk checkout at the pinned commit")
    sdk = load_sdk(ap.parse_args().sdk)

    command_rows = []
    for protocols, api, args, call in command_cases():
        for protocol in protocols:
            if call is None:
                frames = encode_packed_mit(sdk, protocol, *args)
            else:
                piper = sdk[f"interface_v{protocol}"]
                piper._arm_can = RecordingCan()
                try:
                    call(piper, *args)
                    frames = piper._arm_can.sent
                except (ValueError, TypeError):
                    frames = None
            rejected = frames is None
            assert len(args) <= 6 and (rejected or 1 <= len(frames) <= 3)
            padded = list(args) + [0] * (6 - len(args))
            command_rows.append(
                "    {%d, \"%s\", {%s}, %s, %d, %s},"
                % (protocol, api, ", ".join(cpp_double(a) for a in padded), "true" if rejected else "false",
                   0 if rejected else len(frames), cpp_frames([] if rejected else frames))
            )

    decode_rows = []
    for protocols, kind, can_id, data in decode_cases():
        for protocol in protocols:
            fields = decode(sdk, protocol, kind, can_id, data)
            fields += [0] * (7 - len(fields))
            decode_rows.append(
                "    {%d, \"%s\", {0x%03X, 8, {%s}}, {%s}},"
                % (protocol, kind, can_id, ", ".join("0x%02X" % b for b in data), ", ".join(str(f) for f in fields))
            )

    OUTPUT.write_text(f"""// Generated by generate_sdk_reference.py from the official piper_sdk; do not edit.
// https://github.com/agilexrobotics/piper_sdk at commit {PINNED_COMMIT}.
#pragma once

#include <cstdint>
#include <limits>

namespace sdk_reference
{{

inline constexpr const char *kSdkCommit = "{PINNED_COMMIT}";
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
inline constexpr double kInf = std::numeric_limits<double>::infinity();

struct Frame
{{
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
}};

/// One SDK command call: `api` names the C++ entry point, `args` its inputs as the SDK took
/// them (zero-padded). `rejected` means the SDK raised instead of sending.
struct CommandCase
{{
    int protocol;
    const char *api;
    double args[6];
    bool rejected;
    int num_frames;
    Frame frames[3];
}};

/// One decoded feedback frame; the meaning of `fields` depends on `kind`.
struct DecodeCase
{{
    int protocol;
    const char *kind;
    Frame frame;
    int64_t fields[7];
}};

inline constexpr CommandCase kCommandCases[] = {{
{chr(10).join(command_rows)}
}};

inline constexpr DecodeCase kDecodeCases[] = {{
{chr(10).join(decode_rows)}
}};

}} // namespace sdk_reference
""")
    print(f"wrote {OUTPUT}: {len(command_rows)} command cases, {len(decode_rows)} decode cases")


if __name__ == "__main__":
    main()
