#include "piper_cpp/can_utils.h"
#include <iostream>

// One-shot SocketCAN activation helper. Brings up every detected CAN port using the standard
// `can` interface naming and a 1 Mbit/s bitrate -- the bitrate the Piper arm expects.
//
// Use this when running the C++ SDK on a fresh boot where the CAN device exists in the kernel
// but hasn't been brought up yet. Equivalent to `ip link set can0 up type can bitrate 1000000`
// for each detected adapter, but pulls names automatically.
//
// Usage:
//   can_activate

int main(int /*argc*/, char* /*argv*/[])
{
    piper_cpp::activate({}, "can", 1000000);
    return 0;
}
