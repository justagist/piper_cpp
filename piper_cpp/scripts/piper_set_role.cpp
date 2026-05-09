#include "piper_cpp/interface/piper_interface_v2.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// One-shot leader/follower role setter. Mirrors `piper_set_master.py` / `piper_set_slave.py`.
//
// IMPORTANT: when switching from master back to slave, the arm must be POWER-CYCLED before
// motion commands take effect again. The setting is persistent across reboots until changed.
//
// Flags (one is required):
//   --master    Set this arm as the leader/teaching arm (gravity-comp free-drive, ignores ctrl).
//   --slave     Set this arm as the follower/motion-output arm (the normal SDK control mode).

int main(int argc, char* argv[])
{
    enum class Role
    {
        Unset,
        Master,
        Slave
    } role = Role::Unset;

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--master")
            role = Role::Master;
        else if (a == "--slave")
            role = Role::Slave;
        else
        {
            std::cerr << "usage: piper_set_role (--master | --slave)\n";
            return 1;
        }
    }
    if (role == Role::Unset)
    {
        std::cerr << "Specify --master or --slave.\n";
        return 1;
    }

    using namespace piper_cpp;
    PiperInterfaceV2 piper("can0");
    if (!piper.connectPort(/*can_init=*/true, /*piper_init=*/false, /*start_threads=*/false))
    {
        std::cerr << "Failed to open CAN port\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (role == Role::Master)
    {
        std::cout << "Configuring this arm as the leader/teaching arm (0xFA).\n";
        piper.setAsMasterArm();
        std::cout << "Done. Arm now in free-drive; motion commands will be ignored until\n"
                     "switched back to slave AND power-cycled.\n";
    }
    else
    {
        std::cout << "Configuring this arm as the follower/motion-output arm (0xFC).\n";
        piper.setAsSlaveArm();
        std::cout << "Done. If the arm was previously in master mode, POWER-CYCLE it now\n"
                     "before motion commands will take effect.\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    piper.disconnectPort(std::chrono::milliseconds{200});
    return 0;
}
