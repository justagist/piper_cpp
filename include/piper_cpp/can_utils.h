#ifndef PIPER_CPP_CAN_UTILS_H
#define PIPER_CPP_CAN_UTILS_H

#include <optional>
#include <string>
#include <vector>

namespace piper_cpp
{
using PortPair = std::pair<std::string, std::string>;

/// Discover available CAN interfaces and their USB bus addresses
std::vector<PortPair> findPorts();

/// List CAN interfaces that are currently up
std::vector<std::string> activePorts();

/// Activate (configure bitrate, bring up, optionally rename) a set of CAN ports
void activate(
    const std::vector<PortPair>& ports = {}, const std::string& default_name_prefix = "can",
    int default_bitrate = 1000000, std::optional<int> timeout = std::nullopt
);

/// Retrieve USB adapter serial for a CAN interface
std::optional<std::string> getCanAdapterSerial(const std::string& can_port);

bool interfaceIsUp(const std::string& name);
std::optional<int> getInterfaceBitrate(const std::string& name);

} // namespace piper_cpp

#endif // PIPER_CPP_CAN_UTILS_H