#include "DRFLEx.h"
#include <iostream>

// Read-only DRFL handshake; no motion or control-authority requests.
int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: probe IP PORT\n";
        return 2;
    }
    DRAFramework::CDRFLEx robot;
    if (!robot.open_connection(argv[1], std::stoul(argv[2]))) {
        std::cerr << "DRFL connection failed\n";
        return 1;
    }
    SYSTEM_VERSION version{};
    const bool success = robot.get_system_version(&version);
    if (success) {
        std::cout << "Controller=" << version._szController
                  << " State=" << robot.get_robot_state() << '\n';
    }
    robot.close_connection();
    return success ? 0 : 1;
}
