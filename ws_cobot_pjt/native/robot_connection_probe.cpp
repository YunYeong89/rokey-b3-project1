#include "DRFLEx.h"
#include <chrono>
#include <iostream>
#include <thread>

// Query versions and receive RT telemetry without requesting control or servo-on.
class RobotConnection {
public:
    ~RobotConnection() {
        if (streaming_) robot_.stop_rt_control();
        if (rt_connected_) robot_.disconnect_rt_control();
        robot_.close_connection();
    }

    bool check(const char* host, const char* rt_host) {
        if (!robot_.open_connection(host, 12345)) {
            std::cerr << "DRFL TCP connection failed\n";
            return false;
        }
        SYSTEM_VERSION version{};
        if (!robot_.get_system_version(&version)) return false;
        const int state_before = robot_.get_robot_state();
        std::cout << "Controller=" << version._szController << std::endl;
        rt_connected_ = robot_.connect_rt_control(rt_host);
        if (!rt_connected_) return false;
        if (!robot_.set_rt_control_output("v1.0", 0.001f, 4)) return false;
        streaming_ = robot_.start_rt_control();
        if (!streaming_) return false;

        int updates = 0;
        double previous = 0;
        for (int i = 0; i < 100; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            auto data = robot_.read_data_rt();
            if (data && data->time_stamp > previous) {
                previous = data->time_stamp;
                ++updates;
            }
        }
        std::cout << "COBOT_CONNECTION_RESULT={\"controller_version\":\""
                  << version._szController << "\",\"state_before\":" << state_before
                  << ",\"state_after\":" << robot_.get_robot_state()
                  << ",\"rt_observed_updates\":" << updates
                  << ",\"motion_commands_sent\":false}" << std::endl;
        return updates > 0;
    }

private:
    DRAFramework::CDRFLEx robot_;
    bool rt_connected_ = false;
    bool streaming_ = false;
};

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    RobotConnection connection;
    return connection.check(argv[1], argv[2]) ? 0 : 1;
}
