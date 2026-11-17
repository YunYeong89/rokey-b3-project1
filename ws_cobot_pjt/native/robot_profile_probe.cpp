#include "DRFLEx.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

// Passive inspection: no authority, servo, mode, tool or motion commands.
class RobotProfiles {
public:
    ~RobotProfiles() { robot_.close_connection(); }

    bool inspect(const char* host) {
        robot_.set_on_monitoring_data_ex(receive);
        if (!robot_.open_connection(host, 12345)) return false;
        SYSTEM_VERSION version{};
        if (!robot_.get_system_version(&version)) return false;
        const int state_before = robot_.get_robot_state();
        // Select this connection's telemetry format; robot configuration is unchanged.
        robot_.setup_monitoring_version(1);
        for (int i = 0; i < 40; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::lock_guard<std::mutex> lock(mutex_);
            if (updates_ >= 3) break;
        }
        const auto tcp = robot_.get_tcp();
        const auto tool = robot_.get_tool();
        std::cout << std::setprecision(9)
                  << "COBOT_PROFILE_IDENTITY={\"controller_version\":"
                  << std::quoted(version._szController) << ",\"tcp\":"
                  << std::quoted(tcp) << ",\"tool\":" << std::quoted(tool)
                  << ",\"robot_mode\":" << robot_.get_robot_mode()
                  << ",\"robot_system\":" << robot_.get_robot_system()
                  << ",\"state_before\":" << state_before << "}" << std::endl;

        const auto config = robot_.get_safety_configuration();
        std::cout << "COBOT_PROFILE_RESULT={\"controller_version\":"
                  << std::quoted(version._szController) << ",\"tcp\":"
                  << std::quoted(tcp) << ",\"tool\":" << std::quoted(tool)
                  << ",\"state_before\":" << state_before
                  << ",\"state_after\":" << robot_.get_robot_state()
                  << ",\"control_commands_sent\":false,\"safety_configuration\":";
        if (config) {
            std::cout << "{\"data_version\":" << config->_iDataVersion
                      << ",\"active_tcp\":" << std::quoted(config->_szActiveTcp)
                      << ",\"active_tool\":" << std::quoted(config->_szActiveTool)
                      << ",\"safety_tcp\":";
            print_tcp(config->_tTcp);
            std::cout << ",\"safety_tool\":";
            print_tool(config->_tTool);
            std::cout << ",\"tcps\":[";
            const int tcp_count = config->_tConfigTCP._iToolCount;
            for (int i = 0; i < tcp_count && i < MAX_CONFIG_TCP_SIZE; ++i) {
                if (i) std::cout << ',';
                print_tcp(config->_tConfigTCP._tTooList[i]);
            }
            std::cout << "],\"tools\":[";
            const int tool_count = config->_tConfigTool._iToolCount;
            for (int i = 0; i < tool_count && i < MAX_CONFIG_TOOL_SIZE; ++i) {
                if (i) std::cout << ',';
                print_tool(config->_tConfigTool._tTooList[i]);
            }
            std::cout << "]}";
        } else {
            std::cout << "null";
        }
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << ",\"monitoring_updates\":" << updates_ << ",\"force_control\":";
        if (updates_) {
            const auto& force = monitoring_._tMiscEx._tCtrlEx;
            std::cout << "{\"ref\":" << int(force._iReferenceCoord) << ",\"mode\":";
            array(force._iForceControlMode);
            std::cout << ",\"wrench_tool\":";
            array(force._fToolActualETT);
            std::cout << ",\"wrench_base\":";
            array(monitoring_._tCtrl._tTorque._fActualETT);
            std::cout << ",\"tcp_pose_base\":";
            array(monitoring_._tCtrl._tTask._fActualPos[0]);
            std::cout << ",\"flange_pose_base\":";
            array(monitoring_._tCtrl._tTask._fActualPos[1]);
            std::cout << '}';
        } else {
            std::cout << "null";
        }
        std::cout << "}" << std::endl;
        return true;
    }

private:
    template<typename T, size_t N> static void array(const T (&values)[N]) {
        std::cout << '[';
        for (size_t i = 0; i < N; ++i) {
            if (i) std::cout << ',';
            std::cout << +values[i];
        }
        std::cout << ']';
    }

    static void print_tcp(const CONFIG_TCP_SYMBOL& tcp) {
        std::cout << "{\"name\":" << std::quoted(tcp._szSymbol) << ",\"pos_mm_deg\":";
        array(tcp._tTCP._fTargetPos);
        std::cout << '}';
    }

    static void print_tool(const CONFIG_TOOL_SYMBOL& tool) {
        std::cout << "{\"name\":" << std::quoted(tool._szSymbol)
                  << ",\"weight_kg\":" << tool._tTool._fWeight << ",\"cog_mm\":";
        array(tool._tTool._fXYZ);
        std::cout << ",\"inertia\":";
        array(tool._tTool._fInertia);
        std::cout << '}';
    }

    static void receive(const LPMONITORING_DATA_EX data) {
        std::lock_guard<std::mutex> lock(mutex_);
        monitoring_ = *data;
        ++updates_;
    }

    DRAFramework::CDRFLEx robot_;
    static std::mutex mutex_;
    static MONITORING_DATA_EX monitoring_;
    static int updates_;
};

std::mutex RobotProfiles::mutex_;
MONITORING_DATA_EX RobotProfiles::monitoring_{};
int RobotProfiles::updates_ = 0;

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    RobotProfiles profiles;
    return profiles.inspect(argv[1]) ? 0 : 1;
}
