#include "DRFLEx.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class EmulatorBenchmark {
    DRAFramework::CDRFLEx robot;
    inline static std::atomic<bool> authority{false};
    inline static std::mutex monitoring_mutex;
    inline static std::vector<double> monitoring_times;
    std::array<float, 6> origin{};
    bool connected = false;
    static double milliseconds(Clock::duration d) {
        return std::chrono::duration<double, std::milli>(d).count();
    }
    static void require(bool ok, const char* message) {
        if (!ok) throw std::runtime_error(message);
    }
    void connect(const std::string& ip, int port) {
        // Only Docker Desktop loopback publication is allowed for this diagnostic.
        require(ip == "host.docker.internal" && port == 12349,
                "This diagnostic is restricted to the local emulator port");
        robot.set_on_monitoring_access_control([](MONITORING_ACCESS_CONTROL access) {
            authority = access == MONITORING_ACCESS_CONTROL_GRANT;
        });
        robot.set_on_monitoring_data_ex([](const LPMONITORING_DATA_EX) {
            const double t = milliseconds(Clock::now().time_since_epoch());
            std::lock_guard<std::mutex> lock(monitoring_mutex);
            monitoring_times.push_back(t);
        });
        for (int attempt = 0; attempt < 40 && !connected; ++attempt) {
            connected = robot.open_connection(ip, port);
            if (!connected) std::this_thread::sleep_for(500ms);
        }
        require(connected, "Connection failed after startup wait");
        connected = true;
        SYSTEM_VERSION version{};
        require(robot.get_system_version(&version), "Version query failed");
        std::cout << "Controller=" << version._szController << std::endl;
        // A physical controller must never receive motion from this diagnostic.
        require(robot.get_robot_system() == ROBOT_SYSTEM_VIRTUAL,
                "Controller is not in virtual mode");
        robot.setup_monitoring_version(1);
        for (int i = 0; i < 20 && !authority; ++i) {
            robot.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            std::this_thread::sleep_for(250ms);
        }
        require(authority, "Control authority unavailable");
        require(robot.set_robot_control(CONTROL_SERVO_ON), "Servo enable failed");
        for (int i = 0; i < 40 && robot.get_robot_state() != STATE_STANDBY; ++i)
            std::this_thread::sleep_for(100ms);
        require(robot.get_robot_state() == STATE_STANDBY, "Standby unavailable");
        require(robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS), "Autonomous mode failed");
        const auto pose = robot.get_current_posj();
        require(pose != nullptr, "Initial pose unavailable");
        std::copy_n(pose->_fPosition, 6, origin.begin());
    }
public:
    ~EmulatorBenchmark() {
        if (connected) {
            robot.stop(STOP_TYPE_QUICK);
            robot.close_connection();
        }
    }
    void run(const std::string& ip, int port, int seconds, const std::string& output) {
        connect(ip, port);
        std::ofstream samples(output + ".csv");
        std::ofstream requests(output + "_requests.csv");
        samples << "elapsed_ms,interval_ms,lateness_ms,pose_call_ms,valid,j1_deg\n";
        requests << "elapsed_ms,request_ms,valid\n";
        auto target = origin;
        target[0] += 2.0f;
        const auto start = Clock::now();
        auto next = start;
        auto previous = start;
        bool outward = false, returned = false;
        int failures = 0;
        double max_excursion = 0, final_error = 0;
        long skipped = 0;
        double next_request = 1000;
        while (Clock::now() - start < std::chrono::seconds(seconds)) {
            std::this_thread::sleep_until(next);
            const auto before = Clock::now();
            const double elapsed = milliseconds(before - start);
            const auto pose = robot.get_current_posj();
            const auto after = Clock::now();
            if (!pose) ++failures;
            const double j1 = pose ? pose->_fPosition[0] : NAN;
            if (pose) {
                max_excursion = std::max(max_excursion, std::abs(j1 - origin[0]));
                final_error = std::abs(j1 - origin[0]);
            }
            samples << elapsed << ',' << milliseconds(before - previous) << ','
                    << milliseconds(before - next) << ',' << milliseconds(after - before)
                    << ',' << (pose != nullptr) << ',' << j1 << '\n';
            previous = before;
            if (!outward && elapsed >= 1000) {
                require(robot.amovej(target.data(), 5.f, 10.f, 2.f), "Outward move rejected");
                outward = true;
            }
            if (!returned && elapsed >= 6000) {
                require(robot.amovej(origin.data(), 5.f, 10.f, 2.f), "Return move rejected");
                returned = true;
            }
            if (elapsed >= next_request) {
                SYSTEM_VERSION version{};
                auto sent = Clock::now();
                const bool ok = robot.get_system_version(&version);
                requests << elapsed << ',' << milliseconds(Clock::now() - sent) << ',' << ok << '\n';
                if (!ok) ++failures;
                next_request = elapsed + 1000;
            }
            next += 10ms;
            while (next < Clock::now()) { next += 10ms; ++skipped; }
        }
        robot.close_connection();
        connected = false;
        std::ofstream monitoring(output + "_monitoring.csv");
        monitoring << "interval_ms\n";
        {
            std::lock_guard<std::mutex> lock(monitoring_mutex);
            for (size_t i = 1; i < monitoring_times.size(); ++i)
                monitoring << monitoring_times[i] - monitoring_times[i-1] << '\n';
        }
        std::ofstream result(output + "_motion.json");
        const bool passed = outward && returned && failures == 0 && max_excursion >= 1.5 && final_error <= 0.1;
        result << "{\"duration_s\":" << seconds << ",\"failures\":" << failures
               << ",\"max_excursion_deg\":" << max_excursion
               << ",\"final_error_deg\":" << final_error
               << ",\"skipped_10ms_slots\":" << skipped
               << ",\"passed\":" << (passed ? "true" : "false") << "}\n";
        require(passed, "Motion or sampling verification failed; inspect result files");
    }
};

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: benchmark host.docker.internal 12349 SECONDS OUTPUT_PREFIX\n";
        return 2;
    }
    try {
        EmulatorBenchmark benchmark;
        benchmark.run(argv[1], std::stoi(argv[2]), std::stoi(argv[3]), argv[4]);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
