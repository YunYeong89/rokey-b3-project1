#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>

#include "rokey_interfaces/msg/joint_servo_setpoint.hpp"
#include "rokey_interfaces/msg/joint_servo_status.hpp"
#include "rokey_interfaces/srv/joint_servo_session.hpp"

namespace dsr_hardware2 {

class JointServo
{
    using Clock = std::chrono::steady_clock;
    using Setpoint = rokey_interfaces::msg::JointServoSetpoint;
    using Status = rokey_interfaces::msg::JointServoStatus;
    using Session = rokey_interfaces::srv::JointServoSession;

public:
    JointServo(CDRFLEx & drfl, const rclcpp::Node::SharedPtr & node) : drfl_(drfl)
    {
        auto qos = rclcpp::QoS(1);
        status_pub_ = node->create_publisher<Status>("joint_servo/status", qos);
        setpoint_sub_ = node->create_subscription<Setpoint>(
            "joint_servo/setpoint", qos.best_effort(),
            [this](const Setpoint & point) { receive(point); });
        session_srv_ = node->create_service<Session>(
            "joint_servo/session", [this](const std::shared_ptr<Session::Request> request,
            std::shared_ptr<Session::Response> response) { session(*request, *response); });
        timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [this]() {
            Status status;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                status.run_id = run_id_;
                status.active = active_;
                status.stopped = !owns_motion_ && stationary(Clock::now());
                status.reason = reason_;
            }
            status_pub_->publish(status);
        });
    }

    bool owns_motion()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return owns_motion_;
    }

    bool lock_direct_motion(std::unique_lock<std::mutex> & guard)
    {
        guard = std::unique_lock<std::mutex>(direct_motion_, std::try_to_lock);
        return guard && !owns_motion();
    }

    bool stop(STOP_TYPE type)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (owns_motion_) {
            active_ = false;
            reason_ = "DSR stop requested";
            stop_sent_ = drfl_.stop(type);
            last_stop_attempt_ = Clock::now();
            return stop_sent_;
        }
        return drfl_.stop(type);
    }

    void observe(const RT_OUTPUT_DATA_LIST & data)
    {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock || !std::isfinite(data.time_stamp) || data.time_stamp <= 0. ||
            data.time_stamp == sample_time_ || (active_ && data.time_stamp < sample_time_)) {
            return;
        }
        sample_time_ = data.time_stamp;
        last_feedback_ = Clock::now();
        mode_ = data.control_mode;
        robot_mode_ = data.robot_mode;
        state_ = data.robot_state;
        std::copy_n(data.actual_joint_position, NUM_JOINT, actual_.begin());
        std::copy_n(data.actual_joint_velocity, NUM_JOINT, velocity_.begin());
    }

    bool write()
    {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock) {
            return true;
        }
        if (!owns_motion_) {
            return false;
        }
        const auto now = Clock::now();
        if (active_ && (!fresh(now) || now - last_command_ > std::chrono::milliseconds(120) ||
            mode_ != CONTROL_MODE_POSITION || robot_mode_ != ROBOT_MODE_AUTONOMOUS ||
            (state_ != STATE_STANDBY && state_ != STATE_MOVING))) {
            active_ = false;
            reason_ = "Joint servo feedback, command or controller mode became invalid";
        }
        if (active_ && sequence_ != 0) {
            float acceleration[NUM_JOINT] = {};
            if (drfl_.servoj_rt(target_.data(), feed_.data(), acceleration, time_)) {
                return true;
            }
            active_ = false;
            reason_ = "Doosan rejected servoj_rt";
        }
        if (!active_) {
            if (!stop_sent_ && now - last_stop_attempt_ >= std::chrono::milliseconds(100)) {
                last_stop_attempt_ = now;
                stop_sent_ = drfl_.stop(STOP_TYPE_SLOW);
            }
            if (last_feedback_ > last_stop_attempt_ && stationary(now)) {
                owns_motion_ = false;
            }
        }
        return true;
    }

private:
    bool fresh(Clock::time_point now) const
    {
        return now - last_feedback_ <= std::chrono::milliseconds(100);
    }

    bool stationary(Clock::time_point now) const
    {
        return fresh(now) && state_ == STATE_STANDBY &&
            std::all_of(velocity_.begin(), velocity_.end(), [](float v) { return std::abs(v) < 0.5f; });
    }

    void session(const Session::Request & request, Session::Response & response)
    {
        std::unique_lock<std::mutex> motion(direct_motion_, std::defer_lock);
        if (request.start && !motion.try_lock()) {
            response.message = "A DSR motion command is in progress";
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!request.start) {
            if (request.run_id.empty() || request.run_id != run_id_) {
                response.message = "Joint servo session does not own the robot";
                return;
            }
            active_ = false;
            response.success = true;
            return;
        }
        const auto now = Clock::now();
        if (request.run_id.empty() || owns_motion_ || !stationary(now) ||
            mode_ != CONTROL_MODE_POSITION || robot_mode_ != ROBOT_MODE_AUTONOMOUS) {
            response.message = "Joint servo requires a stationary robot in automatic position control";
            return;
        }
        run_id_ = request.run_id;
        active_ = owns_motion_ = true;
        stop_sent_ = false;
        sequence_ = 0;
        last_command_ = now;
        reason_.clear();
        response.success = true;
    }

    void receive(const Setpoint & point)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double age = sample_time_ - point.source_time_s;
        if (!active_ || point.run_id != run_id_ || point.sequence <= sequence_ ||
            !std::isfinite(age) || age < -0.005 || age > 0.10) {
            return;
        }
        if (!(point.servo_time_s >= 0.01 && point.servo_time_s <= 0.1)) {
            active_ = false;
            reason_ = "Joint servo target time outside 0.01-0.1 s";
            return;
        }
        for (size_t i = 0; i < NUM_JOINT; ++i) {
            const auto p = point.position_deg[i], v = point.velocity_deg_s[i];
            if (!std::isfinite(p) || !std::isfinite(v) || !std::isfinite(actual_[i]) || std::abs(p - actual_[i]) > 5. ||
                std::abs(v) > 25.) {
                active_ = false;
                reason_ = "Joint servo position or speed outside the stream limits";
                return;
            }
        }
        std::copy(point.position_deg.begin(), point.position_deg.end(), target_.begin());
        std::copy(point.velocity_deg_s.begin(), point.velocity_deg_s.end(), feed_.begin());
        time_ = static_cast<float>(point.servo_time_s);
        sequence_ = point.sequence;
        last_command_ = Clock::now();
    }

    CDRFLEx & drfl_;
    std::mutex mutex_, direct_motion_;
    std::string run_id_, reason_;
    bool active_ = false, owns_motion_ = false, stop_sent_ = false;
    uint64_t sequence_ = 0;
    double sample_time_ = 0.;
    float time_ = 0.04f;  // servoj_rt target time (s), from the setpoint (real_stream.servo_time_s)
    int state_ = -1, mode_ = -1, robot_mode_ = -1;
    std::array<float, NUM_JOINT> actual_{}, velocity_{}, target_{}, feed_{};
    Clock::time_point last_feedback_{}, last_command_{}, last_stop_attempt_{};
    rclcpp::Publisher<Status>::SharedPtr status_pub_;
    rclcpp::Subscription<Setpoint>::SharedPtr setpoint_sub_;
    rclcpp::Service<Session>::SharedPtr session_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dsr_hardware2
