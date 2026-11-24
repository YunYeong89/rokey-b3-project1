/*********************************************************************
 * 
 * dsr_hardware2
 * Copyright (c) 2025 Doosan Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *********************************************************************/
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <atomic>
#include <cmath>
#include <exception>
#include <iterator>
#include <map>
#include <mutex>
#include <unistd.h>     
#include <math.h>

#include "dsr_hardware2/dsr_hw_interface2.h"
#include "dsr_hardware2/util.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dsr_msgs2/msg/force_telemetry.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "../../dsr_common2/include/DRFLEx.h"
#include "joint_servo.hpp"

using namespace std;
using namespace chrono;
using namespace DRAFramework;

bool g_bIsEmulatorMode = FALSE;
std::string g_model;
int m_nVersionDRCF;
constexpr size_t g_k_default_num_joint = 6;
constexpr size_t g_k_p3020_num_joint = 5;
constexpr size_t g_k_p3020_fixed_joint_index = 3;

std::map<std::string, CDRFLEx*> g_drfl_instances;
std::mutex g_drfl_map_mutex;
CDRFLEx* g_active_drfl = nullptr;
static std::map<CDRFLEx *, dsr_hardware2::JointServo *> joint_servos;

bool lock_direct_motion(CDRFLEx * drfl, std::unique_lock<std::mutex> & guard)
{
    std::lock_guard<std::mutex> lock(g_drfl_map_mutex);
    const auto servo = joint_servos.find(drfl);
    return servo == joint_servos.end() || servo->second->lock_direct_motion(guard);
}

bool stop_with_joint_servo(CDRFLEx * drfl, STOP_TYPE type)
{
    std::lock_guard<std::mutex> lock(g_drfl_map_mutex);
    const auto servo = joint_servos.find(drfl);
    return servo == joint_servos.end() ? drfl->stop(type) : servo->second->stop(type);
}

void* get_drfl(const char* robot_name = nullptr){
    std::lock_guard<std::mutex> lock(g_drfl_map_mutex);

    if (robot_name == nullptr || strlen(robot_name) == 0) {
        if (!g_drfl_instances.empty()) {
            return g_drfl_instances.begin()->second;
        }
        return g_active_drfl;
    }

    auto it = g_drfl_instances.find(robot_name);
    if (it != g_drfl_instances.end()) {
        return it->second;
    }

    return g_active_drfl;
}

namespace dsr_hardware2{

class ForceTelemetryPublisher
{
public:
    explicit ForceTelemetryPublisher(const rclcpp::Node::SharedPtr & node)
    {
        auto publisher = node->create_publisher<dsr_msgs2::msg::ForceTelemetry>(
            "realtime/force_telemetry", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile());
        publisher_ = std::make_unique<realtime_tools::RealtimePublisher<dsr_msgs2::msg::ForceTelemetry>>(publisher);
        // DDS subscriber discovery stays outside the hardware read loop.
        demand_timer_ = node->create_wall_timer(std::chrono::milliseconds(200),
            [publisher, subscribed = subscribed_]() {
                subscribed->store(publisher->get_subscription_count() > 0, std::memory_order_relaxed);
            });
    }

    void publish(const RT_OUTPUT_DATA_LIST & data, const rclcpp::Time & read_stamp)
    {
        if (!subscribed_->load(std::memory_order_relaxed)) {
            return;
        }
        const double stamp = data.time_stamp;
        if (!std::isfinite(stamp) || stamp <= 0.0 || stamp == last_stamp_ || !publisher_->trylock()) {
            return;
        }

        auto & msg = publisher_->msg_;
        msg.time_stamp = stamp;
        msg.read_stamp = read_stamp;
        const auto copy = [](const auto & source, auto & target) {
            std::copy(std::begin(source), std::end(source), target.begin());
        };
        copy(data.actual_joint_position, msg.actual_joint_position_deg);
        copy(data.actual_joint_position_abs, msg.actual_joint_position_abs_deg);
        copy(data.actual_joint_velocity, msg.actual_joint_velocity_deg_s);
        copy(data.actual_joint_velocity_abs, msg.actual_joint_velocity_abs_deg_s);
        copy(data.target_joint_acceleration, msg.target_joint_acceleration_deg_s2);
        copy(data.actual_tcp_position, msg.actual_tcp_pose_mm_deg);
        copy(data.actual_tcp_velocity, msg.actual_tcp_velocity_mm_deg_s);
        copy(data.actual_flange_position, msg.actual_flange_pose_mm_deg);
        copy(data.external_tcp_force, msg.external_tcp_wrench_n_nm);
        copy(data.raw_joint_torque, msg.raw_joint_torque_nm);
        copy(data.external_joint_torque, msg.external_joint_torque_nm);
        copy(data.gravity_torque, msg.gravity_torque_nm);
        copy(data.joint_temperature, msg.joint_temperature_c);
        auto row = msg.jacobian_matrix.begin();
        for (const auto & source_row : data.jacobian_matrix) {
            row = std::copy(std::begin(source_row), std::end(source_row), row);
        }
        msg.singularity = data.singularity;
        msg.solution_space = data.solution_space;
        msg.robot_mode = data.robot_mode;
        msg.robot_state = data.robot_state;
        msg.control_mode = data.control_mode;
        last_stamp_ = stamp;
        publisher_->unlockAndPublish();
    }

private:
    std::shared_ptr<std::atomic_bool> subscribed_ = std::make_shared<std::atomic_bool>(false);
    std::unique_ptr<realtime_tools::RealtimePublisher<dsr_msgs2::msg::ForceTelemetry>> publisher_;
    rclcpp::TimerBase::SharedPtr demand_timer_;
    double last_stamp_ = 0.0;
};

CallbackReturn DRHWInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    for(auto parameter : info.hardware_parameters) {
        if("host" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "host : %s", parameter.second.c_str());
            drcf_ip_ = parameter.second;
        } else if("rt_host" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "rt_host : %s", parameter.second.c_str());
            drcf_rt_ip_ = parameter.second;
        } else if("port" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "port : %s", parameter.second.c_str());
            drcf_port_ = std::stoi(parameter.second);
        } else if("mode" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "mode : %s", parameter.second.c_str());
            mode_ = parameter.second;
        } else if("model" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "model : %s", parameter.second.c_str());
            model_ = parameter.second;
            std::transform(model_.begin(), model_.end(), model_.begin(), ::tolower);
            g_model = model_;
        } else if ("update_rate" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"update_rate : %s", parameter.second.c_str());
            update_rate_ = std::stoi(parameter.second);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"), "Unexpected Parameter....\
                 key : %s, value : %s",parameter.first.c_str(), parameter.second.c_str());
        }
    }
    const size_t expected_num_joints = (model_ == "p3020") ? g_k_p3020_num_joint : g_k_default_num_joint;
    if (model_ == "p3020") {
        ignored_joints_.insert(g_k_p3020_fixed_joint_index); // ignore joint 4 for p3020
    }

    // Do not hard-fail by parameter count. Different xacro variants may include
    // additional keys while still providing all required fields.
    if(info.hardware_parameters.size() < 6) {
        RCLCPP_WARN(
            rclcpp::get_logger("dsr_hw_interface2"),
            "Hardware parameter count seems low (%zu). Continuing with parsed keys.",
            info.hardware_parameters.size());
    }

    // robot has 6 (or 5) joints and 2 interfaces
    joint_position_.assign(expected_num_joints, 0);
    joint_velocities_.assign(expected_num_joints, 0);
    joint_position_command_.assign(expected_num_joints, 0);
    joint_velocities_command_.assign(expected_num_joints, 0);

    if(expected_num_joints != info_.joints.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), 
                "[on_init] Hardware joint size : %zu, expected : %zu", info.joints.size(), expected_num_joints);
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), 
                    "[on_init] Hardware name : %s, type : %s, plugin name : %s",
                    info_.name.c_str(), info_.type.c_str(), info_.hardware_plugin_name.c_str());

    for (const auto & joint : info_.joints)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"), 
            "[on_init] joint name : %s, type : %s,",
            joint.name.c_str(), joint.type.c_str());
        for (const auto & interface : joint.state_interfaces)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"), 
                    "[on_init] joint state interface name : %s ", 
                    interface.name.c_str());
            if(interface.name == "effort") {
                RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), 
                    "[on_init] Not Implemented effort interface.. ignored");
                continue;
            }
            joint_interfaces[interface.name].push_back(joint.name);
        }
        for (const auto & interface : joint.command_interfaces)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"),
                "[on_init] joint command_interfaces name : %s ",
                interface.name.c_str());
            if(interface.name == "effort") {
                RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),
                            "[on_init] Not Implemented effort interface.. ignored");
                continue;
            }
            joint_comm_interfaces[interface.name].push_back(joint.name);
        }
    }


//-----------------------------------------------------------------------------------------------------
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    INITAILIZE");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");

    // Try to connect to DRCF for 10 (20 * 0.5) sec. 
    bool is_connected = false;
    for (size_t retry = 0; retry < 20; ++retry) {
        is_connected = m_Drfl.open_connection(drcf_ip_, drcf_port_);
        if(!is_connected) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"Connecting failure.. retry...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"Connected to DRCF");
        break;
    }
    if(!is_connected)
    {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"    DSRInterface::init() DRCF connecting ERROR!!!");
            return CallbackReturn::ERROR;
    }
    // Check whether DRCF loaded successfully for 10 sec..
    // Even thought, the server connected,
    // The drcf could still be in the booting process. 
    // Need to make sure it loaded successfully.
    // By making sure AUTHORITY and STANDBY_STATE.
    static bool get_control_access = false;
    static bool is_standby = false;
    get_control_access = false;
    is_standby = false;
    m_Drfl.set_on_monitoring_access_control([](const MONITORING_ACCESS_CONTROL access) {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"AUTHORITY : %s", to_str(access).c_str());
        if(MONITORING_ACCESS_CONTROL_GRANT == access) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL AUTHORITY GRANTED !!!");
            get_control_access = true;
            is_standby = false; // previous standby state before getting authority is definitely useless.
        }
        if(MONITORING_ACCESS_CONTROL_LOSS == access) {
            get_control_access = false;
            is_standby = false; // previous standby state after losing authority is definitely useless.
        }
    });
    m_Drfl.set_on_monitoring_state([](const ROBOT_STATE state) {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"ROBOT_STATE : %s", to_str(state).c_str());
        if(STATE_STANDBY == state) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL STATE_STANDBY !!!");
            is_standby = true;
        }else {
            is_standby = false;
        }
    });
    for (size_t retry = 0; retry < 10; ++retry, std::this_thread::sleep_for(std::chrono::milliseconds(1000))) {
        if(!get_control_access) {
            m_Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL MANAGE_ACCESS_CONTROL_FORCE_REQUEST called");
            continue;
        }
        if(!is_standby) {
            m_Drfl.set_robot_control(CONTROL_SERVO_ON);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL CONTROL_SERVO_ON called");
            continue;
        }
        if(get_control_access && is_standby)   break;
    }
    if(!(get_control_access && is_standby)) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL STATE CALL FAILURE !!");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    OPEN CONNECTION");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 

    //--- connect Emulator ? ------------------------------    
    if(mode_ == "virtual") {
        g_bIsEmulatorMode = true;
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Emulator Mode");
    } else {
        g_bIsEmulatorMode = false;
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Real Robot Mode");
    }

    //--- Get version -------------------------------------            
    SYSTEM_VERSION tSysVerion;
    memset(&tSysVerion, 0, sizeof(tSysVerion));
    assert(m_Drfl.get_system_version(&tSysVerion));

    //--- Get DRCF version & convert to integer  ----------            
    m_nVersionDRCF = 0; 
    int k=0;
    for(int i=strlen(tSysVerion._szController); i>0; i--)
            if(tSysVerion._szController[i]>='0' && tSysVerion._szController[i]<='9')
                    m_nVersionDRCF += (tSysVerion._szController[i]-'0')*pow(10.0,k++);
    if(m_nVersionDRCF < 100000) m_nVersionDRCF += 100000; 
                 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRCF version = %s",tSysVerion._szController);
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRFL version = %s",m_Drfl.get_library_version());
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    m_nVersionDRCF = %d", m_nVersionDRCF);  //ex> M2.40 = 120400, M2.50 = 120500  
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");

    m_Drfl.setup_monitoring_version(1); //Enabling extended monitoring functions

    if(m_Drfl.GetRobotState() != STATE_STANDBY)    {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Expected State : Stanby, \
            but Actual State : %s ", to_str(m_Drfl.GetRobotState()).c_str());
        return CallbackReturn::ERROR;
    }

    //--- Set Robot mode : MANUAL or AUTO
    if(!m_Drfl.SetRobotMode(ROBOT_MODE_AUTONOMOUS)) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "ROBOT_MODE_AUTONOMOUS Setting Failure !!"); 
        return CallbackReturn::ERROR;
    }

    //--- Set Robot mode : virual or real 
    ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
    if(mode_ == "real") eTargetSystem = ROBOT_SYSTEM_REAL;
    if(!m_Drfl.SetRobotSystem(eTargetSystem)) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "SetRobotSystem {%s} Setting Failure !!",
                mode_.c_str()); 
        return CallbackReturn::ERROR;
    }

    // Basically, Controller automatically servo-off after elapse time (5 min)
    // Deactivate it.
    m_Drfl.set_auto_servo_off(0, 5.0);

    // Virtual controller doesn't support real time connection.
    if(mode_ != "virtual") {
        if(m_nVersionDRCF >= 3000000) {
            drcf_ip_ = drcf_rt_ip_;
        }
        if (!m_Drfl.connect_rt_control(drcf_ip_)) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Unable to connect RT control stream");
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Connected RT control stream");
        const std::string version   = "v1.0";
        const float       period    = 0.001;
        const int         losscount = 4;
        if (!m_Drfl.set_rt_control_output(version, period, losscount)) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Unable to connect RT control stream");
            return CallbackReturn::FAILURE;
        }

        if (!m_Drfl.start_rt_control()) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Unable to start RT control");
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Setting velocity and acceleration limits");
        float limit[6] = {70.0f,70.0f,70.0f,70.0f,70.0f,70.0f};
        if (!m_Drfl.set_velj_rt(limit)) return CallbackReturn::ERROR;
        if (!m_Drfl.set_accj_rt(limit)) return CallbackReturn::ERROR;
    }

    m_Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE);
    {
        std::lock_guard<std::mutex> lock(g_drfl_map_mutex);
        g_drfl_instances[info_.name] = &m_Drfl;
        g_active_drfl = &m_Drfl;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn DRHWInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    const auto result = hardware_interface::SystemInterface::on_configure(previous_state);
    if (result == CallbackReturn::SUCCESS && mode_ == "real") {
        const auto node = get_node();
        if (!node) {
            RCLCPP_WARN(get_logger(), "Force telemetry unavailable: hardware node is missing");
            return result;
        }
        try {
            force_telemetry_ = std::make_unique<ForceTelemetryPublisher>(node);
            joint_servo_ = std::make_unique<JointServo>(m_Drfl, node);
            std::lock_guard<std::mutex> lock(g_drfl_map_mutex);
            joint_servos[&m_Drfl] = joint_servo_.get();
        } catch (const std::exception & error) {
            RCLCPP_WARN(get_logger(), "Force telemetry unavailable: %s", error.what());
        }
    }
    return result;
}

std::vector<hardware_interface::StateInterface> DRHWInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

    for(size_t i=0; i<joint_interfaces["position"].size(); i++) {
        state_interfaces.emplace_back(joint_interfaces["position"][i], "position", &joint_position_[i]);
    }
    // TODO(songms, yurirocha15) support velocity control.
    for(size_t i=0; i<joint_interfaces["velocity"].size(); i++) {
        state_interfaces.emplace_back(joint_interfaces["velocity"][i], "velocity", &joint_velocities_[i]);
    }
    // TODO(songms, yurirocha15) support effort control.
    for(size_t i=0; i<joint_interfaces["effort"].size(); i++) {
        state_interfaces.emplace_back(joint_interfaces["effort"][i], "effort", &joint_effort_[i]);
    }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DRHWInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    pre_joint_position_command_ = joint_position_command_;
    for(size_t i=0; i<joint_comm_interfaces["position"].size(); i++) {
        command_interfaces.emplace_back(joint_comm_interfaces["position"][i], "position", &joint_position_command_[i]);
    }
    for(size_t i=0; i<joint_comm_interfaces["velocity"].size(); i++) {
        command_interfaces.emplace_back(joint_comm_interfaces["velocity"][i], "velocity", &joint_velocities_command_[i]);
    }
    // TODO(songms, yurirocha15) support effort control.
    for(size_t i=0; i<joint_comm_interfaces["effort"].size(); i++) {
        command_interfaces.emplace_back(joint_comm_interfaces["effort"][i], "effort", &joint_effort_command_[i]);
    }
  return command_interfaces;
}


return_type DRHWInterface::read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    const size_t expected_num_joints = joint_position_.size();
    if(mode_ == "real") {
        const LPRT_OUTPUT_DATA_LIST data = m_Drfl.read_data_rt();
        if(nullptr == data) {
            if (joint_servo_ && joint_servo_->owns_motion()) {
                joint_servo_->stop(STOP_TYPE_SLOW);
            }
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),
                                    "[read] read_data_rt retrieves nullptr");
            return return_type::ERROR;
        }
        for(size_t idx_control=0, idx_ros=0; idx_control < g_k_default_num_joint; idx_control++) {
            if (ignored_joints_.find(idx_control) != ignored_joints_.end()) {
                continue;
            }
            joint_position_[idx_ros] = static_cast<float>(data->actual_joint_position[idx_control] * (M_PI / 180.0f));
            joint_velocities_[idx_ros] = static_cast<float>(data->actual_joint_velocity[idx_control] * (M_PI / 180.0f));
            idx_ros++;
        }
        if (force_telemetry_) {
            force_telemetry_->publish(*data, time);
        }
        if (joint_servo_) {
            joint_servo_->observe(*data);
        }
    }else if(mode_ == "virtual") {
        LPROBOT_POSE pose = m_Drfl.GetCurrentPose();
        if(nullptr == pose) {
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),
                                    "[read] GetCurrentPose retrieves nullptr");
            return return_type::ERROR; //? what effection of this to control node 
        }
        for(size_t idx_control=0, idx_ros=0; idx_control < g_k_default_num_joint; idx_control++) {
            if (ignored_joints_.find(idx_control) != ignored_joints_.end()) {
                continue;
            }
            joint_position_[idx_ros++] = deg2rad(pose->_fPosition[idx_control]);
        }
    }else {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), 
                "'mode' is neither 'real' nor 'virtual.'" );
    }
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[READ] joint_position_  : {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}"
    //     ,joint_position_[0]
    //     ,joint_position_[1]
    //     ,joint_position_[2]
    //     ,joint_position_[3]
    //     ,joint_position_[4]
    //     ,joint_position_[5]);
  return return_type::OK;
}

bool positionCommandRunning(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    double var = 0;
    for(size_t i=0; i<lhs.size(); i++) {
        var += abs(lhs[i] - rhs[i]);
    }
    return var >= 0.0001;
}

vector<vector<float>> joint_position_commands;

return_type DRHWInterface::write(const rclcpp::Time &, const rclcpp::Duration &dt)
{
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[WRITE] dt  : %.3f", float(dt.seconds()) );
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[WRITE] joint_position_command_  : {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}"
    //         ,joint_position_command_[0]
    //         ,joint_position_command_[1]
    //         ,joint_position_command_[2]
    //         ,joint_position_command_[3]
    //         ,joint_position_command_[4]
    //         ,joint_position_command_[5]);
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[WRITE] joint_velocities_command_  : {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}"
    //         ,joint_velocities_command_[0]
    //         ,joint_velocities_command_[1]
    //         ,joint_velocities_command_[2]
    //         ,joint_velocities_command_[3]
    //         ,joint_velocities_command_[4]
    //         ,joint_velocities_command_[5]);

    // Measure CPU loop duration for REAL servo timing
    auto now_cpu = std::chrono::steady_clock::now();
    double real_loop_dt = std::chrono::duration<double>(now_cpu - last_tick_).count();
    last_tick_ = now_cpu;
    if (joint_servo_ && joint_servo_->write()) {
        pre_joint_position_command_ = joint_position_command_;
        return return_type::OK;
    }

    // dt provided by controller_manager
    const double dt_sec = dt.seconds();

    // Expected control period from update_rate_ (Hz → seconds)
    double desired_period = 0.0;
    if (update_rate_ > 0)
        desired_period = 1.0 / static_cast<double>(update_rate_);

    // REAL mode: filter unstable dt cycles
    if (mode_ == "real" && desired_period > 0.0)
    {
        double min_dt = desired_period * 0.3;
        double max_dt = desired_period * 1.5;

        if (dt_sec < min_dt || dt_sec > max_dt)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("dsr_hw_interface2"),
                "[REAL] Skip dt=%.6f (expected=%.6f, allowed=[%.6f, %.6f])",
                dt_sec, desired_period, min_dt, max_dt
            );
            return return_type::OK;
        }
    }

    double effective_dt = dt_sec;
    if (mode_ == "virtual")
    {
        if (update_rate_ > 10)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"),"[DEBUG] update_rate_=%d Hz exceeds recommended 10 Hz",update_rate_);
        }

        double desired_period_virtual = (update_rate_ > 0) ? desired_period : 0.1;        // If update_rate_ is invalid, fallback to 10Hz (0.1s)
        double min_dt = desired_period_virtual * 0.3;
        double max_dt = desired_period_virtual * 1.5;

        if (dt_sec < min_dt || dt_sec > max_dt)
        {
            RCLCPP_DEBUG(
                rclcpp::get_logger("dsr_hw_interface2"),
                "[VIRTUAL] Skip dt=%.6f (expected=%.6f, allowed=[%.6f, %.6f])",
                dt_sec, desired_period_virtual, min_dt, max_dt
            );
            return return_type::OK;
        }
        effective_dt = dt_sec;
    }

    // Accumulate simulated time
    total_time_sec_ += effective_dt;

    if (positionCommandRunning(pre_joint_position_command_, joint_position_command_))
    {
        std::unique_lock<std::mutex> guard;
        if (joint_servo_ && !joint_servo_->lock_direct_motion(guard)) {
            return return_type::OK;
        }
        if (idle_)
        {
            m_Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
            idle_ = false;
        }

        // Convert rad → deg
        float pos[g_k_default_num_joint];
        float vel[g_k_default_num_joint];
        for (size_t idx_control = 0, idx_ros = 0; idx_control < g_k_default_num_joint; idx_control++)
        {
            if (ignored_joints_.find(idx_control) != ignored_joints_.end()) {
                pos[idx_control] = 0.0f;
                vel[idx_control] = 0.0f;
            }
            else {
                pos[idx_control] = static_cast<float>(joint_position_command_[idx_ros] * (180.0 / M_PI));
                vel[idx_control] = static_cast<float>(joint_velocities_command_[idx_ros] * (180.0 / M_PI));
                idx_ros++;
            }
        }

        // Select control API
        std::string cmd_type;
        if (mode_ == "real")
        {
            float acc[g_k_default_num_joint] = {0,0,0,0,0,0};
            const float margin = 20.0f;
            float servo_time = static_cast<float>(real_loop_dt * margin);

            m_Drfl.servoj_rt(pos, vel, acc, servo_time);
            cmd_type = "servoj_rt";
        }
        else  // virtual
        {
            float target_vel_acc[g_k_default_num_joint] = {70,70,70,70,70,70};
            m_Drfl.amovej(pos, target_vel_acc, target_vel_acc);
            cmd_type = "amovej";
        }

        // Debug logging
        // RCLCPP_INFO(
        //     rclcpp::get_logger("dsr_hw_interface2"),
        //     "[WRITE] t=%.6f | mode_=%s | dt=%.6f → eff=%.6f\n"
        //     "        update_rate_=%d (period=%.6f)\n"
        //     "        pos={%.3f %.3f %.3f %.3f %.3f %.3f} deg\n"
        //     "        vel={%.3f %.3f %.3f %.3f %.3f %.3f} deg/s\n"
        //     "        cmd=%s",
        //     total_time_sec,
        //     mode_.c_str(),
        //     dt_sec, effective_dt,
        //     update_rate_, desired_period,
        //     pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
        //     vel[0], vel[1], vel[2], vel[3], vel[4], vel[5],
        //     cmd_type.c_str()
        // );

        pre_joint_position_command_ = joint_position_command_;
        return return_type::OK;
    }
    idle_ = true;
    pre_joint_position_command_ = joint_position_command_;
    return return_type::OK;
}


DRHWInterface::~DRHWInterface()
{
    {
        std::lock_guard<std::mutex> lock(g_drfl_map_mutex);
        joint_servos.erase(&m_Drfl);
    }
    if (joint_servo_ && joint_servo_->owns_motion()) {
        joint_servo_->stop(STOP_TYPE_SLOW);
    }
    m_Drfl.stop_rt_control();
    // To-do : Update disconnection function in controller version v3.6
    // m_Drfl.disconnect_rt_control();
    m_Drfl.close_connection();

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    CONNECTION IS CLOSED");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dsr_hardware2::DRHWInterface, hardware_interface::SystemInterface)
