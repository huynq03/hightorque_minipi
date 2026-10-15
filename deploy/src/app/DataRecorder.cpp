#include "app/DataRecorder.h"

#include "FSM/CtrlFSM.h"

#include <sensor_msgs/JointState.h>

#include <chrono>
#include <filesystem>
#include <iomanip>

namespace deploy
{

DataRecorder::DataRecorder(const Config& cfg, const ControlContext& ctx, const CtrlFSM& fsm,
                           std::vector<std::string> joint_names)
    : cfg_(cfg), ctx_(ctx), fsm_(fsm), names_(std::move(joint_names))
{
}

DataRecorder::~DataRecorder() { stop(); }

bool DataRecorder::start(ros::NodeHandle& nh, std::string& error)
{
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(cfg_.csv_path).parent_path(), ec);
    csv_.open(cfg_.csv_path);
    if (!csv_ || !(cfg_.hz > 0.0))
    {
        error = !csv_ ? "cannot create " + cfg_.csv_path : std::string("record_hz must be > 0");
        return false;
    }
    writeHeader_();
    if (cfg_.publish)
    {
        pub_state_ = nh.advertise<sensor_msgs::JointState>("mini_pi/joint_states", 10);
        pub_cmd_ = nh.advertise<sensor_msgs::JointState>("mini_pi/joint_command", 10);
    }
    running_ = true;
    thread_ = std::thread(&DataRecorder::loop_, this);
    ROS_INFO("DataRecorder: %.0f Hz -> %s%s", cfg_.hz, cfg_.csv_path.c_str(),
             cfg_.publish ? ", topics mini_pi/joint_states, mini_pi/joint_command" : "");
    return true;
}

void DataRecorder::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (csv_.is_open()) csv_.close();
}

void DataRecorder::writeHeader_()
{
    csv_ << "t,wall_time,state,faulted,cmd_valid,roll,pitch,yaw,gyro_x,gyro_y,gyro_z,"
            "quat_x,quat_y,quat_z,quat_w";
    for (const char* f : {"q", "dq", "tau", "q_target", "kp", "kd", "delta"})
        for (const auto& n : names_) csv_ << "," << f << "_" << n;
    csv_ << ",policy_seq";
    for (std::size_t i = 0; i < NUM_DOF; ++i) csv_ << ",action_" << i;   // policy order
    csv_ << "\n";
}

void DataRecorder::loop_()
{
    ros::WallRate rate(cfg_.hz);
    const auto t0 = Clock::now();
    auto last_flush = t0;
    std::uint64_t last_tick = 0;
    // RobotState::robot_tau keeps the MOTOR's sign (JointMapper reproduces the
    // vendor controller); logged in the joint's sign so it compares with q and
    // the PD torque.
    JointArray tau_sign;
    tau_sign.fill(1.0);
    if (ctx_.mapper->mode() == TransformMode::Full)
        for (std::size_t i = 0; i < NUM_DOF; ++i) tau_sign[i] = ctx_.mapper->direction()[i];
    JointArray tau{};
    RobotState s;
    RobotCommand c;
    sensor_msgs::JointState js, jc;
    js.name = names_;
    jc.name = names_;
    for (auto* m : {&js, &jc})
    {
        m->position.resize(NUM_DOF);
        m->velocity.resize(NUM_DOF);
        m->effort.resize(NUM_DOF);
    }

    while (running_ && ros::ok())
    {
        if (fsm_.snapshot(s, c) && c.seq != last_tick)
        {
            last_tick = c.seq;
            const std::string state = fsm_.currentStateName();
            const RLPolicyRunner* pol = ctx_.policyFor(state);
            PolicyOutput po = (pol && pol->running()) ? pol->latest() : PolicyOutput{};
            const bool faulted = ctx_.safety->faulted();
            const double t = std::chrono::duration<double>(Clock::now() - t0).count();
            for (std::size_t i = 0; i < NUM_DOF; ++i) tau[i] = tau_sign[i] * s.robot_tau[i];

            {
                csv_ << std::fixed << std::setprecision(4) << t << "," << std::setprecision(3)
                     << ros::WallTime::now().toSec() << "," << state << "," << faulted << ","
                     << c.valid << std::setprecision(5);
                for (double v : s.rpy) csv_ << "," << v;
                for (double v : s.angular_velocity) csv_ << "," << v;
                for (double v : s.orientation) csv_ << "," << v;
                for (const JointArray* a : {&s.robot_q, &s.robot_dq, &tau, &c.q, &c.kp, &c.kd,
                                            &po.delta})
                    for (double v : *a) csv_ << "," << v;
                csv_ << "," << (po.valid ? po.seq : 0);
                for (double v : po.action) csv_ << "," << v;
                csv_ << "\n";
                if (Clock::now() - last_flush > std::chrono::seconds(1))
                {
                    csv_.flush();
                    last_flush = Clock::now();
                }
            }
            if (cfg_.publish)
            {
                js.header.stamp = jc.header.stamp = ros::Time::now();
                for (std::size_t i = 0; i < NUM_DOF; ++i)
                {
                    js.position[i] = s.robot_q[i];
                    js.velocity[i] = s.robot_dq[i];
                    js.effort[i] = tau[i];
                    jc.position[i] = c.q[i];
                    jc.velocity[i] = 0.0;
                    jc.effort[i] = c.kp[i] * (c.q[i] - s.robot_q[i]) - c.kd[i] * s.robot_dq[i];
                }
                pub_state_.publish(js);
                pub_cmd_.publish(jc);
            }
        }
        rate.sleep();
    }
}

} // namespace deploy
