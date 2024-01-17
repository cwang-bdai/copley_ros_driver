#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include <array>
#include <atomic>
#include <csignal>
#include <cmath>
#include <iostream>

#include <cstdio>
#include <cstdlib>

// #include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "CML.h"
#include "ecat_linux.h"

#define AXIS_COUNT 1
#define PI_VALUE 3.14159265358979323846

using std::placeholders::_1;
using namespace std::chrono_literals;
CML_NAMESPACE_USE();

class CopleyDriver : public rclcpp::Node
{
public:
    CopleyDriver();
    ~CopleyDriver();
    void motor_run();
    void publish_motor_state();
    void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
    void drive_init_();
    void motor_state_update_();
    void showerr_(const Error *err, const char *str);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;

    std::shared_ptr<std::thread> copley_th_;
    sensor_msgs::msg::JointState msg_;

    int32 counts_per_rev_[AXIS_COUNT];

    double counts_per_rad_[AXIS_COUNT];
    double counts_sec_per_rad_[AXIS_COUNT];
    double units_per_nm[AXIS_COUNT];

    std::vector<std::string> joint_names_ = {"motor1"};
    std::array<double, AXIS_COUNT> pos_;
    std::array<double, AXIS_COUNT> vel_;
    std::array<double, AXIS_COUNT> tau_;

    int32 pos_d_[AXIS_COUNT];
    int32 vel_d_[AXIS_COUNT];
    int32 tau_d_[AXIS_COUNT];

    const Error *err_ = 0;
    EtherCAT net_;
    int16 canNodeID_ = 0;
    Amp amp_[AXIS_COUNT];

    double eps;
};