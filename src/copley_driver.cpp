#include "copley_driver.h"

// volatile std::sig_atomic_t
int done = 0;
void signal_callback_handler(int signum)
{
    std::cout << "CTRL+C interrupted. " << std::endl;
    // Terminate program
    if (signum == SIGINT)
    {
        done = 1;
    }
    rclcpp::shutdown();
}

CopleyDriver::CopleyDriver() : Node("copley_driver")
{
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/copley_joint_states", 1);
    subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/copley_desired_pos", 1, std::bind(&CopleyDriver::command_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(5ms, std::bind(&CopleyDriver::publish_motor_state, this));
    copley_th_ = std::make_shared<std::thread>(&CopleyDriver::motor_run, this);
}

CopleyDriver::~CopleyDriver()
{
    copley_th_->join();
}

void CopleyDriver::showerr_(const Error *err, const char *str)
{
    if (err)
    {
        printf("Error %s: %s\n", str, err->toString());
        exit(1);
    }
}

void CopleyDriver::drive_init_()
{
    // array to store the counts per rev for each axis
    for (int i = 0; i < AXIS_COUNT; i++)
    {
        // clear latched faults
        err_ = amp_[i].ClearFaults();
        showerr_(err_, "clearing faults");

        // enable the axis
        err_ = amp_[i].Enable();
        showerr_(err_, "enabling axis");

        MtrInfo mtrInfoObj;
        amp_[i].GetMtrInfo(mtrInfoObj);
        counts_per_rev_[i] = mtrInfoObj.ctsPerRev;

        counts_per_rad_[i] = counts_per_rev_[i] / PI_VALUE;
        counts_sec_per_rad_[i] = counts_per_rev_[i] / PI_VALUE / 10;
        units_per_nm[i] = 1000;
    }

    // Home the motor.
    HomeConfig hcfg;
    hcfg.method = CHM_NONE;
    hcfg.velFast = 100000;
    hcfg.velSlow = 50000;
    hcfg.accel = 90000;
    hcfg.offset = 0;

    // Send the command to home each motor
    for (int i = 0; i < AXIS_COUNT; i++)
    {
        err_ = amp_[i].GoHome(hcfg);
        showerr_(err_, "Going home");
        err_ = amp_[i].WaitMoveDone(20000);
        showerr_(err_, "waiting on home");
    }

    for (int i = 0; i < AXIS_COUNT; i++)
    {
        pos_[i] = 0.0;
        vel_[i] = 0.0;
        tau_[i] = 0.0;

        pos_d_[i] = 0.0;
        vel_d_[i] = 0.0;
        tau_d_[i] = 0.0;
    }

    eps = 1e-6;
}

void CopleyDriver::motor_state_update_()
{
    uunit x[AXIS_COUNT], v[AXIS_COUNT];
    int16 tau[AXIS_COUNT];

    for (int i = 0; i < AXIS_COUNT; i++)
    {
        amp_[i].GetPositionMotor(x[i]);
        amp_[i].GetVelocityActual(v[i]);
        amp_[i].GetTorqueActual(tau[i]);

        pos_[i] = double(x[i] / counts_per_rad_[i]);
        vel_[i] = double(v[i] / counts_sec_per_rad_[i]);
        tau_[i] = double(tau[i] / units_per_nm[i]);
    }
}

void CopleyDriver::motor_run()
{
    LinuxEcatHardware hw("enx3c18a0d4bdc4");
    err_ = net_.Open(hw);
    showerr_(err_, "Opening network");
    err_ = amp_[0].Init(net_, canNodeID_);
    showerr_(err_, "Initting amp");

    drive_init_();
    std::cout << "Drive init finished." << std::endl;
    std::cout << "Hit return to start moves..." << std::endl;
    getchar();

    ProfileConfigTrap trap;

    trap.vel = 800000;
    trap.acc = 100000;
    trap.dec = 100000;
    trap.pos = 0;

    size_t i = 0;
    while (!done)
    {
        // if (i & 1)
        // {
        //     trap.pos = 10000;
        //     err_ = amp_[0].DoMove(trap);
        // }
        // else
        // {
        //     trap.pos = -10000;
        //     err_ = amp_[0].DoMove(trap);
        // }

        trap.pos = pos_d_[0];
        err_ = amp_[0].DoMove(trap);
        showerr_(err_, "doing move");
        err_ = amp_[0].WaitMoveDone(30000);
        showerr_(err_, "waiting on move");

        if (i == UINT_MAX)
            i = 0;
        else
            i++;
    }

    err_ = amp_[0].QuickStop();
    showerr_(err_, "Quick stop");
}

void CopleyDriver::publish_motor_state()
{
    motor_state_update_();
    msg_.name.resize(AXIS_COUNT);
    msg_.position.resize(AXIS_COUNT);
    msg_.velocity.resize(AXIS_COUNT);
    msg_.effort.resize(AXIS_COUNT);

    for (size_t i = 0; i < AXIS_COUNT; i++)
    {
        msg_.name[i] = joint_names_[i];
        msg_.position[i] = pos_[i];
        msg_.velocity[i] = vel_[i];
        msg_.effort[i] = tau_[i];
    }

    publisher_->publish(msg_);
}

void CopleyDriver::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (int i = 0; i < AXIS_COUNT; i++)
    {
        pos_d_[i] = int(msg->position[i] * counts_per_rad_[i]);
        vel_d_[i] = int(msg->velocity[i] * counts_sec_per_rad_[i]);
        tau_d_[i] = int(msg->effort[i] * units_per_nm[i]);
    }
    std::cout << "Received new message." << std::endl;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_callback_handler);

    rclcpp::InitOptions option = rclcpp::InitOptions();
    option.shutdown_on_signal = false;
    rclcpp::init(argc, argv, option);
    auto copley_driver = std::make_shared<CopleyDriver>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(copley_driver);
    sleep(1);
    executor.spin();

    std::cout << "Driver Stopped." << std::endl;

    return 0;
}