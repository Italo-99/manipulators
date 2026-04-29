// Import libraries
#include "manipulators/DriverTrajectoryConverter.h"

// Constructor
DriverTrajectoryConverter::DriverTrajectoryConverter(std::string node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options),
      joint_map_initialized_(false),
      cmd_map_initialized_(false),
      timeout_active_(false),
      real_joint_state_received_(false),
      mean_(0.0)
{
    declareParameters();

    joints_names_group_ = get_parameter("joints_names_group").as_string_array();
    velocity_topic_ = get_parameter("velocity_topic").as_string();
    real_joint_state_topic_ = get_parameter("real_joint_state_topic").as_string();
    kp_ = get_parameter("kp").as_double();
    min_motor_speed_ = get_parameter("min_motor_speed").as_double();
    joint_state_timeout_s_ = get_parameter("joint_state_timeout").as_double();
    spinner_rate_ = get_parameter("spinner_rate").as_int();

    // Validate runtime parameters
    if (joints_names_group_.empty())
    {
        RCLCPP_ERROR(get_logger(), "joint_names_group parameter must be provided. Shutting down...");
        rclcpp::shutdown();
        return;
    }
    if (spinner_rate_ <= 0)
    {
        RCLCPP_ERROR(get_logger(), "spinner_rate must be > 0. Shutting down...");
        rclcpp::shutdown();
        return;
    }
    if (joint_state_timeout_s_ <= 0.0)
    {
        RCLCPP_ERROR(get_logger(), "joint_state_timeout must be > 0. Shutting down...");
        rclcpp::shutdown();
        return;
    }

    // Setup the joint name to index map
    for (size_t i = 0; i < joints_names_group_.size(); ++i)
    {
        joint_name_to_index_[joints_names_group_[i]] = i;
        std::cout << "Found joint ready for the driver: " << joints_names_group_[i] << std::endl;
    }

    subscribers_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = subscribers_callback_group_;

    // Setup subscribers and publishers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        real_joint_state_topic_,
        1,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            jointStateCallback(msg);
        },
        sub_options);

    joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/move_group/fake_controller_joint_states",
        1,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            jointCmdCallback(msg);
        },
        sub_options);

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(velocity_topic_, 1);

    // Initialize Eigen matrices to zero
    joints_values_.setZero();
    dq_cmd_.setZero();
    qd_cmd_.setZero();
    real_vel_.setZero();
    vel_msg_.data.resize(6, 0.0);
    zero_vel_msg_.data.resize(6, 0.0);

    rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        std::bind(&DriverTrajectoryConverter::shutdown_handler, this) // Register shutdown handler
    );
}

void DriverTrajectoryConverter::declareParameters(){
    this->declare_parameter("joints_names_group", std::vector<std::string>());
    this->declare_parameter("velocity_topic", "/ur_rtde/controllers/joint_velocity_controller/command");
    this->declare_parameter("real_joint_state_topic", "/fake/joint_states");
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("min_motor_speed", 0.001);
    this->declare_parameter("joint_state_timeout", 2.0);
    this->declare_parameter("spinner_rate", 500);
}

// Shutdown handler
void DriverTrajectoryConverter::shutdown_handler()
{
    // Show the result of the jacobian control mean duration
    RCLCPP_INFO(get_logger(), "Mean duration of real driver control computations: %f seconds", mean_);

    // Publish zero velocities when shutting down
    velocity_publisher_->publish(zero_vel_msg_);
}

// Check if real joint state map is initialized
bool DriverTrajectoryConverter::isReady()
{
    return joint_map_initialized_;
}

bool DriverTrajectoryConverter::isJointStateTimeout() const
{
    if (!real_joint_state_received_)
    {
        return true;
    }

    return (this->now() - last_joint_state_rx_time_).seconds() > joint_state_timeout_s_;
}

// Callback to receive actual joint states
void DriverTrajectoryConverter::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joints_state)
{
    uint counter_group = 0;

    // Loop through the received joint states
    for (uint i = 0; i < joints_state->name.size(); i++)
    {
        const std::string& joint_name = joints_state->name[i];
        auto it = joint_name_to_index_.find(joint_name);
        if (it != joint_name_to_index_.end())
        {
            size_t index = it->second;
            if (i < joints_state->position.size())
            {
                joints_values_[index] = joints_state->position[i];
                counter_group++;
            }
        }
    }

    // If all joints have been updated, mark as initialized
    if (counter_group >= joints_names_group_.size())
    {
        joint_map_initialized_ = true;
    }

    last_real_joint_state_ = *joints_state;
    last_joint_state_rx_time_ = this->now();
    real_joint_state_received_ = true;
}

// Callback to receive the fake controller joint states (commands)
void DriverTrajectoryConverter::jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr cmd_state)
{
    uint counter_group = 0;

    // Loop through the received joint commands
    for (uint i = 0; i < cmd_state->name.size(); i++)
    {
        const std::string& joint_name = cmd_state->name[i];
        auto it = joint_name_to_index_.find(joint_name);
        if (it != joint_name_to_index_.end())
        {
            size_t index = it->second;
            if (i < cmd_state->position.size()) {qd_cmd_[index] = cmd_state->position[i];}
            if (i < cmd_state->velocity.size()) {dq_cmd_[index] = cmd_state->velocity[i];}
            counter_group++;
        }
    }

    // If all joints have been updated, mark as initialized
    if (counter_group >= joints_names_group_.size())
    {
        cmd_map_initialized_ = true;
    }
}

void DriverTrajectoryConverter::pubZOH_JointState()
{
    if (!real_joint_state_received_)
    {
        return;
    }

    auto zoh_msg = last_real_joint_state_;
    zoh_msg.header.stamp = this->now();
    joint_state_publisher_->publish(zoh_msg);
}

// Compute the velocity command using the proportional control
void DriverTrajectoryConverter::computeVel()
{
    if (isJointStateTimeout())
    {
        if (!timeout_active_)
        {
            timeout_active_ = true;
            RCLCPP_WARN(
                get_logger(),
                "Joint state timeout active: publishing zero velocity until fresh data is received.");
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *this->get_clock(),
                5000,
                "Joint state timeout still active: publishing zero velocity.");
        }

        velocity_publisher_->publish(zero_vel_msg_);
        return;
    }

    if (timeout_active_)
    {
        timeout_active_ = false;
        RCLCPP_INFO(get_logger(), "Joint state timeout cleared: resuming computed velocities.");
    }

    // Compute the velocity output: real_vel_ = dq_cmd_ + kp_ * (qd_cmd_ - joints_values_)
    real_vel_ = dq_cmd_ + kp_ * (qd_cmd_ - joints_values_);

    // Apply minimum velocity threshold and prepare velocity message
    for (size_t i = 0; i < 6; ++i)
    {
        // Apply minimum velocity threshold
        if (std::abs(real_vel_[i]) < min_motor_speed_)
        {
            real_vel_[i] = 0.0;
        }

        // Set the velocity message
        vel_msg_.data[i] = real_vel_[i];
    }

    // Publish velocity command to the robot
    velocity_publisher_->publish(vel_msg_);
}

// Spinner to continuously call callbacks and compute velocity
void DriverTrajectoryConverter::spinner()
{
    executor_.add_node(shared_from_this());

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    // This will create a timer which waits for the driver to be ready, then starts the main loop and cancels itself
    while (rclcpp::ok())
    {
        if (isReady())
        {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / spinner_rate_),
                [this, &steady_clock]()
                {
                    auto start_time = steady_clock.now();

                    pubZOH_JointState();
                    if (cmd_map_initialized_)
                    {
                        computeVel();
                    }
                    else
                    {
                        velocity_publisher_->publish(zero_vel_msg_);
                    }

                    double elapsed_time = (steady_clock.now() - start_time).seconds();
                    mean_ = (mean_ * static_cast<double>(k) + elapsed_time) / static_cast<double>(k + 1);
                    k++;
                },
                subscribers_callback_group_);

            RCLCPP_INFO(get_logger(), "Driver is ready: streaming joint states.");
            break;
        }

        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Run executor
    executor_.spin();

    rclcpp::shutdown();
}
