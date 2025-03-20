// Import libraries
#include "manipulators/DriverTrajectoryConverter.h"

DriverTrajectoryConverter* DriverTrajectoryConverter::instance__ = nullptr;

// Constructor
DriverTrajectoryConverter::DriverTrajectoryConverter(std::string node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options), joint_map_initialized_(false), cmd_map_initialized_(false), mean_(0.0)
{
    declareParameters();

    // Load joint names group (from launch file)
    std::vector<std::string> joints_names_group = this->get_parameter("joints_names_group").as_string_array();
    if (joints_names_group.size() == 0)
    {
        RCLCPP_ERROR(get_logger(), "joint_names_group parameter must be provided. Shutting down...");
        rclcpp::shutdown();
    }

    // Setup the joint name to index map
    for (size_t i = 0; i < joints_names_group.size(); ++i)
    {
        joint_name_to_index_[joints_names_group[i]] = i;
        std::cout << "Found joint ready for the driver: " << joints_names_group[i] << std::endl;
    }

    // Setup subscriber and publisher
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            jointStateCallback(msg);
        }
    );
    joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/move_group/fake_controller_joint_states", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            jointCmdCallback(msg);
        }
    );

    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(get_parameter("velocity_topic").as_string(), 1);

    // Initialize Eigen matrices to zero
    joints_values_.setZero();
    dq_cmd_.setZero();
    qd_cmd_.setZero();
    real_vel_.setZero();
    vel_msg_.data.resize(6);
}

DriverTrajectoryConverter::~DriverTrajectoryConverter()
{
    instance__ = nullptr;
}

void DriverTrajectoryConverter::declareParameters(){
    this->declare_parameter("joints_names_group", std::vector<std::string>());
    this->declare_parameter("velocity_topic", "/ur_rtde/controllers/joint_velocity_controller/command");
    this->declare_parameter("kp", 4.0);
    this->declare_parameter("min_motor_speed", 0.001);
    this->declare_parameter("spinner_rate", 500);
}

void DriverTrajectoryConverter::static_shutdown_handler(int sig)
{
    //Very unelegant way to call a non-static shutdown handler before the context is destroyed, but it works
    sig++; //Suppress unused var warning
    instance__->shutdown_handler();
}

// Shutdown handler
void DriverTrajectoryConverter::shutdown_handler()
{
    // Show the result of the jacobian control mean duration
    RCLCPP_INFO(get_logger(), "Mean duration of real driver control computations: %f seconds", mean_);
    // Publish zero velocities when shutting down
    std_msgs::msg::Float64MultiArray zero_vel;
    zero_vel.data.resize(6, 0.0);
    
    velocity_publisher_->publish(zero_vel);

    for(size_t i = 0; i < 2; ++i)
    {
        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Shutdown ROS
    rclcpp::shutdown();
}

// Check if both joint state and command maps are initialized
bool DriverTrajectoryConverter::isReady()
{
    return joint_map_initialized_ && cmd_map_initialized_;
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
        // std::cout << "Iterator 1: " << it->first << std::endl;  // Joint name
        // std::cout << "Iterator 2: " << it->second << std::endl; // Joint position in the map
        if (it != joint_name_to_index_.end())
        {
            size_t index = it->second;
            joints_values_[index] = joints_state->position[i];
            counter_group++;
        }
    }

    // If all joints have been updated, mark as initialized
    if (counter_group >= get_parameter("joints_names_group").as_string_array().size())
    {
        joint_map_initialized_ = true;
    }
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
        // std::cout << "Iterator 1: " << it->first << std::endl;  // Joint name
        // std::cout << "Iterator 2: " << it->second << std::endl; // Joint position in the map
        if (it != joint_name_to_index_.end())
        {
            size_t index = it->second;
            if (cmd_state->position.size() > index) {qd_cmd_[index] = cmd_state->position[i];}
            if (cmd_state->velocity.size() > index) {dq_cmd_[index] = cmd_state->velocity[i];}
            counter_group++;
        }
    }

    // If all joints have been updated, mark as initialized
    if (counter_group >= get_parameter("joints_names_group").as_string_array().size())
    {
        cmd_map_initialized_ = true;
    }
}

// Compute the velocity command using the proportional control
void DriverTrajectoryConverter::computeVel()
{
    // Compute the velocity output: real_vel_ = dq_cmd_ + kp_ * (qd_cmd_ - joints_values_)
    real_vel_ = dq_cmd_ + get_parameter("kp").as_double() * (qd_cmd_ - joints_values_);

    // Apply minimum velocity threshold and prepare velocity message
    for (size_t i = 0; i < 6; ++i)
    {
        // Apply minimum velocity threshold
        if (std::abs(real_vel_[i]) < get_parameter("min_motor_speed").as_double())
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
    // Number of samples for mean computation
    unsigned long long int k = 0;
    instance__ = this;
    signal(SIGINT, DriverTrajectoryConverter::static_shutdown_handler);
    rclcpp::Rate rate(get_parameter("spinner_rate").as_int());

    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());                    // Process callbacks
        auto start_time = std::chrono::high_resolution_clock::now(); // Start time for mean computation
        computeVel();                       // Compute velocity after every callback cycle

        // Update mean computation
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
        
        // Calculate the mean time for each iteration of the spinner
        mean_ = (mean_ * static_cast<double>(k) + elapsed_time.count()) / static_cast<double>(k + 1);
        k++;

        // Sleep according to the defined spinner rate
        rate.sleep();
    }
}