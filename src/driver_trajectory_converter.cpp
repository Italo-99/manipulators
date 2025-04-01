// Import libraries
#include "manipulators/DriverTrajectoryConverter.h"

// Constructor
DriverTrajectoryConverter::DriverTrajectoryConverter(std::string node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options), joint_map_initialized_(false), cmd_map_initialized_(false), mean_(0.0)
{
    declareParameters();

    joints_names_group_ = get_parameter("joints_names_group").as_string_array();
    velocity_topic_ = get_parameter("velocity_topic").as_string();
    kp_ = get_parameter("kp").as_double();
    min_motor_speed_ = get_parameter("min_motor_speed").as_double();
    spinner_rate_ = get_parameter("spinner_rate").as_int();

    // Load joint names group (from launch file)
    if (joints_names_group_.size() == 0)
    {
        RCLCPP_ERROR(get_logger(), "joint_names_group parameter must be provided. Shutting down...");
        rclcpp::shutdown();
    }

    // Setup the joint name to index map
    for (size_t i = 0; i < joints_names_group_.size(); ++i)
    {
        joint_name_to_index_[joints_names_group_[i]] = i;
        std::cout << "Found joint ready for the driver: " << joints_names_group_[i] << std::endl;
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

    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(velocity_topic_, 1);

    // Initialize Eigen matrices to zero
    joints_values_.setZero();
    dq_cmd_.setZero();
    qd_cmd_.setZero();
    real_vel_.setZero();
    vel_msg_.data.resize(6);

    rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        std::bind(&DriverTrajectoryConverter::shutdown_handler, this) // Register shutdown handler
    );
}

void DriverTrajectoryConverter::declareParameters(){
    this->declare_parameter("joints_names_group", std::vector<std::string>());
    this->declare_parameter("velocity_topic", "/ur_rtde/controllers/joint_velocity_controller/command");
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("min_motor_speed", 0.001);
    this->declare_parameter("spinner_rate", 500);
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
    if (counter_group >= joints_names_group_.size())
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
        if (it != joint_name_to_index_.end())
        {
            size_t index = it->second;
            if (cmd_state->position.size() > index) {qd_cmd_[index] = cmd_state->position[i];}
            if (cmd_state->velocity.size() > index) {dq_cmd_[index] = cmd_state->velocity[i];}
            counter_group++;
        }
    }

    // If all joints have been updated, mark as initialized
    if (counter_group >= joints_names_group_.size())
    {
        cmd_map_initialized_ = true;
    }
}

// Compute the velocity command using the proportional control
void DriverTrajectoryConverter::computeVel()
{
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
    // Create a single-threaded executor
    executor.add_node(shared_from_this());

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    //This will create a timer which will wait for the driver to be ready, then it will start the main loop and cancel itself
    while(rclcpp::ok()){
        if (isReady())
        {
            // Start the main loop
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / spinner_rate_),  // Period based on spinner rate
                [this, &steady_clock]()
                {
                    // Start time for mean computation
                    auto start_time = steady_clock.now();
        
                    // Compute velocity after every callback cycle
                    computeVel();
        
                    // Update mean computation
                    double elapsed_time = (steady_clock.now() - start_time).seconds();
                    mean_ = (mean_ * static_cast<double>(k) + elapsed_time) / static_cast<double>(k + 1);
                    k++;
                }
            );

            RCLCPP_INFO(get_logger(), "Driver is ready.");
            break;
        }

        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Run executor
    executor.spin();

    rclcpp::shutdown();
}
