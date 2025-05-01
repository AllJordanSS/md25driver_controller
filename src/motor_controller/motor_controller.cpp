#include "motor_controller/motor_controller.hpp"

TractionMotorControl::TractionMotorControl(): 
    Node("motor_controller", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{   
    this->loadParameters();

    //! @note Publishers
    md25_cmd_pub = create_publisher<sensor_msgs::msg::ChannelFloat32>("/md25_voltage_commands", 1);

    //! @note Subscribers
    encoders_sub = create_subscription<md25_controller::msg::Md25Encoders>(encoders_topic, 1, 
        std::bind(&TractionMotorControl::encodersCallback, this, std::placeholders::_1));

    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic, 1, 
        std::bind(&TractionMotorControl::controlCallback, this, std::placeholders::_1));
    
    //! @note Configure PIDs controllers
    this->initPIDs();

    //! @note Get initial time
    time = this->now(); 

    // //! @note ROS timer
    timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&TractionMotorControl::update, this));
}

void TractionMotorControl::loadParameters()
{   
    if(!this->get_parameter("traction_control.control_rate", rate))
        rate = 100;

    //! @note Left
    if(!this->get_parameter("traction_control.pid_params_left.P", kp_left))
        kp_left = 15;
    if(!this->get_parameter("traction_control.pid_params_left.I", ki_left))
        ki_left = 30;
    if(!this->get_parameter("traction_control.pid_params_left.D", kd_left))
        kd_left = 0;
    if(!this->get_parameter("traction_control.pid_params_left.lBounds", lBounds_left))
        lBounds_left = -12;
    if(!this->get_parameter("traction_control.pid_params_left.hBounds", hBounds_left))
        hBounds_left = 12;
    if(!this->get_parameter("traction_control.pid_params_left.windupGuard", windupGuard_left))
        windupGuard_left = 0.15;

    //! @note Right
    if(!this->get_parameter("traction_control.pid_params_right.P", kp_right))
        kp_right = 15;
    if(!this->get_parameter("traction_control.pid_params_right.I", ki_right))
        ki_right = 30;
    if(!this->get_parameter("traction_control.pid_params_right.D", kd_right))
        kd_right = 0;
    if(!this->get_parameter("traction_control.pid_params_right.lBounds", lBounds_right))
        lBounds_right = -12;
    if(!this->get_parameter("traction_control.pid_params_right.hBounds", hBounds_right))
        hBounds_right = 12;
    if(!this->get_parameter("traction_control.pid_params_right.windupGuard", windupGuard_right))
        windupGuard_right = 0.15;

    //! @note Topic names
    if(!this->get_parameter("traction_general.cmd_vel_topic", cmd_vel_topic))
        cmd_vel_topic = "/cmd_vel";
    if(!this->get_parameter("traction_general.encoders_topic", encoders_topic))
        encoders_topic = "/md25_encoders";
    if(!this->get_parameter("traction_general.md25_cmd_topic", md25_cmd_topic))
        md25_cmd_topic = "/md25_voltage_commands";

    //! @note Wheels parameters
    if(!this->get_parameter("traction_general.wheel_radius", wheel_radius))
        wheel_radius = 0.0874;
    if(!this->get_parameter("traction_general.wheels_separation", wheels_separation))
        wheels_separation = 0.26;

    reference_linear_vel = 0.0;
    reference_angular_vel = 0.0;
    reference_left_wheel_vel = 0; 
    reference_right_wheel_vel = 0;
    current_left_wheel_vel = 0.0;
    current_right_wheel_vel = 0.0;
    left_wheel_signal_volts = 0;
    right_wheel_signal_volts = 0;

    Ts = 0;

    rpm_to_ms_const = wheel_radius * 2 * 3.1415 / 60; 

    encoder_has_null = true;
    control_has_null = true;
}

void TractionMotorControl::initPIDs()
{
    //! @note Left
    left_PIDcontroller = PIDcontrol();
    left_PIDcontroller.setGains(kp_left, ki_left, kd_left);
    left_PIDcontroller.setBounds(lBounds_left, hBounds_left);
    left_PIDcontroller.setWindupGuard(windupGuard_left);

    //! @note Right
    right_PIDcontroller = PIDcontrol();
    right_PIDcontroller.setGains(kp_right, ki_right, kd_right);
    right_PIDcontroller.setBounds(lBounds_right, hBounds_right);
    right_PIDcontroller.setWindupGuard(windupGuard_right);
}

void TractionMotorControl::update()
{
    if(!encoder_has_null and !control_has_null)
    {
        reference_left_wheel_vel =  lysa_to_wheels_left(reference_linear_vel, reference_angular_vel);
        reference_right_wheel_vel =  lysa_to_wheels_right(reference_linear_vel, reference_angular_vel);

        Ts = this->now().seconds() - time.seconds();
        time = this->now();

        left_wheel_signal_volts = left_PIDcontroller.updatePID(reference_left_wheel_vel, current_left_wheel_vel, Ts);
        right_wheel_signal_volts = right_PIDcontroller.updatePID(reference_right_wheel_vel, current_right_wheel_vel, Ts);

        voltageCommand.values.clear();
        voltageCommand.values.push_back(left_wheel_signal_volts);
        voltageCommand.values.push_back(right_wheel_signal_volts);
    }
    else
    {
        voltageCommand.values.clear();
        voltageCommand.values.push_back(0.0);
        voltageCommand.values.push_back(0.0);  
    }

    md25_cmd_pub->publish(voltageCommand);
}

float TractionMotorControl::lysa_to_wheels_left(float linear, float angular)
{
    return linear - (angular*wheels_separation/2);
}

float TractionMotorControl::lysa_to_wheels_right(float linear, float angular)
{
    return linear + (angular * wheels_separation/2);
}

void TractionMotorControl::controlCallback(const geometry_msgs::msg::Twist::SharedPtr control)
{   
    if(control)
    {
        reference_linear_vel = control->linear.x;
        reference_angular_vel = control->angular.z;
        control_has_null = false;
    }
    else
    {
        control_has_null = true;
    }
}

void TractionMotorControl::encodersCallback(const md25_controller::msg::Md25Encoders::SharedPtr encoder)
{
    if(encoder)
    {
        current_left_wheel_vel = rpm_to_ms_const * encoder->left_vel_rpm;
        current_right_wheel_vel = rpm_to_ms_const * encoder->right_vel_rpm;
        encoder_has_null = false;           
    }
    else
    {
        encoder_has_null = true;
    }
}