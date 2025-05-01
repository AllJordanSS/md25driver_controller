#include "md25_base_controller/md25_base_controller.hpp"

Md25BaseController::Md25BaseController(): 
    Node("md25_base_controller", rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{   
    this->loadParameters();

    //! @note Publishers
    encoders_pub = create_publisher<md25_controller::msg::Md25Encoders>(encoders_topic_name, 1);
    data_pub = create_publisher<md25_controller::msg::Md25Data>(data_topic_name, 1);

    //! @note Subscribers
    commands_sub = create_subscription<sensor_msgs::msg::ChannelFloat32>(commands_topic_name, 1, 
        std::bind(&Md25BaseController::commandsCallback, this, std::placeholders::_1));

    reset_encoders_sub = create_subscription<std_msgs::msg::Bool>(reset_encoders_topic_name, 1, 
        std::bind(&Md25BaseController::resetCallback, this, std::placeholders::_1));
    
    //! @note MD25Driver
    md25_driver = new MD25Driver(address, i2c_port.c_str());
    
    md25_driver->configureDevice(cpr, motorRatio, rate);

    if(timeout)
    {
        md25_driver->enableTimeout();
        std::cout << "Timeout is enabled" << std::endl;
    }

    if(!regulator)
    {
        md25_driver->disableRegulation();
        std::cout << "Regulation is disabled" << std::endl;
    }

    //! @note ROS timer
    timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&Md25BaseController::update, this));
}

void Md25BaseController::loadParameters()
{   
    if(!this->get_parameter("md25.i2c_bus", i2c_port))
        i2c_port = "/dev/i2c-1";

    if(!this->get_parameter("md25.i2c_address", address))
        address = 0x58;

    if(!this->get_parameter("md25.regulator", regulator))
        regulator = true;

    if(!this->get_parameter("md25.timeout", timeout))
        timeout = true;

    if(!this->get_parameter("md25.acquisition_rate", rate))
        rate = 100;

    if(!this->get_parameter("md25.publish_topics.encoders", encoders_topic_name))
        encoders_topic_name = "/md25_encoders";

    if(!this->get_parameter("md25.publish_topics.data", data_topic_name))
        data_topic_name = "/md25_data";

    if(!this->get_parameter("md25.subscribe_topics.commands", commands_topic_name))
        commands_topic_name = "/md25_voltage_commands"; 

    if(!this->get_parameter("md25.subscribe_topics.reset_encoders", reset_encoders_topic_name))
        reset_encoders_topic_name = "/md25/reset";

    if(!this->get_parameter("encoders.cpr", cpr))
        cpr = 48.0;

    if(!this->get_parameter("encoders.motorRatio", motorRatio))
        motorRatio = 46.85;

    leftVoltageCommand = 0; rightVoltageCommand = 0;
}

void Md25BaseController::update()
{   
    //! @note Set motor velocities
    md25_driver->setLeftMotorVelocity(leftVoltageCommand);
    md25_driver->setRightMotorVelocity(rightVoltageCommand);

    //! @note Publish data
    md25_data_msg.header.stamp = this->now();   
    md25_data_msg.volts = md25_driver->getBatteryVoltage();
    auto[current_l, current_r] = md25_driver->getCurrents();
    md25_data_msg.current_l = current_l;
    md25_data_msg.current_r = current_r;
    md25_data_msg.error = 0;       
    md25_data_msg.acceleration = md25_driver->readAcceleration();   
    md25_data_msg.mode = md25_driver->readMode(); 
    md25_data_msg.regulator = md25_driver->readRegulation();   
    md25_data_msg.timeout = md25_driver->readTimeout();  

    data_pub->publish(md25_data_msg);    

    //! @note Publish encoder 
    auto[leftEncoder, rightEncoder, leftVelRPM, rightVelRPM,
		leftDeltaCounts, rightDeltaCounts] = md25_driver->getVelocities();

    md_25_encoders_msg.header.stamp = this->now();   
    md_25_encoders_msg.encoder_l = leftEncoder;
    md_25_encoders_msg.encoder_r = rightEncoder;
    md_25_encoders_msg.left_vel_rpm = leftVelRPM;
    md_25_encoders_msg.right_vel_rpm = rightVelRPM;
    md_25_encoders_msg.left_delta_pulses = leftDeltaCounts;
    md_25_encoders_msg.right_delta_pulses = rightDeltaCounts;

    encoders_pub->publish(md_25_encoders_msg);
}

void Md25BaseController::commandsCallback(const sensor_msgs::msg::ChannelFloat32::SharedPtr cmd)
{
    if(cmd)
    {
        leftVoltageCommand = cmd->values.at(0);
        rightVoltageCommand = cmd->values.at(1);
    }
}

void Md25BaseController::resetCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg)
    {
        if(msg->data)
        {
            md25_driver->resetEncoders();
        }
    }
}


