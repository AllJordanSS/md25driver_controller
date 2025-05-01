//! @note ROS
#include <rclcpp/rclcpp.hpp>

//! @note ROS standard msg
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <std_msgs/msg/int8.h> 

//! @note ROS lysa interfaces
#include "md25_controller/msg/md25_encoders.hpp"

//! @note PID class
#include "motor_controller/PID_control.hpp"

class TractionMotorControl : public rclcpp::Node
{
private:

    //! @note ROS
    rclcpp::Time time;

    //! @note ROS publishers
    rclcpp::Publisher<sensor_msgs::msg::ChannelFloat32>::SharedPtr md25_cmd_pub;

    //! @note ROS subscribers
    rclcpp::Subscription<md25_controller::msg::Md25Encoders>::SharedPtr encoders_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    //! @note Msgs
    sensor_msgs::msg::ChannelFloat32 voltageCommand;

    //! @note Topic names 
    std::string encoders_topic, cmd_vel_topic, md25_cmd_topic;

    //! @note Lysa parameters
    float rpm_to_ms_const, wheel_radius, wheels_separation;
    
    //! @note Rate
    int rate;
    float Ts;
    
    //! @note Velocity parameters
    float reference_linear_vel, reference_angular_vel, current_left_wheel_vel, 
          current_right_wheel_vel, reference_left_wheel_vel, reference_right_wheel_vel,
          left_wheel_signal_volts, right_wheel_signal_volts;

    //! @note PID parameters
    float kp_left, ki_left, kd_left, lBounds_left, hBounds_left, windupGuard_left;
    float kp_right, ki_right, kd_right, lBounds_right, hBounds_right, windupGuard_right;

    //! @note PID control
    PIDcontrol left_PIDcontroller, right_PIDcontroller; 

    //! @note Flags
    bool encoder_has_null, control_has_null;

    //! @note ROS timer
    rclcpp::TimerBase::SharedPtr timer;
    
public:
    //! @note Class constructor
    TractionMotorControl();

    //! @note Function to load parameters from .yaml
    void loadParameters();

    //! @note Convert body velocity to wheels velocity
    float lysa_to_wheels_left(float linear, float angular);
    float lysa_to_wheels_right(float linear, float angular);

    //! @note Initiate control
    void initPIDs();

    //! @note Update control
    void update();

    //! @note ROS callbacks
    void controlCallback(const geometry_msgs::msg::Twist::SharedPtr control);
    void encodersCallback(const md25_controller::msg::Md25Encoders::SharedPtr encoder);

};//End of class Md25BaseController