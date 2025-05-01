#include <math.h>

//! @note ROS
#include <rclcpp/rclcpp.hpp>

//! @note ROS tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//! @note ROS standard msg
#include <nav_msgs/msg/odometry.hpp>

//! @note ROS lysa interfaces
#include "md25_controller/msg/md25_encoders.hpp"

class Odometry : public rclcpp::Node
{
private:

    //! @note ROS
    rclcpp::Time time;
    rclcpp::Time current;
    int rate;

    //! @note ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    //! @note ROS subscribers
    rclcpp::Subscription<md25_controller::msg::Md25Encoders>::SharedPtr encoders_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odom_sub;

    //! @note Msgs
    tf2::Quaternion quat;
    geometry_msgs::msg::Quaternion odom_quat;
    nav_msgs::msg::Odometry odom_msg;
    nav_msgs::msg::Odometry odom_camera_msg;

    //! @note Topic names 
    std::string encoders_topic, odom_camera_topic, odom_topic;

    //! @note Robot parameters
    float rpm_to_ms_const, wheel_radius, wheels_separation;
    float lysa_linear_vel, lysa_angular_vel;

    //! @note Odometry parameters
    float dt;
    float delta_x, delta_y, delta_th;
    float pos_x, pos_y, pos_theta;
    float errorL, errorA;
    float kLinear, kAngular;

    //! @note Flags
    bool encoder_has_null, odom_camera_has_null;

    //! @note ROS timer
    rclcpp::TimerBase::SharedPtr timer;
    
public:
    //! @note Class constructor
    Odometry();

    //! @note Function to initialize the parameters
    void initParameters();

    //! @note Function to create the covariance matrix
    void initCovarianceMatrix();

    //! @note Main loop
    void update();

    //! @note Function to get the linear velocity of the robot
    float get_lysa_linear_vel(float left_wheel_vel, float right_quatwheel_vel);

    //! @note Function to get the angular velocity of the robot
    float get_lysa_angular_vel(float left_wheel_vel, float right_wheel_vel);

    //! @note ROS callbacks
    void encodersCallback(const md25_controller::msg::Md25Encoders::SharedPtr encoder);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    
};//End of class Odometry