//! @note ROS
#include <rclcpp/rclcpp.hpp>

//! @note ROS standard msg
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>

//! @note ROS lysa interfaces
#include "md25_controller/msg/md25_data.hpp"
#include "md25_controller/msg/md25_encoders.hpp"

// @note MD5 driver
#include "MD25Driver/md25_driver.hpp"

class Md25BaseController : public rclcpp::Node
{
private:

    //! @note ROS publishers
    rclcpp::Publisher<md25_controller::msg::Md25Encoders>::SharedPtr encoders_pub;
    rclcpp::Publisher<md25_controller::msg::Md25Data>::SharedPtr data_pub;

    //! @note ROS subscribers
    rclcpp::Subscription<sensor_msgs::msg::ChannelFloat32>::SharedPtr commands_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_encoders_sub;

    //! @note MD25Driver
    MD25Driver *md25_driver;
    float leftVoltageCommand, rightVoltageCommand;

    //! @note MD25 messages
    md25_controller::msg::Md25Data md25_data_msg;
    md25_controller::msg::Md25Encoders md_25_encoders_msg;

    //! @note ROS timer
    rclcpp::TimerBase::SharedPtr timer;

    //! @note Driver parameters
    int address, rate;
    float cpr, motorRatio;
    bool regulator, timeout;
    std::string i2c_port;
    std::string encoders_topic_name,
                data_topic_name,
                commands_topic_name,
                reset_encoders_topic_name;
    

public:
    //! @note Class constructor
    Md25BaseController();

    //! @note Main loop
    void update();

    //! @note Function to load parameters from yaml file
    void loadParameters();

    //! @note ROS callbacks
    void commandsCallback(const sensor_msgs::msg::ChannelFloat32::SharedPtr cmd);
    void resetCallback(const std_msgs::msg::Bool::SharedPtr msg);

};//End of class Md25BaseController

