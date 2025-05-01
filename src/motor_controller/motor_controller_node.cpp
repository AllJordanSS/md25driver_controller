#include <motor_controller/motor_controller.hpp>

int main(int argc, char **argv)
{
    //Initiate ROS
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TractionMotorControl>());
    rclcpp::shutdown();
    return 0;
}
