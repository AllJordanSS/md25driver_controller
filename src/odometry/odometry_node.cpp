#include <odometry/odometry.hpp>

int main(int argc, char **argv)
{
    //Initiate ROS
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odometry>());
    rclcpp::shutdown();
    return 0;
}
