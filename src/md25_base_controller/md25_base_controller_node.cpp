#include <md25_base_controller/md25_base_controller.hpp>

int main(int argc, char **argv)
{
    //Initiate ROS
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Md25BaseController>());
    rclcpp::shutdown();
    return 0;
}
