#include "odometry/odometry.hpp"

Odometry::Odometry() : Node("wheels_odometry", rclcpp::NodeOptions()
                                                   .allow_undeclared_parameters(true)
                                                   .automatically_declare_parameters_from_overrides(true))
{
    this->initParameters();

    //! @note Publishers
    odom_pub = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);

    //! @note Subscribers
    encoders_sub = create_subscription<md25_controller::msg::Md25Encoders>(encoders_topic, 1,
                                                                           std::bind(&Odometry::encodersCallback, this, std::placeholders::_1));

    camera_odom_sub = create_subscription<nav_msgs::msg::Odometry>(odom_camera_topic, 1,
                                                                   std::bind(&Odometry::odomCallback, this, std::placeholders::_1));

    this->initCovarianceMatrix();

    //! @note ROS timer
    timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&Odometry::update, this));
}

void Odometry::initParameters()
{
    if (!this->get_parameter("traction_general.rate", rate))
        rate = 10;

    if (!this->get_parameter("traction_general.encoders_topic", encoders_topic))
        encoders_topic = "/md25_encoders";
    if (!this->get_parameter("traction_general.odom_camera_topic", odom_camera_topic))
        odom_camera_topic = "/camera/odom/sample";
    if (!this->get_parameter("traction_general.odom_topic", odom_topic))
        odom_topic = "/wheels_odom";

    if (!this->get_parameter("traction_general.wheel_radius", wheel_radius))
        wheel_radius = 0.0874;
    if (!this->get_parameter("traction_general.wheels_separation", wheels_separation))
        wheels_separation = 0.26;

    if (!this->get_parameter("traction_general.linearGain", kLinear))
        kLinear = 10.0;
    if (!this->get_parameter("traction_general.angularGain", kAngular))
        kAngular = 40.0;

    dt = 0;
    lysa_linear_vel = 0; lysa_angular_vel = 0;
    delta_x = 0; delta_y = 0; delta_th = 0;
    pos_x = 0; pos_y = 0; pos_theta = 0;
    errorL = 0; errorA = 0;

    rpm_to_ms_const = wheel_radius * 2 * 3.1415 / 60;

    encoder_has_null = true;
    odom_camera_has_null = true;

    //! @note Get initial time
    time = this->now();
}

void Odometry::initCovarianceMatrix()
{
    for (int i = 0; i < 36; i++)
    {
        odom_msg.pose.covariance.at(i) = 0;
        odom_msg.twist.covariance.at(i) = 0;
    }
}

void Odometry::update()
{

    // if(!encoder_has_null and !odom_camera_has_null)
    if (!encoder_has_null)
    {
        current = this->now();
        dt = current.seconds() - time.seconds();

        delta_x = lysa_linear_vel * cos(pos_theta) * dt;
        delta_y = lysa_linear_vel * sin(pos_theta) * dt;
        delta_th = lysa_angular_vel * dt;

        pos_x += delta_x;
        pos_y += delta_y;
        pos_theta += delta_th;

        time = current;

        // odom_quat = tf::createQuaternionMsgFromYaw(pos_theta);
        quat.setRPY(0, 0, pos_theta);
        tf2::convert(quat, odom_quat);
        // odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pos_theta);

        // Publish over topic
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = pos_x;
        odom_msg.pose.pose.position.y = pos_y;
        odom_msg.pose.pose.position.z = 0;
        odom_msg.pose.pose.orientation = odom_quat;
        odom_msg.pose.covariance.at(0) = 10000;
        odom_msg.pose.covariance.at(7) = 10000;
        odom_msg.pose.covariance.at(14) = 10000;
        odom_msg.pose.covariance.at(21) = 10000;
        odom_msg.pose.covariance.at(28) = 10000;
        odom_msg.pose.covariance.at(35) = 10000;

        odom_msg.twist.twist.linear.x = lysa_linear_vel;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;
        odom_msg.twist.twist.angular.x = 0;
        odom_msg.twist.twist.angular.y = 0;
        odom_msg.twist.twist.angular.z = lysa_angular_vel;

        // TODO verificação
        if (odom_msg.twist.twist.linear.x == 0 and odom_msg.twist.twist.angular.z == 0 and
            abs(odom_camera_msg.twist.twist.linear.x) < 0.1 and abs(odom_camera_msg.twist.twist.angular.z) < 0.1)
        {
            odom_msg.twist.covariance.at(0) = 0.0;
            odom_msg.twist.covariance.at(7) = 0.0;
            odom_msg.twist.covariance.at(14) = 0.0;
            odom_msg.twist.covariance.at(21) = 0.0;
            odom_msg.twist.covariance.at(28) = 0.0;
            odom_msg.twist.covariance.at(35) = 0.0;
        }
        else
        {
            errorL = abs(odom_msg.twist.twist.linear.x - odom_camera_msg.twist.twist.linear.x);
            errorA = abs(odom_msg.twist.twist.angular.z - odom_camera_msg.twist.twist.angular.z);

            odom_msg.twist.covariance.at(0) = 0.1 + kLinear * errorL + kAngular * errorA;
            odom_msg.twist.covariance.at(7) = 0.1 + kLinear * errorL + kAngular * errorA;
            odom_msg.twist.covariance.at(14) = 0.1 + kLinear * errorL + kAngular * errorA;
            odom_msg.twist.covariance.at(21) = 0.1 + kLinear * errorL + kAngular * errorA;
            odom_msg.twist.covariance.at(28) = 0.1 + kLinear * errorL + kAngular * errorA;
            odom_msg.twist.covariance.at(35) = 0.1 + kLinear * errorL + kAngular * errorA;
        }

        odom_pub->publish(odom_msg);
    }
}

float Odometry::get_lysa_linear_vel(float left_wheel_vel, float right_wheel_vel)
{
    return ((rpm_to_ms_const * left_wheel_vel) + (rpm_to_ms_const * right_wheel_vel)) / 2;
}

float Odometry::get_lysa_angular_vel(float left_wheel_vel, float right_wheel_vel)
{
    return ((rpm_to_ms_const * right_wheel_vel) - (rpm_to_ms_const * left_wheel_vel)) / wheels_separation;
}

void Odometry::encodersCallback(const md25_controller::msg::Md25Encoders::SharedPtr encoder)
{
    if (encoder)
    {
        lysa_linear_vel = get_lysa_linear_vel(encoder->left_vel_rpm, encoder->right_vel_rpm);
        lysa_angular_vel = get_lysa_angular_vel(encoder->left_vel_rpm, encoder->right_vel_rpm);
        encoder_has_null = false;
    }
    else
    {
        encoder_has_null = true;
    }
}

void Odometry::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    if (odom)
    {
        odom_camera_msg.twist.twist.linear.x = odom->twist.twist.linear.x;
        odom_camera_msg.twist.twist.angular.z = odom->twist.twist.angular.z;
        odom_camera_has_null = false;
    }
    else
    {
        odom_camera_has_null = true;
    }
}