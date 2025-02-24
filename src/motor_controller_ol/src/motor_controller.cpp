#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <libserial/SerialPort.h>
#include <string>

class MotorController : public rclcpp::Node
{
public:
    MotorController(): Node("motor_controller")
{
    this->declare_parameter("wheel_base", 0.12);
    this->declare_parameter("wheel_radius", 0.034);
    this->declare_parameter("usb_port", "/dev/ttyACM0");
    
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    port = this->get_parameter("usb_port").as_string();
    

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/bumperbot_controller/cmd_vel_unstamped", 5,
        std::bind(&MotorController::cmdVelCallback, this, std::placeholders::_1));

    try
    {
        serial_port_.Open(port);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    }
    catch (const LibSerial::OpenFailed &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port opening failed: %s", e.what());
    }
};

private:
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void sendToMicrocontroller(double right_wheel_speed, double left_wheel_speed);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscriber_;
    LibSerial::SerialPort serial_port_;

    double wheel_base_;
    double wheel_radius_;
    std::string port;
};

#endif // MOTOR_CONTROLLER_HPP



void MotorController::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    double linear_x = msg->twist.linear.x;
    double angular_z = msg->twist.angular.z;

    double right_wheel_speed = (linear_x + (angular_z * wheel_base_ / 2.0));
    double left_wheel_speed = (linear_x - (angular_z * wheel_base_ / 2.0));

    sendToMicrocontroller(right_wheel_speed, left_wheel_speed);
}

void MotorController::sendToMicrocontroller(double right_wheel_speed, double left_wheel_speed)
{
    if (!serial_port_.IsOpen())
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
        return;
    }

    char right_wheel_sign = (right_wheel_speed >= 0) ? 'p' : 'n';
    char left_wheel_sign = (left_wheel_speed >= 0) ? 'p' : 'n';

    std::string compensate_zeros_right = (std::abs(right_wheel_speed) < 10.0) ? "0" : "";
    std::string compensate_zeros_left = (std::abs(left_wheel_speed) < 10.0) ? "0" : "";

    std::ostringstream message_stream;
    message_stream << std::fixed << std::setprecision(2)
                   << "r" << right_wheel_sign << compensate_zeros_right << std::abs(right_wheel_speed)
                   << ",l" << left_wheel_sign << compensate_zeros_left << std::abs(left_wheel_speed) << ",";

    serial_port_.Write(message_stream.str());

    RCLCPP_INFO(this->get_logger(), "Sent to microcontroller: %s", message_stream.str().c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}
