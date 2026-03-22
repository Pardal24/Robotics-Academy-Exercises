#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"
#include "cmath"
#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

//INSTEAD OF GUI USE RVIZ2

class CarNode : public rclcpp::Node
{
public:
    CarNode()
    : Node("car_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cam_f1_left/image_raw", 10,
            std::bind(&CarNode::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/webgui_image", 10);

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        //timer_ = this->create_wall_timer(100ms, std::bind(&CarNode::time_cycle, this));
    }

private:

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        publisher_->publish(out);
    }
    // void time_cycle()
    // {
    //     if (last_msg_)
    //     {
    //         //RCLCPP_INFO(this->get_logger(), "Publishing image");
    //         publisher_->publish(*last_msg_);
    //     }
    //     //PID
    // }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Image::SharedPtr last_msg_;
};

// class VelPublisher : public rclcpp::Node
// {
// public: 
//     VelPublisher()
//     : Node("vel_publisher")
//     {
//         publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
//         timer_ = this->create_wall_timer(100ms, std::bind(&VelPublisher::control_cycle, this));
//     }

// private:
//     //Tirar isto do private
//     void control_cycle()
//     {
//         auto message = geometry_msgs::msg::Twist();
//         message.linear.x = 2.0;
//         message.angular.z=1.8;
//         RCLCPP_INFO(this->get_logger(), "Publishing:");
//         publisher_->publish(message);
//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
// };




void exercise() {
    //Frequency freq = Frequency();

    while (true)
    {
        //rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CarNode>());
        //rclcpp::shutdown();
        //freq.tick();


    }
}