#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"
#include "cmath"
#include <chrono>
#include <memory>
#include <functional>
#include <opencv2/opencv.hpp>
#include <iostream>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
using namespace cv;

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
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/red_line",10);

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        //timer_ = this->create_wall_timer(100ms, std::bind(&CarNode::time_cycle, this));
    }

private:

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if(cv_ptr){
            Mat image = cv_ptr->image;

            Mat hsv;

            cvtColor(image, hsv, COLOR_BGR2HSV);

            Scalar low_red1(0,120,70);
            Scalar upper_red1(10,255,255);

            Scalar low_red2(170,120,70);
            Scalar upper_red2(179,255,255);

            Mat mask1, mask2;
            inRange(hsv, low_red1, upper_red1, mask1 );
            inRange(hsv, low_red2, upper_red2, mask2);

            Mat mask;
            bitwise_or(mask1,mask2,mask);
            
            cv_bridge::CvImage img_bridge;
            const sensor_msgs::msg::Image::SharedPtr img_msg;

            std_msgs::Header header; 
            header.seq = counter; 
            header.stamp = ros::Time::now(); 
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mask);
            img_bridge.toImageMsg(img_msg);
            publisher_->publish(img_msg);

        }

    }


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Image::SharedPtr last_msg_;
};



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