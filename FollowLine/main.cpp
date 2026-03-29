#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"
#include "cmath"
#include <chrono>
#include <memory>
#include <functional>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
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
    :  Node("car_node"),
    k_p_(1.0),
    k_i_(0.0),
    k_d_(0.0),
    dt_(0.001),
    err_prev_(0.0),
    integral_(0.0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cam_f1_left/image_raw", 10,
            std::bind(&CarNode::topic_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/red_line",10);

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        //timer_ = this->create_wall_timer(100ms, std::bind(&CarNode::time_cycle, this));
    }
    
private:
    float k_p_;
    float k_i_;
    float k_d_;
    float dt_;
    float err_prev_;
    float integral_;

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
            inRange(hsv, low_red1, upper_red1, mask1);
            inRange(hsv, low_red2, upper_red2, mask2);

            Mat mask;
            bitwise_or(mask1,mask2,mask);

            //morphological opening -> Removes noise
            erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

            //morphological closing -> Fixes object shape
            dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

            Moments img_moments = moments(mask);

            double m00 = img_moments.m00;
            double m10 = img_moments.m10;
            double m01 = img_moments.m01;

            Mat mask_color;
            cvtColor(mask, mask_color, COLOR_GRAY2BGR);
            
            int w = msg->width;
            int ctr_imgx = w/2;

            if (m00 > 10000)
            {
                int ctr_x = m10/m00;
                int ctr_y = m01/m00;

                circle(mask_color, Point(ctr_x,ctr_y), 5, Scalar(0,0,255), -1);


                int err = ctr_imgx - ctr_x;
                integral_ += err * dt_;
                float derivative = (err - err_prev_) / dt_;
                
                auto message = geometry_msgs::msg::Twist();
                message.linear.x = 1.0;
                float out = (k_p_ * err) + (k_i_ * integral_) + (k_d_ * derivative);
                //message.angular.z = out;

                std::cout << "Kp: " << k_p_ 
                        << " Ki: " << k_i_ 
                        << " Kd: " << k_d_ 
                        << "out: " << out << std::endl;

                vel_publisher_->publish(message);

                err_prev_ = err;

            }

            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();

            sensor_msgs::msg::Image::SharedPtr out_msg =
                cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mask_color).toImageMsg();

            publisher_->publish(*out_msg);

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