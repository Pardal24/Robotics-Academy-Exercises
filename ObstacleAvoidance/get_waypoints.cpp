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
#include <nav_msgs/msg/odometry.hpp>
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
    k_p_(0.5),
    k_i_(0.01),
    k_d_(0.001),
    dt_(0.001),
    err_prev_(0.0),
    integral_(0.0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cam_f1_left/image_raw", 10,
            std::bind(&CarNode::topic_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/red_line",10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&CarNode::odom_callback, this, std::placeholders::_1))

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        //timer_ = this->create_wall_timer(100ms, std::bind(&CarNode::time_cycle, this));
    }
    
private:
    //float trash;
    float k_p_;
    float k_i_;
    float k_d_;
    float dt_;
    float err_prev_;
    float integral_;
    float waypoints_ [];

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (finished_) return;

        float car_x = msg->pose.pose.position.x;
        float car_y = msg->pose.pose.position.y;

        float min_dist = 1.0;
        float lap_threshold = 1.0;

        if (!start_set_)
        {
            start_x_ = car_x;
            start_y_ = car_y;
            start_set_ = true;

            waypoints_.push_back({car_x, car_y});
            return;
        }

        float last_x = waypoints_.back().first;
        float last_y = waypoints_.back().second;

        float dx = car_x - last_x;
        float dy = car_y - last_y;

        float dist = sqrt(dx * dx + dy * dy);

        if (dist >= min_dist)
        {
            waypoints_.push_back({car_x, car_y});
        }

        float dx_start = car_x - start_x_;
        float dy_start = car_y - start_y_;
        float dist_to_start = sqrt(dx_start * dx_start + dy_start * dy_start);

        if (!lap_started_ && waypoints_.size() > 20)
        {
            lap_started_ = true;
        }

        if (lap_started_ && dist_to_start < lap_threshold)
        {
            finished_ = true;
            save_waypoints();
            RCLCPP_INFO(this->get_logger(), "Lap completed! Waypoints saved.");
        }
    }

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
                message.linear.x = 5.0;
                float out = (k_p_ * err) + (k_i_ * integral_) + (k_d_ * derivative);
                message.angular.z = out/100;

                std::cout << "Kp: " << k_p_ 
                        << " Ki: " << k_i_ 
                        << " Kd: " << k_d_ 
                        << " out: " << out << std::endl;

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

void save_waypoints()
{
    std::ofstream file("waypoints.txt");

    for (const auto& wp : waypoints_)
    {
        file << wp.first << " " << wp.second << std::endl;
    }

    file.close();
}

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