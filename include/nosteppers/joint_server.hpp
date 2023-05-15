#include <memory>
#include <string>

#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/float32.hpp"

//joint data from coppelia is float, need to map it to integer with minimal accuracy loss.
//integer truncation is really bad in almost every case.
int map(float x, float in_min, float in_max, int out_min, int out_max);

class JointServer : public rclcpp::Node {

    public:
        using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
        using mFloat32 = std_msgs::msg::Float32;

        JointServer();
        virtual ~JointServer();

    private:

        rclcpp::Publisher<SetPosition>::SharedPtr set_position_publisher_;

        //this hack sucks but its 5/14/2023 with final presentation tomorrow and i just want this done
        //i can feel your eyes on it. its rude!
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr j0_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr j1_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr j2_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr j3_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr j4_sub_;

};