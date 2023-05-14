#include <memory>
#include <chrono>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <dynamixel_srw_tester.hpp>

using namespace std::chrono_literals;

#define CALLBACK_INTERVAL 500ms

SyncReadWriteTester::SyncReadWriteTester(int *theta_iter) : Node("srw_tester") {

    // SetPosition characteristics
    // msg->position
    // msg->id

    // id param
    this->declare_parameter("id", 0);
    id_param = this->get_parameter("id").as_int();

    // topic param
    this->declare_parameter("topic", "set_position");
    std::string topic_param = this->get_parameter("topic").as_string();

    // qos depth param
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);
    const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    set_position_publisher_ = this->create_publisher<SetPosition>(topic_param, QOS_RKL10V);

    timer_ = this->create_wall_timer( CALLBACK_INTERVAL, [this, theta_iter]()->void {

        auto message = SetPosition();
        message.id = id_param;
        message.position = desired_position;
        set_position_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing to ID_'%d': %d", message.id, message.position);

        *theta_iter += 0.5;

    });
}

SyncReadWriteTester::~SyncReadWriteTester()
{
}

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);

    int theta = 0;
    int width = 100;
    int shift = 10;

    auto tester = std::make_shared<SyncReadWriteTester>(&theta);

    while(rclcpp::ok()){

        tester->desired_position = (sin(theta) + shift) * width * 0.5;

        rclcpp::spin_some(tester);

        theta += 1;

    }

    rclcpp::shutdown();

}