#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

class SyncReadWriteTester : public rclcpp::Node {

    public:
        using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
    
        SyncReadWriteTester(int *theta_iter);
        virtual ~SyncReadWriteTester();

        int desired_position;

    private:
        rclcpp::Publisher<SetPosition>::SharedPtr set_position_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        int id_param;

};