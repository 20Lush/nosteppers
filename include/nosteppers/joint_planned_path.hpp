#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/float32.hpp"

#define MAX_STATES 7 //8 states, n-1

class PlannedPath : public rclcpp::Node {

    public:
        using mFloat32 = std_msgs::msg::Float32;

        PlannedPath();
        virtual ~PlannedPath();

    private:

        //disgusting
        rclcpp::Publisher<mFloat32>::SharedPtr j0_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr j1_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr j2_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr j3_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr j4_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        int currState_idx;


};

//i know. ok? i know.
const std::vector<std::vector<float>> states = {    {180, 180, 180, 180, 180},
                                                    {140, 140, 140, 140, 140},
                                                    {240, 100, 210, 90, 120},
                                                    {220, 180, 200, 230, 210},
                                                    {180, 120, 180, 180, 260},
                                                    {90, 80, 260, 100, 90},
                                                    {230, 120, 200, 200, 180},
                                                    {160, 200, 100, 90, 180}  };