#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

struct Target {

    float x;
    float y;
    float z;
    float theta; //every tgt has the ability to rotate the pose about the z axis (world frame)

};

struct Operation {

    Target pick;
    Target place;

};

class TargetLoader : public rclcpp::Node {

    public:
        using mFloat32 = std_msgs::msg::Float32;
        std::vector<Operation> operations;

        TargetLoader();
        virtual ~TargetLoader();

        void execute(std::vector<Operation> ops, int ops_idx, bool return_to_neutral);

    private:
        rclcpp::Publisher<mFloat32>::SharedPtr x_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr y_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr z_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr theta_publisher_;

        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ee_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        void loadOperations(); // call in construct
        void publishTarget(Target tgt);
        void publishEE(int state);

        int currOperation_idx;
        float blockHeight = 0.075;
        float pickHeight = 0.065;

};


#define TRAVERSE_HEIGHT 0.225

const Target neutral_pos = {-0.1, 0.0, 0.25, 90};
const Target pick_pos = {-0.05, 0.225, TRAVERSE_HEIGHT, 0};

//these are the hard coded operations. ideally these would be placed in a json or xml but
//time is running out and ill just have to accept the build every iterationto adjust
std::array<Target, 21> pick_tgts = {{

    neutral_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos,
    pick_pos
    //3:39AM day of competitiong

}};

std::array<Target, 21> place_tgts= {{

    neutral_pos,
    {-0.25,       -0.1,   TRAVERSE_HEIGHT, 90},
    {-0.25,      -0.05,   TRAVERSE_HEIGHT, 90},
    {-0.25,      0.0,   TRAVERSE_HEIGHT, 90},
    {-0.25,      0.05,   TRAVERSE_HEIGHT, 90},
    {-0.25,        0.1,   TRAVERSE_HEIGHT, 90},

    {-0.25,       -0.1,   TRAVERSE_HEIGHT, 90},
    {-0.25,      -0.05,   TRAVERSE_HEIGHT, 90},
    {-0.25,      0.0,   TRAVERSE_HEIGHT, 90},
    {-0.25,      0.05,   TRAVERSE_HEIGHT, 90},
    {-0.25,        0.1,   TRAVERSE_HEIGHT, 90},


    {-0.25,       -0.1,   TRAVERSE_HEIGHT, 90},
    {-0.25,      -0.05,   TRAVERSE_HEIGHT, 90},
    {-0.25,      0.0,   TRAVERSE_HEIGHT, 90},
    {-0.25,      0.05,   TRAVERSE_HEIGHT, 90},
    {-0.25,        0.1,   TRAVERSE_HEIGHT, 90},


    {-0.25,       -0.1,   TRAVERSE_HEIGHT, 90},
    {-0.25,      -0.05,   TRAVERSE_HEIGHT, 90},
    {-0.25,      0.0,   TRAVERSE_HEIGHT, 90},
    {-0.25,      0.05,   TRAVERSE_HEIGHT, 90},
    {-0.25,        0.1,   TRAVERSE_HEIGHT, 90},





}};