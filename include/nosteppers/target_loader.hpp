#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/float32.hpp"

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

        TargetLoader(std::vector<Operation> ops_param);
        virtual ~TargetLoader();

        void execute(std::vector<Operation> ops, int ops_idx, bool return_to_neutral);

    private:
        rclcpp::Publisher<mFloat32>::SharedPtr x_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr y_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr z_publisher_;
        rclcpp::Publisher<mFloat32>::SharedPtr theta_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        void loadOperations(std::vector<Operation> ops); // call in construct
        void publishTarget(Target tgt);

        int currOperation_idx;

};



//these are the hard coded operations. ideally these would be placed in a json or xml but
//time is running out and ill just have to accept the build every iterationto adjust
const std::vector<Target> pick_tgts = {

    {100.0f, 100, 100, 90}, 
    {100, 100, 100, 90}


};

const std::vector<Target> place_tgts = {

    {100.0f, 100, 100, 90}, 
    {100, 100, 100, 90}


};