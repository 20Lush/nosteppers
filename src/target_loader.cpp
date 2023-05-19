#include "target_loader.hpp"

using namespace std::chrono_literals;
#define PATH_MAJOR_DELAY 4000ms
#define PATH_MINOR_DELAY 2000ms
#define END_EFFECTOR_DELAY 1000ms

#define LOG_INTERVAL 1000ms

TargetLoader::TargetLoader(std::vector<Operation> ops_param) : Node ("target_loader") {

    int8_t qos_depth = 10;
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    loadOperations(ops_param);

    x_publisher_ = this->create_publisher<mFloat32>("tgt_x", QOS_RKL10V);
    y_publisher_ = this->create_publisher<mFloat32>("tgt_y", QOS_RKL10V);
    z_publisher_ = this->create_publisher<mFloat32>("tgt_z", QOS_RKL10V);
    theta_publisher_ = this->create_publisher<mFloat32>("tgt_theta", QOS_RKL10V);

    timer_ = this->create_wall_timer(LOG_INTERVAL, [this]() -> void {

        RCLCPP_INFO(this->get_logger(), "Serving Operation [%d]", currOperation_idx);

    });

}

TargetLoader::~TargetLoader() {}

void TargetLoader::publishTarget(Target tgt) {

    auto msg_conv = mFloat32();
    msg_conv.data = tgt.x;
    x_publisher_->publish(msg_conv);

    msg_conv.data = tgt.y;
    y_publisher_->publish(msg_conv);

    msg_conv.data = tgt.z;
    z_publisher_->publish(msg_conv);

    msg_conv.data = tgt.theta;
    theta_publisher_->publish(msg_conv);

}

void TargetLoader::loadOperations(std::vector<Operation> ops) {
    //correlate picks to places
    for(uint8_t i=0; i < pick_tgts.size(); i++){

        ops.at(i).pick = pick_tgts.at(i);
        ops.at(i).place = place_tgts.at(i);

    }

}

void TargetLoader::execute(std::vector<Operation> ops, int ops_idx, bool return_to_neutral = false){

    //path logic
    //use rclcpp::sleep_for(chrono literal) defaults to nanoseconds if no literal
    //the sleep wont block the coppelia joint updates because you are so good at programming
    //it will block any parallel logic in this executable though to watch urself

    //path to pick

    //sleep for MAJOR physical response (could eventually be replaced with feedback from physical nodes)

    //lower to manipulation height

    //sleep for MINOR physical response

    //close end effector

    //sleep for END_EFFECTOR physical response

    //raise to traverse height

    //sleep for MAJOR physical respone

    //lower to manipulation height

    //sleep for MINOR physical response

    //open end effector

    //raise to traverse height

    //sleep for MINOR physical response

    //if return_to_neutral == true, goto neutral target
    // ---- sleep for MAJOR physical response

    currOperation_idx++;

}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    std::vector<Operation> operations;
    auto tgt_node = std::make_shared<TargetLoader>(operations);

    for(uint8_t i=0; i <= operations.size(); i++){

        tgt_node->execute(operations, i);
        rclcpp::spin_some(tgt_node); //pretty much just for logging

    }

    rclcpp::shutdown();
}