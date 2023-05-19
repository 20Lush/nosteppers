#include "target_loader.hpp"

using namespace std::chrono_literals;
#define PATH_MAJOR_DELAY 4000ms
#define PATH_MINOR_DELAY 2000ms
#define END_EFFECTOR_DELAY 1000ms

#define LOG_INTERVAL 1000ms

// the more i look at this the worse it seems
#define CLOSE 0
#define OPEN 1

TargetLoader::TargetLoader() : Node ("target_loader") {

    int8_t qos_depth = 10;
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    loadOperations();

    x_publisher_ = this->create_publisher<mFloat32>("tgt_x", QOS_RKL10V);
    y_publisher_ = this->create_publisher<mFloat32>("tgt_y", QOS_RKL10V);
    z_publisher_ = this->create_publisher<mFloat32>("tgt_z", QOS_RKL10V);
    theta_publisher_ = this->create_publisher<mFloat32>("tgt_theta", QOS_RKL10V);

    ee_publisher_ = this->create_publisher<std_msgs::msg::Int32>("ee", QOS_RKL10V);

    publishEE(OPEN);

    timer_ = this->create_wall_timer(LOG_INTERVAL, [this]() -> void {

        //RCLCPP_INFO(this->get_logger(), "Serving Operation [%d]", currOperation_idx);

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

void TargetLoader::publishEE(int state) {

    

    auto msg = std_msgs::msg::Int32();
    msg.data = state;
    ee_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "published on EE");
    

}

void TargetLoader::loadOperations() {
    //correlate picks to places
    RCLCPP_INFO(this->get_logger(), "loading op %d", pick_tgts.size());
    for(uint8_t i=0; i < pick_tgts.size(); i++){

        operations.push_back({pick_tgts.at(i), place_tgts.at(i)});

    }
    RCLCPP_INFO(this->get_logger(), "ops loaded: %d", operations.size());

}
 

void TargetLoader::execute(std::vector<Operation> ops, int ops_idx, bool return_to_neutral = false){

    //path logic
    //use rclcpp::sleep_for(chrono literal) defaults to nanoseconds if no literal
    //the sleep wont block the coppelia joint updates because you are so good at programming
    //it will block any parallel logic in this executable though to watch urself

    //path to pick
    publishTarget(ops.at(ops_idx).pick);
    RCLCPP_INFO(this->get_logger(), "pick");

    //sleep for MAJOR physical response (could eventually be replaced with feedback from physical nodes)
    rclcpp::sleep_for(PATH_MAJOR_DELAY);

    //lower to manipulation height
    publishTarget({ ops.at(ops_idx).pick.x, ops.at(ops_idx).pick.y, pickHeight, ops.at(ops_idx).pick.theta});

    //sleep for MINOR physical response
    rclcpp::sleep_for(PATH_MINOR_DELAY);

    //close end effector
    publishEE(CLOSE);
    RCLCPP_INFO(this->get_logger(), "EE CLOSE");

    //sleep for END_EFFECTOR physical response
    rclcpp::sleep_for(END_EFFECTOR_DELAY);

    //raise to traverse height
    publishTarget(ops.at(ops_idx).pick);

    //sleep for MINOR physical respones
    rclcpp::sleep_for(PATH_MINOR_DELAY);

    //path to place
    publishTarget(ops.at(ops_idx).place);
    RCLCPP_INFO(this->get_logger(), "place");

    //sleep for MAJOR physical response
    rclcpp::sleep_for(PATH_MAJOR_DELAY);

    //lower to manipulation height
    publishTarget({ ops.at(ops_idx).place.x, ops.at(ops_idx).place.y, blockHeight, ops.at(ops_idx).place.theta});

    //sleep for MINOR physical response
    rclcpp::sleep_for(PATH_MINOR_DELAY);

    //open end effector
    publishEE(OPEN);
    RCLCPP_INFO(this->get_logger(), "EE OPEN");

    //raise to traverse height
    publishTarget(ops.at(ops_idx).place);

    //sleep for MINOR physical response
    rclcpp::sleep_for(PATH_MINOR_DELAY);

    //if return_to_neutral == true, goto neutral target
    // ---- sleep for MAJOR physical response
    if(return_to_neutral){
        publishTarget(neutral_pos);
        rclcpp::sleep_for(PATH_MAJOR_DELAY);
    }

    currOperation_idx++;

    if(currOperation_idx % 5 == 0) {
        RCLCPP_INFO(this->get_logger(), "layer complete, increasing block height from %f to %f", blockHeight, blockHeight + 0.015);
        blockHeight += 0.015;
        }

}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);


    auto tgt_node = std::make_shared<TargetLoader>();

    for(uint8_t i=0; i <= tgt_node->operations.size(); i++){

        RCLCPP_INFO(tgt_node->get_logger(), "loaded command");
         //pretty much just for logging
        tgt_node->execute(tgt_node->operations, i, true);
        rclcpp::spin_some(tgt_node);

    }

    rclcpp::shutdown();
}