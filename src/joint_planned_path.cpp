//    _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
// ,-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)
// `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-'
// 		      __      __  __  ______  __  __    
//  (•̀ᴗ•́)و   /\ \    /\ \/\ \/\  ___\/\ \_\ \    (◍＞◡＜◍)
// 	 ᶘ ◕ᴥ◕ᶅ	  \ \ \___\ \ \_\ \ \___  \ \  __ \  【≽ܫ≼】
// 	(ﾟ◥益◤ﾟ)   \ \_____\ \_____\/\_____\ \_\ \_\  (ʘ言ʘ╬)
// 	 ᕙ(⇀‸↼‶)ᕗ  \/_____/\/_____/\/_____/\/_/\/_/  (◕‿◕✿)
//    _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
// ,-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)
// `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-'
//

// USAGE :: 
/****************
 * 
 * This executable is the middleware that is responsible for the ingress of individual joint
 * controller data, the translation of that data to an agreed upon format (deg, rad, or tick),
 * and the republication of that data to a topic being monitored by the Dynamixel SDK implementation.
 * 
 * This executable has no conception of the origin of the data, it will blindly serve a filtered input
 * to the servo subroutine. It is austere and should run in the background, parallel to the Dynamixel SRW
 * process also in this project.
 * 
 * TDLR: this program provides 
 * 
 * Topics being published to (IMMUTABLE):
 *  j0 , j1 , j2 , j3 , j4
 * 
*****************/

#include "joint_planned_path.hpp"

using namespace std::chrono_literals;
#define PUBLISH_INTERVAL 2000ms

PlannedPath::PlannedPath() : Node ("planned_path") {

    int8_t qos_depth = 10;
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    currState_idx = 0;

    j0_publisher_ = this->create_publisher<mFloat32>("j0", QOS_RKL10V);
    j1_publisher_ = this->create_publisher<mFloat32>("j1", QOS_RKL10V);
    j2_publisher_ = this->create_publisher<mFloat32>("j2", QOS_RKL10V);
    j3_publisher_ = this->create_publisher<mFloat32>("j3", QOS_RKL10V);
    j4_publisher_ = this->create_publisher<mFloat32>("j4", QOS_RKL10V);

    timer_ = this->create_wall_timer(PUBLISH_INTERVAL, [this]() -> void {
        //publish everybody's joint positions at PUBLISH_INTERVAL
        std::vector<float> currState = states[currState_idx];

        //LOL
        RCLCPP_INFO(this->get_logger(), "Publishing state: %f, %f, %f, %f, %f",
                                                            currState[0],
                                                            currState[1],
                                                            currState[2],
                                                            currState[3],
                                                            currState[4]
                                                            );
        //HERE WE GO LMFAOOOOOOO
        auto omnibus_message = mFloat32();

        omnibus_message.data = currState[0];
        j0_publisher_->publish(omnibus_message);

        omnibus_message.data = currState[1];
        j1_publisher_->publish(omnibus_message);
        
        omnibus_message.data = currState[2];
        j2_publisher_->publish(omnibus_message);
        
        omnibus_message.data = currState[3];
        j3_publisher_->publish(omnibus_message);

        omnibus_message.data = currState[4];
        j4_publisher_->publish(omnibus_message);

        currState_idx += 1;
        if(currState_idx > MAX_STATES) currState_idx = 0;
        //thank god and garbage collection
    });

}

PlannedPath::~PlannedPath(){}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlannedPath>();
    rclcpp::spin(node);

    rclcpp::shutdown();
}