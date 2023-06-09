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
 * TDLR: this program joins the controls with the hardware. 
 * 
 * Topics being subscribed to (IMMUTABLE):
 *  j0 , j1 , j2 , j3 , j4
 * 
 * Topics being published to (MUTABLE, DEFAULTED):
 *  set_position
 * 
*****************/

#include "joint_server.hpp"

#define JOINT_0 0
#define JOINT_1 1
#define JOINT_2 2
#define JOINT_3 3
#define JOINT_4 4

#define MAP_RATIO 0.087890625

//maps degrees to dynamixel position values
int map(float x) {
  return round(x / MAP_RATIO); //im like 80% sure there is bad behavior radiating from here
}

void JointServer::openEE() {

    GPIO::output(solenoid_pin_, GPIO::HIGH);

}

void JointServer::closeEE() {

    GPIO::output(solenoid_pin_, GPIO::LOW);

}

JointServer::JointServer() : Node ("joint_server") {

    // BOILER PLATE START ////////////////////////////////////////////////////////////////////////////
    RCLCPP_INFO(this->get_logger(), "Starting Joint Server ... ");

    this->declare_parameter("topic_output", "set_position");
    std::string topic_param = this->get_parameter("topic_output").as_string();

    int8_t qos_depth = 10;
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
    // BOILER PLATE END //////////////////////////////////////////////////////////////////////////////

    set_position_publisher_ = this->create_publisher<SetPosition>(topic_param, QOS_RKL10V);

    // UGLY HACK START //////////////////////////////////////////////////////////////////////////////
    j0_sub_ = this->create_subscription<mFloat32>("j0", QOS_RKL10V,
        [this](const mFloat32::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Heard from j0: %f", msg->data);

            auto filtered_joint_data = SetPosition();
            filtered_joint_data.id = JOINT_0;
            filtered_joint_data.position = map(msg->data);
            set_position_publisher_->publish(filtered_joint_data);
        });

    j1_sub_ = this->create_subscription<mFloat32>("j1", QOS_RKL10V,
        [this](const mFloat32::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Heard from j1: %f", msg->data);

            auto filtered_joint_data = SetPosition();
            filtered_joint_data.id = JOINT_1;
            filtered_joint_data.position = map(msg->data);
            set_position_publisher_->publish(filtered_joint_data);
        });

    j2_sub_ = this->create_subscription<mFloat32>("j2", QOS_RKL10V,
        [this](const mFloat32::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Heard from j2: %f", msg->data);

            auto filtered_joint_data = SetPosition();
            filtered_joint_data.id = JOINT_2;
            filtered_joint_data.position = map(msg->data);
            set_position_publisher_->publish(filtered_joint_data);
        });
    
    j3_sub_ = this->create_subscription<mFloat32>("j3", QOS_RKL10V,
        [this](const mFloat32::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Heard from j3: %f", msg->data);

            auto filtered_joint_data = SetPosition();
            filtered_joint_data.id = JOINT_3;
            filtered_joint_data.position = map(msg->data + 14);
            set_position_publisher_->publish(filtered_joint_data);
        });

    j4_sub_ = this->create_subscription<mFloat32>("j4", QOS_RKL10V,
        [this](const mFloat32::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Heard from j4: %f", msg->data);

            auto filtered_joint_data = SetPosition();
            filtered_joint_data.id = JOINT_4;
            filtered_joint_data.position = map(msg->data);
            set_position_publisher_->publish(filtered_joint_data);
        });
    // UGLY HACK END //////////////////////////////////////////////////////////////////////////////
    // that hurt

    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(solenoid_pin_, GPIO::OUT, GPIO::LOW);

    ee_sub_ = this->create_subscription<std_msgs::msg::Int32>("ee", QOS_RKL10V,
        [this](const std_msgs::msg::Int32::SharedPtr msg) -> void {

            RCLCPP_INFO(this->get_logger(), "Heard from EE: %d", msg->data);

            if(msg->data == 1) openEE();
            else closeEE();

        });
}

JointServer::~JointServer() {}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto jointserver = std::make_shared<JointServer>();
    rclcpp::spin(jointserver);

    rclcpp::shutdown();
}