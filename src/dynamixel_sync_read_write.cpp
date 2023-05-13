// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//    _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
// ,-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)
// `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-'
// 		        __      __  __  ______  __  __    
// 	(•̀ᴗ•́)و	 /\ \    /\ \/\ \/\  ___\/\ \_\ \    (◍＞◡＜◍)
// 	 ᶘ ◕ᴥ◕ᶅ	  \ \ \___\ \ \_\ \ \___  \ \  __ \  【≽ܫ≼】
// 	(ﾟ◥益◤ﾟ)   \ \_____\ \_____\/\_____\ \_\ \_\  (ʘ言ʘ╬)
// 	 ᕙ(⇀‸↼‶)ᕗ  \/_____/\/_____/\/_____/\/_/\/_/  (◕‿◕✿)
//    _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
// ,-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)
// `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-'
//

// This node implementation is derived from ROBOTIS CO., LTD. example code "read_write_node"
// Example Code Author: Will Son

#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "dynamixel_sync_read_write.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

SyncReadWrite::SyncReadWrite()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting SyncReadWrite Node ...");

  this->declare_parameter("topic", "set_position");
  std::string topic_param = this->get_parameter("topic").as_string();

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
    this->create_subscription<SetPosition>(
    // topic name
    topic_param,
    // qos profile
    QOS_RKL10V,
    // lambda callback function
    [this](const SetPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Goal position localization [mapping might be done here]
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

      // Write packet to dynamixel stack
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_POSITION,
        goal_position,
        &dxl_error
      );

      //callback logging
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
      }

    }

  );
}

SyncReadWrite::~SyncReadWrite()
{
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  return 0;
}
