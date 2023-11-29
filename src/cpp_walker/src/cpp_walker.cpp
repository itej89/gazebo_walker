// Copyright 2016 Open Source Robotics Foundation, Inc.
//
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

/**
 * @file cpp_walker.cpp
 * @author Tej Kiran (itej89@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <functional>
#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;

/**
 * @brief ROS Node Class that demostrates the subscriber functionality
 *
 */
class cpp_walker : public rclcpp::Node {
 public:
  cpp_walker() : Node("cpp_walker") {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing cpp_walker.");

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Created subscriber");

    /**
     * @brief Create a publisher to the topic "cmd_vel"
     *
     */
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", 10);

    /**
     * @brief Create the subscription to the "scan"
     *
     */
    subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&cpp_walker::topic_callback, this, _1));

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Initialization of cpp_walker done.");
  }

 private:
  /**
   * @brief Checks the robots preripheral vision for obstacles
   * from laser scan data
   * 
   * @param msg : sensor_msgs::msg::LaserScan lidar sensor data
   * @return true : if path is obstacle free
   * @return false : if path contains obstacels
   */
  bool checkFreeSpace(const sensor_msgs::msg::LaserScan& msg) const {
    bool isFreeSpace = true;
    /**
     * @brief Check Right side peripheral vision up to 45 samples
     * Note: Total 360 samples for 360 degrees
     */

    auto elem = std::find_if(msg.ranges.begin(), msg.ranges.begin()+45,
        [](float r){ return (r < 0.5);
    });

    if (elem != msg.ranges.begin()+45) {
       isFreeSpace = false;
    } else {
    /**
     * @brief Check Left side peripheral vision up to 45 samples
     * Note: Total 360 samples for 360 degrees
     */
      elem = std::find_if(msg.ranges.begin()+325, msg.ranges.end(), [](float r){
        return (r < 0.5);
      });

      if (elem != msg.ranges.end()) {
        isFreeSpace = false;
      }
    }

    return isFreeSpace;
  }

  /**
   * @brief Create a callback for the topic
   *
   * @param msg
   */
  void topic_callback(const sensor_msgs::msg::LaserScan& msg) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.ranges[0]);

    auto vel = geometry_msgs::msg::Twist();

    bool isFreeSpace = checkFreeSpace(msg);
    if (isFreeSpace) {
      vel.linear.x = 0.3;
    } else {
        vel.angular.z = 0.3;
    }

    publisher_->publish(vel);
  }

  /**
   * @brief Pointer for adding subscription
   *
   */
  rclcpp::Subscription<sensor_msgs::msg ::LaserScan>::SharedPtr
      subscription_;

  /**
   * @brief Pointer for the publisher
   *
   */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

/**
 * @brief Main funciton  for the subscriber node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cpp_walker>());
  rclcpp::shutdown();
  return 0;
}
