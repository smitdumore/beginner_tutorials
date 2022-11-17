/********************************************************************
 * Copyright (c) 2022 Smit Dumore
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/
/**
 *  @file    publisher_member_function.cpp
 *  @author  Smit Dumore
 *  @date    11/16/2022
 *  @version 2.0
 *
 *  @brief This file implements a ROS2 publisher ans also acts as a
 *          ROS2 server to add 2 integers 
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
size_t sum_;


/**
 * @brief Minimal Publisher Class to handle ros publisher and server
 * 
 */
class MinimalPublisher : public rclcpp::Node {
public:
  /**
  * @brief Minimal Publisher Class constructor
  * 
  */
  MinimalPublisher() : Node("minimal_publisher") {

    RCLCPP_DEBUG_STREAM(this->get_logger(), "MinimalPublisher class created");
    
    // initialize publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // initialize timer callback
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

    // initialize server
    auto serviceCallbackPtr =
      std::bind(&MinimalPublisher::add, this, std::placeholders::_1, std::placeholders::_2);
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", serviceCallbackPtr);
  }

private:
  /**
  * @brief timer call back to publish message
  * 
  */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = std::to_string(sum_);
    RCLCPP_WARN_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
  }

  /**
  * @brief Service callback
  * 
  */
  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    sum_ = response->sum;

    if(sum_ > INT32_MAX){
      RCLCPP_ERROR_STREAM(this->get_logger(), "sum value overflow !!");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
  }

  //members
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  size_t sum;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),"Shutting down");
  rclcpp::shutdown();
  return 0;
}
