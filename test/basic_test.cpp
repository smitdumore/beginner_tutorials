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
 *  @file    basic_test.cpp
 *  @author  Smit Dumore
 *  @date    12/1/2022
 *  @version 1.0
 *
 *  @brief This file implements a ROS2 level 2 test 
 *
 */
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include <gtest/gtest.h>
#include "std_msgs/msg/string.hpp"

class TestingClass : public testing::Test {
    /*
    public:
        TestingClass() 
            : my_node_(std::make_shared<rclcpp::Node>("basic_test"))
            {
                RCLCPP_ERROR_STREAM(my_node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
            }
    */
    protected:
        rclcpp::Node::SharedPtr mynode;
};

TEST_F(TestingClass, TalkerTest) {
    //std::cout << "TEST BEGINNING!!" << std::endl;
    mynode = rclcpp::Node::make_shared("publishertestnode");
    auto mytestpub = mynode->create_publisher
                        <std_msgs::msg::String> ("chatter", 5.0);
    int pubs_num = mynode->count_publishers("chatter");
    EXPECT_EQ(1, pubs_num);
}
