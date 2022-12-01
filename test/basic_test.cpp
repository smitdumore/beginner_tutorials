// Description: Test if a simple task plan works

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
    auto mytestpub = mynode->create_publisher<std_msgs::msg::String> ("chatter", 5.0);
    int pubs_num = mynode->count_publishers("chatter");
    EXPECT_EQ(1, pubs_num);
}
