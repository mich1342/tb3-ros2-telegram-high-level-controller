#include "httplib.h"
#include <chrono>
#include <functional>
#include <memory>

#include <iostream>
#include <bits/stdc++.h>

#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
httplib::Client cli("node-server-dummy.herokuapp.com");

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("telegram_pub"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("telegram_target", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      if (auto res = cli.Get("/lastmessage")) {
        if (res->status == 200) {
          message.data = res->body;
          publisher_->publish(message);   
        }
      }     
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}