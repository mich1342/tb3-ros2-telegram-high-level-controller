#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#define linear_gain 0.05
#define angular_gain 0.01
#define linear_limit 1
#define angular_limit 1

//#define M_PI 3.1415926535897932384626433832795

using std::placeholders::_1;
using namespace std::chrono_literals;


class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",100); // publish buat gerakin
      vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10); // publish buat tunjukkin di rviz
      subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&MinimalPublisher::scanCallback, this, _1)); //subsribe buat ngambil data dari Lidar scan
      sub2_ = this->create_subscription<std_msgs::msg::String>(
        "telegram_target",
        rclcpp::SensorDataQoS(),
        std::bind(&MinimalPublisher::telegramCallback, this, _1));
      
    }

  private:
    void telegramCallback(const std_msgs::msg::String::SharedPtr msg)
    {
      std::vector<std::string> temp_result;
      std::stringstream data(msg->data);
      std::string line;
      try{
        while(std::getline(data,line,';'))
        {
          temp_result.push_back(line);
        }
        long new_msg_id = std::stol(temp_result[0]);
        if(new_msg_id == msg_id){
          ;
        }else{
          RCLCPP_INFO(this->get_logger(), msg->data.c_str());
          if(temp_result[1] == "Jog"){
            temp_x = 0;
            temp_a = 0;
            follow_mode = false;
          }
          if(temp_result[1] == "Follow"){
            follow_mode = true;
          }
          if(temp_result[1] == "W"){
            temp_x += linear_gain;
            if(temp_x > linear_limit){
              temp_x = linear_limit;
            }
          }
          if(temp_result[1] == "S"){
            temp_x -= linear_gain;
            if(temp_x < -linear_limit){
              temp_x = -linear_limit;
            }
          }
          if(temp_result[1] == "D"){
            temp_a -= angular_gain;
            if(temp_a < -angular_limit){
              temp_a = -angular_limit;
            }
          }
          if(temp_result[1] == "A"){
            temp_a += angular_gain;
            if(temp_a > angular_limit){
              temp_a = angular_limit;
            }
          }
          if(temp_result[1] == "X"){
            temp_a = 0;
            temp_x = 0;
          }
          msg_id = new_msg_id;
        }
        
      }catch(const std::exception& e){
        ;
      }
      
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      x_pillar = (float)(rand()%10) / 10.0;
      y_pillar = 0.1;
      alpha_pillar = 1;
      publisher_->publish(message);
      pController();
      cmd_pub_->publish(vel_msg_);
      i += 0.5;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {
    	float smallest_distance = 100;
      int arr_size = floor((scan_msg->angle_max-scan_msg->angle_min)/scan_msg->angle_increment);
    for (int i=0 ; i< arr_size ;i++)
    {
        if(scan_msg->ranges[i] < smallest_distance){
            smallest_distance = scan_msg->ranges[i];
            alpha_pillar = (scan_msg->angle_min + i*scan_msg->angle_increment);

        }
    }
    x_pillar = smallest_distance*cos(alpha_pillar);
    y_pillar = smallest_distance*sin(alpha_pillar);
    	RCLCPP_INFO(this->get_logger(), "Pillar offset angle(rad):%lf", alpha_pillar);
	    RCLCPP_INFO(this->get_logger(), "pillar x distance(m):%lf", x_pillar);
	    RCLCPP_INFO(this->get_logger(), "pillar y distance(m):%lf", y_pillar);

    if(follow_mode){
      pController(); 
    }else{
      jog_mode();
    }

    //visMsg();
    cmd_pub_->publish(vel_msg_); //harus publish vel_msg_, karena ini variabel milik si topik /cmd_vel
    vis_pub_->publish( marker ); //publish marker, krn ini variabel punya topik /marker
    }
    
    // fungsi buat linear sama angular conditioning, kalo sensor baca segini geraknya gimana, tpi baru jalan kalo di turtlebot3
    void pController()
    {
	    float p_gain_vel = 0.1;
      float p_gain_ang = 0.2;
       RCLCPP_INFO(this->get_logger(), "X_Pillar: '%f'", x_pillar);
       if (x_pillar > 0.2)
        {
            if (x_pillar <= 0.3)
            {
                vel_msg_.linear.x = 0;
                vel_msg_.linear.y = 0; 
                vel_msg_.angular.z =0;

            }
            else 
            {
                vel_msg_.linear.x = x_pillar * p_gain_vel  ;
                //vel_msg_.linear.y = y_pillar * p_gain_vel ;
                if(alpha_pillar > M_PI)
                {   alpha_pillar = alpha_pillar - 2*M_PI;
                    vel_msg_.angular.z = (alpha_pillar * p_gain_ang ) ;
                }
                else
                {
                    vel_msg_.angular.z = (alpha_pillar * p_gain_ang ) ;
                }
            }
 
       }
       else if(x_pillar < -0.2)
       {
           p_gain_ang = 1;
           if(alpha_pillar > M_PI)
                {   alpha_pillar = alpha_pillar - 2*M_PI;
                    vel_msg_.angular.z = (alpha_pillar * p_gain_ang ) ;
                }
                else
                {
                    vel_msg_.angular.z = (alpha_pillar * p_gain_ang ) ;
                }
       }
       else
       {
            vel_msg_.linear.x = 0;
            vel_msg_.linear.y = 0; 
            vel_msg_.angular.z = 0;
       }
    }
    void jog_mode()
    {
	    vel_msg_.linear.x = temp_x;
      vel_msg_.linear.y = 0; 
      vel_msg_.angular.z = temp_a;
    }
    
    // function buat publish data ke rviz
    void visMsg()
    {
    	marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Time();
        marker.ns = "pillar";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x_pillar;
        marker.pose.position.y = y_pillar; 
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }

    float i = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // new declaration
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; //ini buat publisher gerak
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_; //ini buat publisher nunjukin data di Rviz
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_; //ini buat subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;

    float x_pillar = 0;
    float y_pillar = 0; 
    float alpha_pillar = 0; // ini di define sebagai global variable karena dipake di scanCallback sama di pController
    float temp_x = 0;
    float temp_a = 0;
    geometry_msgs::msg::Twist vel_msg_; // assign variable buat vel_msg_
    visualization_msgs::msg::Marker marker; // assign variable buat marker, marker itu buat ngisi data ke rviznya
    size_t count_;
    long msg_id = -1;
    bool follow_mode = false;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }