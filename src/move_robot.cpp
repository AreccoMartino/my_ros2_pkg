#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  // init ros 2 and node generation
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_robot_node");

  
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  float linear_velocity = 0.0f;
  float angular_velocity = 0.0f;


  while (rclcpp::ok()) {

    std::cout << "Linear velocity (m/s): ";
    std::cin >> linear_velocity;

    std::cout << "Angular velocity (rad/s): ";
    std::cin >> angular_velocity;

    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    publisher->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: linear.x: '%f', angular.z: '%f'", msg.linear.x, msg.angular.z);

    // pause to not overrun the cicle
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }


  rclcpp::shutdown();
  return 0;
}

