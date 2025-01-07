#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
double current_x = 0.0;
double current_y = 0.0;
bool finished = false;
rclcpp::Logger logger = rclcpp::get_logger("robot_snake_movement");


void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    geometry_msgs::msg::Twist cmd_vel_msg;

    if (current_y > 9 && !finished){
    	cmd_vel_msg.linear.x = 0.0;  
        cmd_vel_msg.angular.z = 0.0; 
        finished = true;
        RCLCPP_INFO(logger, "Robot snake completed the task!!!");
    }

    // Direction management
    if (current_x >= 4.0 && !finished)
    {
        cmd_vel_msg.linear.x = 1.0;  
        cmd_vel_msg.angular.z = 3.0; 
    }
    else if (current_x <= -4.0 && !finished)
    {
        cmd_vel_msg.linear.x = 1.0;  
        cmd_vel_msg.angular.z = -3.0;
    }
    else if (!finished)
    {
        cmd_vel_msg.linear.x = 1; 
        cmd_vel_msg.angular.z = 0.0; 
    }
   
    pub->publish(cmd_vel_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_snake_movement");

    // Publisher per inviare i comandi di velocità
    pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscriber per leggere la posizione dal topic /odom
    auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odomCallback);

    RCLCPP_INFO(node->get_logger(), "Robot snake movement node started.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

