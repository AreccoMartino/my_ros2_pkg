#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "interfaces/srv/stop_go.hpp"
#include "interfaces/msg/pos.hpp"

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
double current_x = 0.0;
double current_y = 0.0;
bool finished = false;
rclcpp::Logger logger = rclcpp::get_logger("robot_snake_movement");
bool stop = false;

// need a pub for the positions 
rclcpp::Publisher<interfaces::msg::Pos>::SharedPtr pos_pub;



void StopGoCallBack(const std::shared_ptr<interfaces::srv::StopGo::Request> request,
                         std::shared_ptr<interfaces::srv::StopGo::Response> response)
{
    stop = request->stop_go;
    response->success = true;
}



void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    geometry_msgs::msg::Twist cmd_vel_msg;
    
    // publishing the positions
    interfaces::msg::Pos position_msg;
    position_msg.x = current_x*3.28;
    position_msg.y = current_y*3.28;
    
    pos_pub->publish(position_msg);


    if (current_y > 9 && !finished){
    	cmd_vel_msg.linear.x = 0.0;  
        cmd_vel_msg.angular.z = 0.0; 
        finished = true;
        RCLCPP_INFO(logger, "Robot snake completed the task!!!");
    }
    
	if (!stop) {
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
    } // if is stop i put to zero the vel
    else {
    	cmd_vel_msg.linear.x = 0.0;  
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
        
    // Service for the velocity input
    auto service = node->create_service<interfaces::srv::StopGo>(
        "/stop_go", &StopGoCallBack);
        
        
    // publisher
    pos_pub = node->create_publisher<interfaces::msg::Pos>("/pos", 10);

    RCLCPP_INFO(node->get_logger(), "Robot snake movement node started.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

