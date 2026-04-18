/*
MODIFICATION NOTICE
This file is part of a derivative work based on the original RCETI project (https://github.com/bturner86239/RCETI).
It was modified by CSE 2.3 in March, 2026 in accordance with Section 4(b) of the Apache License 2.0.

Major Changes:
Removed all keyboard inputs. Hands control back to ROS.
*/
#include <iostream>
#include "rceti_continuum/Continuum.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <termios.h> // For terminal input settings
#include <unistd.h>  // For read()

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("core_node");
    
    Continuum robot(node);

    robot.update();

    RCLCPP_INFO(node->get_logger(), "Continuum Core initialized. Listening to /joint_states...");

    double update_rate_hz = 15.0; 
    if (node->has_parameter("update_rate_hz")) update_rate_hz = node->get_parameter("update_rate_hz").as_double();
    else update_rate_hz = node->declare_parameter("update_rate_hz", 15.0);
    
    
    int timer_ms = static_cast<int>(1000.0 / update_rate_hz);

    auto update_timer = node->create_wall_timer(std::chrono::milliseconds(timer_ms), [&robot]() { robot.update(); });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}