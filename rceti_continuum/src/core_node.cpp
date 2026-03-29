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
    auto node = rclcpp::Node::make_shared("continuum_core");
    
    Continuum robot(node);
    RCLCPP_INFO(node->get_logger(), "Number of segments: %d", robot.numberOfSegments);

    tf2::Quaternion q;
    q.setRPY(0.0, 90 * PI / 180, 0.0);
    double base_x = node->declare_parameter("base_pose.x", 0.0);
    double base_y = node->declare_parameter("base_pose.y", 0.0);
    double base_z = node->declare_parameter("base_pose.z", 0.0);
    robot.setSegmentBasePose(0, tf2::Vector3(base_x, base_y, base_z), q);

    for (int i = 0; i < robot.numberOfSegments; i++)
    {
        robot.addSegment(i, .23, 2, .0003); // SegID, Length, noOfDisks, radius of disk
        robot.setSegmentShape(0, 0.0001, 0);
    }

    // Initialize the default shape
    robot.setSegmentShape(0, 0, 0); 
    robot.update();

    RCLCPP_INFO(node->get_logger(), "Continuum Core initialized. Listening to /joint_states from the Python Keyboard Controller...");

    auto update_timer = node->create_wall_timer(
        std::chrono::milliseconds(50), [&robot]() { robot.update(); }
    );

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}