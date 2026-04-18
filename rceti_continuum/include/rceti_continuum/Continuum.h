/*
 * Continuum.h
 *
 *  Created on: Apr 1, 2017
 *      Author: haitham
 */
/*
MODIFICATION NOTICE
This file is part of a derivative work based on the original RCETI project (https://github.com/bturner86239/RCETI).
It was modified by CSE 2.3 in March, 2026 in accordance with Section 4(b) of the Apache License 2.0.

Major Changes:
Removed all keyboard inputs. Directly reads in input from keyboard through jointStateCallback method. Removed all methods and variables related to robot head.
*/
#ifndef rceti_continuum_INCLUDE_rceti_continuum_CONTINUUM_H_
#define rceti_continuum_INCLUDE_rceti_continuum_CONTINUUM_H_

#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <stdlib.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "rceti_continuum/KinematicsEngine.h"
#include "rceti_continuum/UrdfGenerator.h"

constexpr int RESOLUTION = 100;
constexpr int DELAY = 1;
constexpr int NORMAL = 0;

/**
 * @class Continuum
 * @brief Main controller for the continuum robot simulation and visualization.
 *
 * This class orchestrates the physical simulation of the robot. It listens to incoming 
 * motor commands via rceti_controller, passes them to the KinematicsEngine to calculate 
 * the resulting curve, and broadcasts the updated 3D positions (TF2 frames) and visual 
 * markers so the robot moves in RViz.
 */
class Continuum {
	private:
		/** @brief The total number of independent bending segments making up the tube. */
		int number_of_segments_ = 5;

		/** @brief The length of each segment. */
		double segment_length_ = 0.05;

		/** @brief The number of disks in each segment. */
		int segment_disks_ = 10;

		/** @brief The radius of disks in each segment. */
		double segment_radius_ = 0.004;

		/** @brief Broadcasts the calculated positions of each rigid disk to the ROS 2 TF tree. */
		std::shared_ptr<tf2_ros::TransformBroadcaster> segTFBroadcaster;

		/** @brief Array of visual markers representing the the robot. */
		std::vector<visualization_msgs::msg::MarkerArray> cableMarkers;

		/** @brief Publishes the cable markers to RViz for visualization. */
		std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> cablePublisher;

		/**
		 * @brief Initializes the shape, scale, and color of the visual markers for a segment.
		 * @param segID The ID of the segment to initialize.
		 */
		void initCableMarker(int segID);
		
		/** @brief Subscribes to the /joint_states topic to listen for motor actuation commands. */
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

		/**
		 * @brief Callback triggered when new motor commands are received. 
		 * @details Translates X/Y motor bending inputs into mathematical curvature (kappa) 
		 * and direction (phi), then updates the segment shape.
		 * @param msg The incoming ROS 2 joint state message.
		 */
		void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

		/** @brief Subscribes to the */
		rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
		
	public:
		/** @brief Pointer to the engine handling all constant curvature kinematics math. */
		KinematicsEngine* math_engine_;

		/** @brief Pointer to the engine responsible for generating the dynamic URDF file. */
        UrdfGenerator* urdf_generator_;

		/**
		 * @brief Constructor. Initializes ROS 2 interfaces, parameters, and internal engines.
		 * @param node A shared pointer to the parent ROS 2 node.
		 */
		Continuum(std::shared_ptr<rclcpp::Node> node);

		/**
		 * @brief Destructor. Safely cleans up the dynamically allocated math and URDF engines.
		 */
		~Continuum();
		
		/**
		 * @brief The main execution loop of the node.
		 * @details Iterates through all segments, pulls the latest positional math from 
		 * the KinematicsEngine, calculates the current TF2 frames for every disk, and 
		 * broadcasts them alongside the updated visual markers.
		 */
		void update(void);
};

#endif /* rceti_continuum_INCLUDE_rceti_continuum_CONTINUUM_H_ */
