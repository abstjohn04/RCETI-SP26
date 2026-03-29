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

constexpr double PI = 3.1415926;
constexpr int RESOLUTION = 100;
constexpr int DELAY = 1;
constexpr int NORMAL = 0;

class Continuum {
	private:
		std::vector<tf2::Transform> endEffectorPose;
		std::vector<tf2::Transform> basePose;
		std::vector<std::vector<tf2::Transform>> segTFFrame;
		std::shared_ptr<tf2_ros::TransformBroadcaster> segTFBroadcaster;

		std::vector<visualization_msgs::msg::MarkerArray> cableMarkers;
		std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> cablePublisher;

		rclcpp::TimerBase::SharedPtr frame_timer;

		std::vector<double> arrayOfKappa;
		std::vector<double> arrayOfPhi;
		std::vector<double> segmentLength;
		std::vector<int> noOfDisks;
		std::vector<int> segmentMode;
		std::vector<double> segKappa;
		std::vector<double> segPhi;
		
		std::ofstream robotURDFfile;
		void createURDF(int segID, double length, int n_disks, double radius);
		void initCableMarker(int segID);
		tf2::Quaternion getDiskQuaternion(int segID, int diskID);

		tf2::Vector3 getDiskPosition(int segID, int i);
		void timerScanning();
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
		void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

	public:
		Continuum(std::shared_ptr<rclcpp::Node> node);
		int numberOfSegments;

		void addSegment(int segID, double length, int n_disks, double radius);

		void setSegmentBasePose(int segID, tf2::Vector3 basePos, tf2::Quaternion baseRot);
		void setSegmentShape(int segID, double kappa, double phi);
		void update(void);
};

#endif /* rceti_continuum_INCLUDE_rceti_continuum_CONTINUUM_H_ */
