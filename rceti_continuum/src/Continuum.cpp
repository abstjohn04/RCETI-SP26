/*
 * Continuum.cpp
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
Moved math logic to KinematicsEngine.cpp and urdf management to UrdfGenerator.cpp.
*/

#include "rceti_continuum/Continuum.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

Continuum::Continuum(std::shared_ptr<rclcpp::Node> node)
{
    char cableTopic[30];
    node->declare_parameter("number_of_segments", number_of_segments_);
    node->declare_parameter("segment_length_m", segment_length_);
    node->declare_parameter("segment_disks", segment_disks_);
    node->declare_parameter("segment_radius_m", segment_radius_);

    node->get_parameter("number_of_segments", number_of_segments_);
    node->get_parameter("segment_length_m", segment_length_);
    node->get_parameter("segment_disks", segment_disks_);
    node->get_parameter("segment_radius_m", segment_radius_);

    math_engine_ = new KinematicsEngine(number_of_segments_);
    urdf_generator_ = new UrdfGenerator();

    for (int i = 0; i < number_of_segments_; i++) {
        math_engine_->addSegment(i, segment_length_, segment_disks_,  segment_radius_);

        // 2. Stack them end-to-end along the Z-axis initially
        // Segment 0 starts at Z=0.0, Segment 1 starts at Z=0.05, Segment 2 at Z=0.10, etc.
        tf2::Vector3 initial_pos(0.0, 0.0, i * segment_length_);
        tf2::Quaternion initial_rot;
        initial_rot.setRPY(0, 0, 0); // Pointing straight up initially
        math_engine_->setSegmentBasePose(i, initial_pos, initial_rot);

        // 3. Write this segment's XML blocks into the dynamic URDF file
        urdf_generator_->createURDF(i, segment_length_, segment_disks_, segment_radius_, number_of_segments_);
    }
    segTFBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    cableMarkers.resize(number_of_segments_ + 1);
    cablePublisher.resize(number_of_segments_ + 1);

	joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&Continuum::jointStateCallback, this, std::placeholders::_1));

    for (int segmentNum = 0; segmentNum <= number_of_segments_; segmentNum++)
    {
        sprintf(cableTopic, "cable_%d", segmentNum);
        cableMarkers[segmentNum].markers.resize(RESOLUTION);
        cablePublisher[segmentNum] = node->create_publisher<visualization_msgs::msg::MarkerArray>(cableTopic, 1);
    }
}

Continuum::~Continuum() {
    delete math_engine_;
    delete urdf_generator_;
}

void Continuum::update(void) 
{
    char childFrameName[30];

    for (int segID = 0; segID < number_of_segments_; segID++)
    {
        if (segID > 0) {
            int prev_seg = segID - 1;
            int prev_n_disks = segment_disks_ - 1;
  
            tf2::Vector3 prev_tip_pos = math_engine_->getDiskPosition(prev_seg, prev_n_disks - 1);
            tf2::Quaternion prev_tip_rot = math_engine_->getDiskQuaternion(prev_seg, prev_n_disks - 1);
            tf2::Transform prev_base = math_engine_->getBasePose(prev_seg);
            
            // Calculate absolute world pose: Base_Pos + (Base_Rot * Local_Pos)
            tf2::Vector3 abs_tip_pos = prev_base.getOrigin() + (tf2::Matrix3x3(prev_base.getRotation()) * prev_tip_pos);
            // Calculate absolute rotation: Base_Rot * Local_Rot
            tf2::Quaternion abs_tip_rot = prev_base.getRotation() * prev_tip_rot;
            
            // Snap the current segment's base exactly to the calculated tip
            math_engine_->setSegmentBasePose(segID, abs_tip_pos, abs_tip_rot);
        }
        tf2::Vector3 eePc;

        // Accesses necessary variables from math engine
        double phi = math_engine_->getPhi(segID);
        double kappa = math_engine_->getKappa(segID);
        double length = math_engine_->getSegmentLength(segID);
        tf2::Transform base = math_engine_->getBasePose(segID);

        for (int i = 0; i < segment_disks_ && rclcpp::ok(); i++)
        {
            // Re-use the engine's built-in getDiskPosition
            tf2::Vector3 eeP = math_engine_->getDiskPosition(segID, i);
            eeP = tf2::Matrix3x3(base.getRotation()) * eeP;

            // Fix zeroed base poses safely
            for (int s = 0; s < number_of_segments_; s++) {
                if (math_engine_->getBasePose(s).getOrigin().length() == 0) math_engine_->setSegmentBasePose(s, tf2::Vector3(0, 0, 0), tf2::Quaternion(0, 0, 0, 1));
            }
                        
            // Calculate and broadcast the current frame directly
            tf2::Transform currentFrame;
            currentFrame.setOrigin(tf2::Vector3(base.getOrigin().x() + eeP.getX(), base.getOrigin().y() + eeP.getY(), base.getOrigin().z() + eeP.getZ()));
            currentFrame.setRotation(base.getRotation() * math_engine_->getDiskQuaternion(segID, i));

            sprintf(childFrameName, "S%dL%d", segID, i);
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = rclcpp::Clock().now();
            transformStamped.header.frame_id = "continuum_base_link";
            transformStamped.child_frame_id = childFrameName;
            transformStamped.transform.translation.x = currentFrame.getOrigin().x();
            transformStamped.transform.translation.y = currentFrame.getOrigin().y();
            transformStamped.transform.translation.z = currentFrame.getOrigin().z();

            tf2::Quaternion q = currentFrame.getRotation();
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            segTFBroadcaster->sendTransform(transformStamped);
        }

        if (segID >= number_of_segments_ || segment_disks_ <= 0) continue; 
        cablePublisher[segID]->publish(cableMarkers[segID]);
    }

}

void Continuum::initCableMarker(int segID) {
	uint32_t shape = visualization_msgs::msg::Marker::SPHERE;

	  for (int r=0; r<RESOLUTION; r++) {
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    cableMarkers[segID].markers[r].header.frame_id = "continuum_base_link";
	    cableMarkers[segID].markers[r].header.stamp = rclcpp::Clock().now();

	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
	    cableMarkers[segID].markers[r].ns = "basic_shapes";
	    cableMarkers[segID].markers[r].id = r;

	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	    cableMarkers[segID].markers[r].type = shape;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    cableMarkers[segID].markers[r].action = visualization_msgs::msg::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    cableMarkers[segID].markers[r].scale.x = .008225;//real dimension of physical continuum
	    cableMarkers[segID].markers[r].scale.y = .008225;//real dimension of physical continuum
	    cableMarkers[segID].markers[r].scale.z = .023;//real dimension of physical continuum

	    // Set the color -- be sure to set alpha to something non-zero!
	    cableMarkers[segID].markers[r].color.b = 1.0f;
	    cableMarkers[segID].markers[r].color.a = 1.0;
	    cableMarkers[segID].markers[r].lifetime = rclcpp::Duration(0,0);
	  }
}

void Continuum::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    try {
        double bend_y = 0.0;
        double bend_x = 0.0;
        bool found_y = false;
        bool found_x = false;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "continuum_motor_1") {
                bend_y = msg->position[i];
                found_y = true;
            } else if (msg->name[i] == "continuum_motor_3") {
                bend_x = msg->position[i];
                found_x = true;
            }
        }

        if (found_y && found_x) {
            // 1. Calculate the target Angle (Theta) and Direction (Phi)
            double theta = sqrt(pow(bend_x, 2) + pow(bend_y, 2));
            double phi = atan2(bend_y, bend_x);

            double kappa = theta / segment_length_;

            // 3. Prevent mathematical singularity (Kappa can never be exactly 0)
            if (kappa < 0.0001) kappa = 0.0001;

            // 4. FIX THE RIGID BODY: Loop through and bend EVERY segment
            // This causes the entire robot to curl smoothly like a true continuum snake
            for (int s = 0; s < number_of_segments_; s++) math_engine_->setSegmentShape(s, kappa, phi);
            
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rceti_continuum"), "Error in JointState callback: %s", e.what());
    }
}
