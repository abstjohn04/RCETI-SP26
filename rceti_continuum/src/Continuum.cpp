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

    node->declare_parameter("number_of_sections", 2);
    node->get_parameter("number_of_sections", numberOfSegments);

    math_engine_ = new KinematicsEngine(numberOfSegments);
    urdf_generator_ = new UrdfGenerator();

    segTFBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    cableMarkers.resize(numberOfSegments + 1);
    cablePublisher.resize(numberOfSegments + 1);

	joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&Continuum::jointStateCallback, this, std::placeholders::_1));

    for (int segmentNum = 0; segmentNum <= numberOfSegments; segmentNum++)
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

void Continuum::addSegment(int segID, double length, int n_disks, double radius) {
    urdf_generator_->createURDF(segID, length, n_disks, radius, numberOfSegments);
    math_engine_->addSegment(segID, length, n_disks, radius);
    initCableMarker(segID);
}

void Continuum::setSegmentBasePose(int segID, tf2::Vector3 basePos, tf2::Quaternion baseRot) {
    math_engine_->setSegmentBasePose(segID, basePos, baseRot);
}

void Continuum::setSegmentShape(int segID, double kappa, double phi) {
    math_engine_->setSegmentShape(segID, kappa, phi);
}

void Continuum::update(void) 
{
    char childFrameName[30];
    rclcpp::Rate rate(15);

    // Updates the location of each segment of the tube
    for (int segID = 0; segID < numberOfSegments; segID++)
    {
        tf2::Vector3 eePc;

        // Accesses necessary variables from math engine
        int n_disks = math_engine_->getNoOfDisks(segID);
        double phi = math_engine_->getPhi(segID);
        double kappa = math_engine_->getKappa(segID);
        double length = math_engine_->getSegmentLength(segID);
        tf2::Transform base = math_engine_->getBasePose(segID);

        for(int i = 0; i < n_disks && rclcpp::ok(); i++)
        {
            // Re-use the engine's built-in getDiskPosition
            tf2::Vector3 eeP = math_engine_->getDiskPosition(segID, i);
            eeP = tf2::Matrix3x3(base.getRotation()) * eeP;

            // Fix zeroed base poses safely
            for (int s = 0; s < numberOfSegments; s++) {
                if (math_engine_->getBasePose(s).getOrigin().length() == 0) {
                    math_engine_->setSegmentBasePose(s, tf2::Vector3(0, 0, 0), tf2::Quaternion(0, 0, 0, 1));
                }
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

        if (segID >= numberOfSegments || n_disks <= 0) continue; 

        for (int i = 0; i < RESOLUTION && rclcpp::ok(); i++)
        {
            eePc[0] = cos(phi)*(cos(kappa*((i/((double)RESOLUTION-1.0))*length)) - 1)/kappa;
            eePc[1] = sin(phi)*( cos(kappa*((i/((double)RESOLUTION-1.0))*length)) - 1)/kappa;
            eePc[2] = (sin(kappa*((i/((double)RESOLUTION-1.0))*length))/kappa);

            eePc = tf2::Matrix3x3(base.getRotation()) * eePc;
            cableMarkers[segID].markers[i].pose.position.x = base.getOrigin().x() + eePc[0];
            cableMarkers[segID].markers[i].pose.position.y = base.getOrigin().y() + eePc[1];
            cableMarkers[segID].markers[i].pose.position.z = base.getOrigin().z() + eePc[2];
            
            cableMarkers[segID].markers[i].pose.orientation.x = 0;
            cableMarkers[segID].markers[i].pose.orientation.y = 0;
            cableMarkers[segID].markers[i].pose.orientation.z = 0;
            cableMarkers[segID].markers[i].pose.orientation.w = 1;
        }
        cablePublisher[segID]->publish(cableMarkers[segID]);
    }
    rate.sleep();
}

void Continuum::initCableMarker(int segID){
	uint32_t shape = visualization_msgs::msg::Marker::SPHERE;

	  for(int r=0; r<RESOLUTION; r++)
	  {

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
            double kappa = sqrt(pow(bend_x, 2) + pow(bend_y, 2));
            if (kappa == 0.0) kappa = 0.0000001;
            double phi = atan2(bend_y, bend_x);
            setSegmentShape(0, kappa, phi);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rceti_continuum"), "Error in JointState callback: %s", e.what());
    }
}
