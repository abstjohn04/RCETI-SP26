#include "rceti_continuum/KinematicsEngine.h"

KinematicsEngine::KinematicsEngine(int number_of_segments) {
    segment_lengths_.resize(number_of_segments);
    segmentMode.resize(number_of_segments);
    noOfDisks.resize(number_of_segments);
    endEffectorPose.resize(number_of_segments);
    basePose.resize(number_of_segments);
    segKappa.resize(number_of_segments);
    segPhi.resize(number_of_segments);
    segTFFrame.resize(number_of_segments); 
}

tf2::Quaternion KinematicsEngine::getDiskQuaternion(int segID, int diskID) {
    int disks = noOfDisks[segID];
    if (disks <= 1) disks = 2; 

    double phi = segPhi[segID];

    double theta = segKappa[segID] * (diskID / ((double)disks - 1.0)) * segment_lengths_[segID];

    double c_phi = cos(phi);
    double s_phi = sin(phi);
    double c_theta = cos(theta);
    double s_theta = sin(theta);

    tf2::Matrix3x3 Rot;
    Rot.setValue(
        pow(c_phi, 2) * (c_theta - 1) + 1, s_phi * c_phi * (c_theta - 1), -c_phi * s_theta,
        s_phi * c_phi * (c_theta - 1), pow(c_phi, 2) * (1 - c_theta) + c_theta, -s_phi * s_theta,
        c_phi * s_theta, s_phi * s_theta, c_theta
    );

    tf2::Quaternion qRot;
    Rot.getRotation(qRot);
    return qRot;
}

tf2::Vector3 KinematicsEngine::getDiskPosition(int segID, int diskID) {
    int disks = noOfDisks[segID];
    if (disks <= 1) disks = 2;

    double kappa = segKappa[segID];
    double phi = segPhi[segID];
    
    double arc_length = (diskID / ((double)disks - 1.0)) * segment_lengths_[segID];
    double theta = kappa * arc_length;

    tf2::Vector3 eeP;
    eeP[0] = cos(phi) * (cos(theta) - 1) / kappa;
    eeP[1] = sin(phi) * (cos(theta) - 1) / kappa;
    eeP[2] = sin(theta) / kappa;
    
    return eeP;
}

void KinematicsEngine::addSegment(int segID, double segment_length, int segment_disks, double segment_radius) {
	segTFFrame[segID].resize(segment_disks);

	segment_lengths_[segID] = segment_length;
	noOfDisks[segID] = segment_disks;

    basePose[segID].setOrigin(endEffectorPose[segID-1].getOrigin());
    basePose[segID].setRotation(endEffectorPose[segID-1].getRotation());
	
	segKappa[segID] = 0.00001;
	segPhi[segID] = 0.0;
}

void KinematicsEngine::setSegmentBasePose(int segID, tf2::Vector3 basePos, tf2::Quaternion baseRot)
{
	basePose[segID].setOrigin(basePos);
	basePose[segID].setRotation(baseRot);
}

void KinematicsEngine::setSegmentShape(int segID, double kappa, double phi){
	if(kappa == 0) kappa = 0.0000001;
	segKappa[segID] = kappa;
	segPhi[segID] = phi;
	tf2::Matrix3x3 Rot;
	tf2::Quaternion qRot;
	Rot.setValue(pow(cos(phi),2) * (cos(kappa*segment_lengths_[segID]) - 1) + 1, sin(phi)*cos(phi)*( cos(kappa*segment_lengths_[segID]) - 1), -cos(phi)*sin(kappa*segment_lengths_[segID]),
							sin(phi)*cos(phi)*( cos(kappa*segment_lengths_[segID]) - 1), pow(cos(phi),2) * ( 1 - cos(kappa*segment_lengths_[segID]) ) + cos( kappa * segment_lengths_[segID] ),  -sin(phi)*sin(kappa*segment_lengths_[segID]),
							cos(phi)*sin(kappa*segment_lengths_[segID]),  sin(phi)*sin(kappa*segment_lengths_[segID]), cos(kappa*segment_lengths_[segID]));
	Rot.getRotation(qRot);
	endEffectorPose[segID].setRotation(basePose[segID].getRotation() * qRot);

	tf2::Vector3 eePosition = basePose[segID].getOrigin() + ( tf2::Matrix3x3(basePose[segID].getRotation())*tf2::Vector3(cos(phi)*( cos(kappa*segment_lengths_[segID]) - 1)/kappa, sin(phi)*( cos(kappa*segment_lengths_[segID]) - 1)/kappa, sin(kappa*segment_lengths_[segID])/kappa));
	endEffectorPose[segID].setOrigin(eePosition);
}