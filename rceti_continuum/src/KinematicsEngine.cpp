#include "rceti_continuum/KinematicsEngine.h"

KinematicsEngine::KinematicsEngine(int num_segments) {
    numberOfSegments = num_segments;

    segmentLength.resize(numberOfSegments);
    segmentMode.resize(numberOfSegments);
    noOfDisks.resize(numberOfSegments);
    endEffectorPose.resize(numberOfSegments);
    basePose.resize(numberOfSegments);
    segKappa.resize(numberOfSegments);
    segPhi.resize(numberOfSegments);
    
    // Resize the outer layer of the 2D frame array
    segTFFrame.resize(numberOfSegments); 
}

tf2::Quaternion KinematicsEngine::getDiskQuaternion(int segID, int diskID){
	tf2::Matrix3x3 Rot;
	tf2::Quaternion qRot;
	Rot.setValue(pow(cos(segPhi[segID]),2) * (cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1) + 1, sin(segPhi[segID])*cos(segPhi[segID])*( cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1), -cos(segPhi[segID])*sin(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])),
							sin(segPhi[segID])*cos(segPhi[segID])*( cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1), pow(cos(segPhi[segID]),2) * ( 1 - cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])) ) + cos( segKappa[segID] * ((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])),  -sin(segPhi[segID])*sin(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])),
							cos(segPhi[segID])*sin(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])),  sin(segPhi[segID])*sin(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])), cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])));
	Rot.getRotation(qRot);
	return qRot;
}

tf2::Vector3 KinematicsEngine::getDiskPosition(int segID, int i)
{
	tf2::Vector3 eeP;
	eeP[0] = cos(segPhi[segID])*(cos(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1)/segKappa[segID];
	eeP[1] = sin(segPhi[segID])*( cos(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1)/segKappa[segID];
	eeP[2] = (sin(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID]))/segKappa[segID]);
	return eeP;
}

void KinematicsEngine::addSegment(int segID, double segLength, int n_disks, double radius){	// TODO Auto-generated constructor stub
	segTFFrame[segID].resize(n_disks);

	segmentLength[segID] = segLength;
	noOfDisks[segID] = n_disks;

	if(segID >0)
	{
		basePose[segID].setOrigin(endEffectorPose[segID-1].getOrigin());
		basePose[segID].setRotation(endEffectorPose[segID-1].getRotation());
	}
	else
	{
		basePose[segID].setOrigin(tf2::Vector3(0,0,0));
		basePose[segID].setRotation(tf2::Quaternion(0,0,0,1));
		endEffectorPose[segID].setOrigin(tf2::Vector3(0,0,segmentLength[segID]));
		endEffectorPose[segID].setRotation(tf2::Quaternion(0,0,0,1));

	}
	segKappa[segID] = 0.00001;
	segPhi[segID] = 0.0;
}

void KinematicsEngine::setSegmentBasePose(int segID, tf2::Vector3 basePos, tf2::Quaternion baseRot)
{
	basePose[segID].setOrigin(basePos);
	basePose[segID].setRotation(baseRot);

	for (int s=segID+1;s<numberOfSegments;s++)
	{
		basePose[s].setOrigin(basePose[s-1].getOrigin() + (tf2::Matrix3x3(basePose[s-1].getRotation())*getDiskPosition(s-1,(noOfDisks[s-1]-1))));
		basePose[s].setRotation(basePose[s-1].getRotation()*getDiskQuaternion(s-1,(noOfDisks[s-1]-1)));
	}
}

void KinematicsEngine::setSegmentShape(int segID, double kappa, double phi){
	if(kappa == 0) kappa = 0.0000001;
	segKappa[segID] = kappa;
	segPhi[segID] = phi;
	tf2::Matrix3x3 Rot;
	tf2::Quaternion qRot;
	Rot.setValue(pow(cos(phi),2) * (cos(kappa*segmentLength[segID]) - 1) + 1, sin(phi)*cos(phi)*( cos(kappa*segmentLength[segID]) - 1), -cos(phi)*sin(kappa*segmentLength[segID]),
							sin(phi)*cos(phi)*( cos(kappa*segmentLength[segID]) - 1), pow(cos(phi),2) * ( 1 - cos(kappa*segmentLength[segID]) ) + cos( kappa * segmentLength[segID] ),  -sin(phi)*sin(kappa*segmentLength[segID]),
							cos(phi)*sin(kappa*segmentLength[segID]),  sin(phi)*sin(kappa*segmentLength[segID]), cos(kappa*segmentLength[segID]));
	Rot.getRotation(qRot);
	endEffectorPose[segID].setRotation(basePose[segID].getRotation() * qRot);

	tf2::Vector3 eePosition = basePose[segID].getOrigin() + ( tf2::Matrix3x3(basePose[segID].getRotation())*tf2::Vector3(cos(phi)*( cos(kappa*segmentLength[segID]) - 1)/kappa, sin(phi)*( cos(kappa*segmentLength[segID]) - 1)/kappa, sin(kappa*segmentLength[segID])/kappa));
	endEffectorPose[segID].setOrigin(eePosition);

	for (int s = segID+1; s < numberOfSegments; s++)
	{
		basePose[s].setOrigin(endEffectorPose[s-1].getOrigin());
		basePose[s].setRotation(endEffectorPose[s-1].getRotation());
		if(s==1)
		{
			endEffectorPose[s].setOrigin(basePose[s].getOrigin() + tf2::Matrix3x3(basePose[s].getRotation())*getDiskPosition(s,(noOfDisks[s]-1)));
			endEffectorPose[s].setRotation(basePose[s].getRotation()*getDiskQuaternion(s,(noOfDisks[s]-1)));
		}
	}
}