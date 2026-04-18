/* Created April 2026 by CSE 2.3 */
#ifndef RCETI_CONTINUUM_KINEMATICS_ENGINE_H_
#define RCETI_CONTINUUM_KINEMATICS_ENGINE_H_

#include <vector>
#include <math.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

constexpr double PI = 3.1415926;

/**
 * @class KinematicsEngine
 * @brief Handles the core mathematical calculations for constant curvature continuum kinematics.
 *
 * This engine takes raw physical parameters (length, radius) and bending commands (curvature, direction),
 * and calculates the precise 3D position and orientation of any point along the robot's curved backbone.
 */
class KinematicsEngine {
    private:
        /** @brief The calculated 3D pose of the tip of each segment. */
        std::vector<tf2::Transform> endEffectorPose;

        /** @brief The 3D pose where each segment begins. */
		std::vector<tf2::Transform> basePose;

        /** @brief The calculated 3D poses of every individual disk making up the segments. */
		std::vector<std::vector<tf2::Transform>> segTFFrame;

        /** @brief Buffer array for delayed curvature values (used for dynamic movements). */
        std::vector<double> arrayOfKappa;

        /** @brief Buffer array for delayed direction values (used for dynamic movements). */
		std::vector<double> arrayOfPhi;

        /** @brief The physical length of each segment in meters. */
		std::vector<double> segment_lengths_;

        /** @brief The number of discrete rigid "vertebrae" used to approximate the continuous curve. */
		std::vector<int> noOfDisks;

        /** @brief Tracks the current operational mode of the segment. */
		std::vector<int> segmentMode;

        /** @brief Current curvature value (kappa) for each segment. 0 is perfectly straight. */
		std::vector<double> segKappa;

        /** @brief Current direction angle (phi) for each segment in radians. */
		std::vector<double> segPhi;
        
    public:
        /**
         * @brief Constructor. Allocates memory for the specified number of robot segments.
         * @param number_of_segments The total number of segments this engine will calculate.
         */
        KinematicsEngine(int number_of_segments);

        /**
         * @brief Initializes the physical properties of a specific continuum segment.
         * @param segID The index of the segment.
         * @param segment_length The length of the segment when straight.
         * @param segment_disks The number of discrete rigid rings to calculate along the arc.
         * @param segment_radius The radius of the segment.
         */
		void addSegment(int segID, double segment_length, int segment_disks, double segment_radius);

        /**
         * @brief Defines the starting origin point and orientation of a segment.
         * @param segID The index of the segment.
         * @param basePos The 3D translation (x, y, z) of the segment's base.
         * @param baseRot The 3D rotation (quaternion) of the segment's base.
         */
		void setSegmentBasePose(int segID, tf2::Vector3 basePos, tf2::Quaternion baseRot);

        /**
         * @brief Sets the mathematical curve for a segment, driving the kinematics update.
         * @param segID The index of the segment to bend.
         * @param kappa The curvature (1/radius of the curve).
         * @param phi The direction to bend toward (in radians).
         */
		void setSegmentShape(int segID, double kappa, double phi);

        /** @brief Returns the number of discrete disks in the specified segment. */
        int getNoOfDisks(int segID) { return noOfDisks[segID]; }

        /** @brief Returns the current curvature (kappa) of the specified segment. */
        double getKappa(int segID) { return segKappa[segID]; }

        /** @brief Returns the current bend direction (phi) of the specified segment. */
        double getPhi(int segID) { return segPhi[segID]; }

        /** @brief Returns the length of the specified segment. */
        double getSegmentLength(int segID) { return segment_lengths_[segID]; }

        /** @brief Returns the base pose (origin) of the specified segment. */
        tf2::Transform getBasePose(int segID) { return basePose[segID]; }

        /** @brief Returns the full calculated 3D frame (pose) for a specific disk on a segment. */
        tf2::Transform getDiskFrame(int segID, int diskID) { return segTFFrame[segID][diskID]; }

        /**
         * @brief Calculates and returns the specific 3D orientation of a single disk along the curve.
         * @param segID The index of the segment.
         * @param diskID The index of the specific disk.
         * @return A tf2::Quaternion representing the orientation of the disk face.
         */
        tf2::Quaternion getDiskQuaternion(int segID, int diskID);

        /**
         * @brief Calculates and returns the specific 3D position (x, y, z) of a single disk along the curve.
         * @param segID The index of the segment.
         * @param diskID The index of the specific disk.
         * @return A tf2::Vector3 representing the local coordinates of the disk.
         */
		tf2::Vector3 getDiskPosition(int segID, int diskID);
};

#endif