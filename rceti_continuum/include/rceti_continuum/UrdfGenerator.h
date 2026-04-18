/* Created April 2026 by CSE 2.3 */
#ifndef RCETI_CONTINUUM_URDF_GENERATOR_H_
#define RCETI_CONTINUUM_URDF_GENERATOR_H_

#include <fstream>
#include <iostream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

/**
 * @class UrdfGenerator
 * @brief Dynamically generates the URDF (Unified Robot Description Format) XML file.
 *
 * This engine automatically writes the XML tags needed to spawn the correct number of rigid disks 
 * in RViz, giving the KinematicsEngine physical models to manipulate.
 */
class UrdfGenerator {
    private:
        /** @brief The output file stream used to write the dynamic URDF/XACRO file to the disk. */
        std::ofstream robotURDFfile;
        
    public:
        /**
         * @brief Constructor. 
         * @details Handles opening the file stream in the correct ROS 2 package directory 
         * and writing the standard URDF XML headers.
         */
        UrdfGenerator();

        /**
         * @brief Generates the URDF XML blocks for a specific continuum segment.
         * @details Writes a series of `<link>` and `<joint>` tags representing 
         * every discrete disk in the segment. If the current segment is the final one, 
         * it also handles writing the closing `</robot>` XML tags to finish the file.
         * @param segID The index of the segment being generated.
         * @param segment_length The physical length of the segment.
         * @param segment_disks The number of discrete rigid disks that make up this segment.
         * @param segment_radius The radius of the individual disks.
         * @param number_of_segments The total number of segments in the robot (used to determine when to close the XML file).
         */
        void createURDF(int segID, double segment_length, int segment_disks, double segment_radius, int number_of_segments);
};

#endif