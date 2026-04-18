#include "rceti_continuum/UrdfGenerator.h"

UrdfGenerator::UrdfGenerator() {
    robotURDFfile.open(ament_index_cpp::get_package_share_directory("rceti_continuum") + "/urdf/generated_robot.urdf");

    if (!robotURDFfile.is_open()) {
        std::cerr << "CRITICAL ERROR: Failed to open URDF file for writing" << std::endl;
        return;
    }

    robotURDFfile << "<?xml version=\"1.0\"?>\n";
    robotURDFfile << "<robot name=\"rceti_continuum\">\n\n";
    
    robotURDFfile << "  \n";
    robotURDFfile << "  <link name=\"continuum_base_link\">\n";
    robotURDFfile << "    <visual>\n";
    robotURDFfile << "      <geometry>\n";
    robotURDFfile << "        <cylinder length=\"0.01\" radius=\"0.005\"/>\n";
    robotURDFfile << "      </geometry>\n";
    robotURDFfile << "    </visual>\n";
    robotURDFfile << "  </link>\n\n";
}

void UrdfGenerator::createURDF(int segID, double segment_length, int segment_disks, double segment_radius, int number_of_segments) {
    if (!robotURDFfile.is_open()) return;

    double disk_length = segment_length / segment_disks;

    for (int i = 0; i < segment_disks; i++) {
        
        robotURDFfile << "  <link name=\"S" << segID << "L" << i << "\">\n";
        robotURDFfile << "    <visual>\n";
        robotURDFfile << "      <geometry>\n";
        robotURDFfile << "        <cylinder length=\"" << disk_length << "\" radius=\"" << segment_radius << "\"/>\n";
        robotURDFfile << "      </geometry>\n";
        robotURDFfile << "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n";
        robotURDFfile << "    </visual>\n";
        robotURDFfile << "  </link>\n\n";

        robotURDFfile << "  <joint name=\"joint_S" << segID << "L" << i << "\" type=\"floating\">\n";
        robotURDFfile << "    <parent link=\"continuum_base_link\"/>\n";
        robotURDFfile << "    <child link=\"S" << segID << "L" << i << "\"/>\n";
        robotURDFfile << "  </joint>\n\n";
    }

    if (segID == number_of_segments - 1) {
        robotURDFfile << "</robot>\n";
        robotURDFfile.close();
        std::cout << "[UrdfGenerator] Successfully wrote dynamic URDF file!" << std::endl;
    }
}
