#include "rceti_continuum/UrdfGenerator.h"

UrdfGenerator::UrdfGenerator() {}

void UrdfGenerator::createURDF(int segID, double segLength, int n_disks, double radius, int totalSegments)
{
    // Define a scaling factor
    double scale_factor = .1; // Example: Scale down by 50%

    // Get the path to the URDF file
    std::string path = ament_index_cpp::get_package_share_directory("rceti_continuum");
    path = path + "/urdf/continuum_macro.xacro";
	

    if (segID == 0)
    { // If the first time to create the robot, delete the previous file
        remove(path.c_str());

        robotURDFfile.open(path.c_str(), std::fstream::app);
        robotURDFfile << "<?xml version=\"1.1\"?>" << std::endl;
        robotURDFfile << "<robot xmlns:xacro=\"http://ros.org/wiki/xacro\" name=\"rceti_continuum\">" << std::endl;
		robotURDFfile << "<xacro:macro name=\"rceti_continuum\">" << std::endl;
        robotURDFfile << "<link name=\"continuum_base_link\"/>" << std::endl;
		robotURDFfile << "<origin xyz=\"1.0 2.0 0.5\" rpy=\"0 0 0\"/>" << std::endl; // Set the position and orientation
        robotURDFfile << "<material name=\"white\">" << std::endl;
        robotURDFfile << "<color rgba=\"0 1 0 1\"/>" << std::endl;
        robotURDFfile << "</material>" << std::endl;
    }
    else
    {
        robotURDFfile.open(path.c_str(), std::fstream::app);
    }

    robotURDFfile << std::endl;
    for (int disk = 0; disk < n_disks; disk++)
    {
        // Scale the position of the disk
        double scaled_position = scale_factor * (disk / (n_disks - 1.0)) * segLength;

        robotURDFfile << "<link name=\"S" << segID << "L" << disk << "\">" << std::endl;
        robotURDFfile << "<visual>" << std::endl;
        robotURDFfile << "<geometry>" << std::endl;

        if (segID == 0 && disk == 0)
        {
            // Scale the size of the base box
            robotURDFfile << "<box size=\"" << scale_factor * 1 << " " << scale_factor * 1 << " " << scale_factor * 0.05 << "\"/>" << std::endl;

        }
        else
        {
            // Scale the size of the cylinder
            robotURDFfile << "<cylinder length=\"" << scale_factor * 0.1 << "\" radius=\"" << scale_factor * radius << "\"/>" << std::endl;
        }

        // Scale the position of the origin
        robotURDFfile << "<origin rpy=\"0 0 0\" xyz=\"0 0 " << scaled_position << "\"/>" << std::endl;
        robotURDFfile << "</geometry>" << std::endl;

        if (segID == 0 && disk == 0)
        {
            robotURDFfile << "<material name=\"white\"/>" << std::endl;
        }

        robotURDFfile << "</visual>" << std::endl;
        robotURDFfile << "</link>" << std::endl;
        robotURDFfile << std::endl;

        // Scale the joint
        robotURDFfile << "<joint name=\"S" << segID << "J" << disk << "\" type=\"floating\">" << std::endl;
        robotURDFfile << "<parent link=\"continuum_base_link\"/>" << std::endl;
        robotURDFfile << "<child link=\"S" << segID << "L" << disk << "\"/>" << std::endl;
        robotURDFfile << "</joint>" << std::endl;
        robotURDFfile << std::endl;
    }

    if (segID == (totalSegments - 1))
    {
		robotURDFfile << "</xacro:macro>" << std::endl;
        robotURDFfile << "</robot>" << std::endl; // Add closing tag
    }

    robotURDFfile.close();
}
