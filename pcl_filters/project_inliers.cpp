#include "pcl_tools.h"
#include<pcl/filters/project_inliers.h>

int main(int argc, char** argv)
{
	std::cout << "Tutorial on projecting points using a parametric model!" << "\n";
	std::cout << "Usage: ./exe_name [1]pcd_file_in \n";

	if (argc != 2)
	{
		std::cerr << "Check the usage for valid input arguments! \n";
		exit(0);
	}

	// pcl objects to get the pointcloud
	pcl::PointCloud<pt_typ>::Ptr cloud (new pcl::PointCloud<pt_typ>);
	pcl::PointCloud<pt_typ>::Ptr cloud_out (new pcl::PointCloud<pt_typ>);

	std::string pcd_file_in = argv[1]; // "lab_cloud.pcd";
	pcl::PCDReader reader;
	reader.read<pt_typ>(pcd_file_in, *cloud);

	std::cout << "Cloud from the input file: " << *cloud << "\n";

	// creating a set of planar co-effs [0,0,1]; for the planar eq ax+by+cz+d=0; 
	// Which is nothing but the X-Y plane
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// project inliers
	pcl::ProjectInliers <pt_typ> proj_in;
	proj_in.setModelType(pcl::SACMODEL_PLANE);
	proj_in.setInputCloud(cloud);
	proj_in.setModelCoefficients(coefficients);
	proj_in.filter(*cloud_out);

	std::cout << "Cloud after filtering: " << *cloud_out << "\n";

	// write to file
	std::string pcd_file_out = "filt_proj_inliers_out.pcd";
	write_to_file(cloud_out, pcd_file_out);

	show_cloud(cloud_out);

	return 0;
}