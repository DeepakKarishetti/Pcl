#include <pcl/filters/passthrough.h> // Passthrough filter
#include "pcl_tools.h"

int main(int argc, char** argv)
{
 	std::cout << "Passthrough filter tutorial!" << "\n";
 	std::cout << "Usage: ./exe_name [1]pcd_file_in \n";

 	if (argc != 2)
 	{
 		std::cerr << "Enter an input pcd file to load in the data! \n";
 		exit(0);
 	}

	// define the pcl structures 
	pcl::PointCloud<pt_typ>::Ptr cloud (new pcl::PointCloud<pt_typ>);
	pcl::PointCloud<pt_typ>::Ptr cloud_filtered_x (new pcl::PointCloud<pt_typ>);
	pcl::PointCloud<pt_typ>::Ptr cloud_filtered_y (new pcl::PointCloud<pt_typ>);
	pcl::PointCloud<pt_typ>::Ptr cloud_filtered_z (new pcl::PointCloud<pt_typ>);

	// load in the point cloud data
	std::string pcd_file_in = argv[1]; // "lab_cloud.pcd";
	if (pcl::io::loadPCDFile<pt_typ> (pcd_file_in.c_str(), *cloud) == -1)
	{
		PCL_ERROR (" Cant read in the file \n");
		return -1;	
	}
	std::cout << "Cloud size: " << cloud->points.size() << "\n";

	// view the cloud
	// show_cloud(cloud);

	// create the filtering objects
	pcl::PassThrough<pt_typ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0.0, 1.0);
	pass.setFilterLimitsNegative(true); // cuts out the points between the given limits;
	pass.filter(*cloud_filtered_x);

	write_to_file(cloud_filtered_x,"filt_check_x.pcd");

	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.setFilterLimitsNegative(true); // cuts out the points between the given limits;
	pass.filter(*cloud_filtered_z);

	std::cout << "Cloud size after Passthrough filtering: " << cloud_filtered_z->points.size() << "\n";
	// show filtered cloud
	show_cloud(cloud_filtered_z);

	// save it to a pcd file and can view it using pcl_viewer
	write_to_file(cloud_filtered_z, "filt_lab_filtered.pcd");

	return 0;
}
