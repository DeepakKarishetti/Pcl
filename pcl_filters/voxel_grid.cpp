#include "pcl_tools.h"
#include <pcl/filters/voxel_grid.h> // VoxelGrid filter

int main(int argc, char** argv)
{
	std::cout << "Voxel grid tutorial!" << "\n";
	std::cout << "Usage: ./exe_name [1]pcd_file_in \n";

	if (argc != 2)
	{
		std::cerr << "Check usage for valid input arguments! \n";
		exit(0);
	}

	// cloud to get the points from the pcd file
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr voxel_cloud_filtered (new pcl::PCLPointCloud2 ());

	// load in the cloud data
	std::string pcd_file_in = argv[1]; // "lab_cloud.pcd";
	pcl::PCDReader reader;
	reader.read(pcd_file_in, *cloud);

	std::cout << "Points from the pcd file: " << cloud->width * cloud->height << "\n";

	// create the filtering objects
	pcl::VoxelGrid<pcl::PCLPointCloud2> filt;
	filt.setInputCloud(cloud);
	filt.setLeafSize(0.5f, 0.5f, 0.5f); // leaf size 50 cm
	filt.filter(*voxel_cloud_filtered);

	std::cout << "Number of points after filtering: " << voxel_cloud_filtered->width * voxel_cloud_filtered->height << "\n";

	// write to pcd file
	std::string pcd_file_out = "filt_voxelgrid_filtered.pcd";
	pcl::PCDWriter writer;
	writer.write(pcd_file_out.c_str(), *voxel_cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

	return 0;
}