#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// the point type used with the point cloud data
typedef pcl::PointXYZ pt_typ;

// display the point cloud
void show_cloud(pcl::PointCloud<pt_typ>::Ptr cloud)
{
	pcl::visualization::CloudViewer viewer("Point cloud viz");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
		std::cout << "";
	}
}

// write the filtered points to pcd file
void write_to_file(pcl::PointCloud<pt_typ>::Ptr cloud_filtered, std::string pcd_file_out)
{
	pcl::io::savePCDFileASCII (pcd_file_out.c_str(), *cloud_filtered);
}

/* 
	View the pcd file written to the file from the terminal using *pcl_viewer*
	$ pcl_viewer output.pcd
*/