#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// the point type used with the point cloud data
typedef pcl::PointXYZ pt_typ;

template <class T>
void show_cloud(pcl::PointCloud<T>::Ptr cloud)
{
	pcl::visualization::CloudViewer viewer("Point cloud viz");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
		std::cout << "";
	}
}

// display the point cloud
/*void show_cloud(pcl::PointCloud<pt_typ>::Ptr cloud)
{
	pcl::visualization::CloudViewer viewer("Point cloud viz");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
		std::cout << "";
	}
}*/

// write the filtered points to pcd file
void write_to_file(pcl::PointCloud<pt_typ>::Ptr cloud_filtered, std::string pcd_file_out)
{
	pcl::io::savePCDFileASCII (pcd_file_out.c_str(), *cloud_filtered);
}

/* 
	View the pcd file written to the file from the terminal using *pcl_viewer*
	$ pcl_viewer output.pcd
*/

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pt_typ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pt_typ> rgb(cloud);
  viewer->addPointCloud<pt_typ> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pt_typ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}