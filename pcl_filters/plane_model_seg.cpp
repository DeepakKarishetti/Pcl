#include "pcl_tools.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int main(int argc, char** argv)
{
	std::cout << "Tutorial on plane model segmentation!" << "\n";
	std::cout << "Usage: ./exe_name [1]pcd_file_in [2]out_path \n";

	if (argc != 3)
	{
		std::cerr << "Check usage for valid input arguments! \n";
		exit(0);
	}

	// define the pcl objects for holding the cloud data
	pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);

	pcl::PointCloud<pt_typ>::Ptr pcl_cloud_filtered (new pcl::PointCloud<pt_typ>);
	pcl::PointCloud<pt_typ>::Ptr cloud_out (new pcl::PointCloud<pt_typ>);
	pcl::PointCloud<pt_typ>::Ptr cloud_f (new pcl::PointCloud<pt_typ>);

	std::string pcd_file_in = argv[1]; // "table_scene_lms400.pcd"; // "lab_cloud.pcd";
	pcl::PCDReader reader;
	reader.read(pcd_file_in, *cloud_in);

	std::cout << "\n Cloud from the pcd file: " << *cloud_in << "\n";

	// Downsample the given data to reduce the computation time for segmentation
	pcl::VoxelGrid<pcl::PCLPointCloud2> vox_filter;
	vox_filter.setInputCloud(cloud_in);
	vox_filter.setLeafSize(0.01f, 0.01f, 0.01f);
	vox_filter.filter(*cloud_filtered);

	//std::cout << "\n Could after doensampling the points: " << *cloud_filtered << "\n";

	// converting to the templated version of pointcloud
	pcl::fromPCLPointCloud2 (*cloud_filtered, *pcl_cloud_filtered);
	// pointcloud after filtering
	std::cout << "\n Pointcloud after filtering: " << pcl_cloud_filtered->height * pcl_cloud_filtered->width << "\n";

	// write the downsampled version to file
	std::string pcd_out = "seg_downsampled_.pcd";
	pcl::PCDWriter writer;
	writer.write<pt_typ>(pcd_out.c_str(), *pcl_cloud_filtered, false);

	// Parametric segmentation:
	// RANSAC is used as a robust estimator
	// Model coefficients, Point indices obj
	pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
	// call the segmentation obj
	pcl::SACSegmentation<pt_typ> seg;
	seg.setOptimizeCoefficients(true); // optional --> ?
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	// create a filtering obj
	pcl::ExtractIndices<pt_typ> extract;
	int counter = 1;
	std::string out_path = argv[2];
	// segment as long as 30% of the cloud points still exist
	while (pcl_cloud_filtered->points.size() > (0.3 * pcl_cloud_filtered->points.size()))
	{
		// segment the largest planar component from the cloud
		seg.setInputCloud(pcl_cloud_filtered);
		seg.segment(*inliers, *coeffs);
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the input dataset!" << "\n";
			break;
		}

		// extract the inliers
		extract.setInputCloud(pcl_cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_out);

		std::cerr << "PointCloud representing the planar component created: " << cloud_out->width * cloud_out->height << "\n";

		// write to file
		std::stringstream ss;
		ss << out_path << "cloud_" << counter << ".pcd";
		writer.write<pt_typ> (ss.str(), *cloud_out, false);

		// filtering objext
		extract.setNegative(true);
		extract.filter(*cloud_f);
		pcl_cloud_filtered.swap(cloud_f);
		counter++;
	}

	//show_cloud()

	return 0;
}
