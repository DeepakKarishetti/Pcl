#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ pt_type;

// Params:
float ang_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}


int main(int argc, char** argv)
{
	std::cout << "NARF keypoints from a range image!\n";

	// read in a pcd cloud from file
	pcl::PointCloud<pt_type>::Ptr cloud_ptr (new pcl::PointCloud<pt_type>);
	pcl::PointCloud<pt_type>& cloud = *cloud_ptr;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity());

	std::string pcd_file_in = argv[1]; // "lab_cloud.pcd";
	if (pcl::io::loadPCDFile<pt_type> (pcd_file_in.c_str(), cloud) == -1)
	{
		PCL_ERROR (" Can't read in the file \n");
		return -1;	
	}
	//std::cout << "Cloud size: " << cloud->points.size() << "\n";

	scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (cloud.sensor_origin_[0], 
						cloud.sensor_origin_[1],cloud.sensor_origin_[3])) * 
						Eigen::Affine3f (cloud.sensor_orientation_);

	std::string far_ranges_filename = pcl::getFilenameWithoutExtension (pcd_file_in) + "_far_ranges.pcd";
	if (pcl::io::loadPCDFile (pcd_file_in.c_str(), far_ranges) == -1)
	{
		std::cout << "The far ranges file does not exist!\n";
	}

	//! Create range image from pointcloud
	float noise_level = 0.0;
	float min_range = 0.0;
	int border = 1;

	pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
	pcl::RangeImage &range_image = *range_image_ptr;
	range_image.createFromPointCloud (cloud, ang_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
				scene_sensor_pose, coordinate_frame, noise_level, min_range, border);

	range_image.integrateFarRanges(far_ranges);
	if (setUnseenToMaxRange)
	{
		range_image.setUnseenToMaxRange();
	}

	
	//! visualization
	pcl::visualization::PCLVisualizer viewer("Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_img_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_img_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	/*
	viewer.addCoordinateSystem(1.0f, "global");
	pcl::visualization::PointCloudColorHandlerCustom<pt_type> pt_cld_color_handler(cloud_ptr, 150, 150, 150);
	viewer.addPointCloud(cloud_ptr, pt_cld_color_handler, "Original cloud");
	*/
	viewer.initCameraParameters();

	//setViewerPose(viewer, range_image.getTransformationToWorldSystem());

	//! show the range image
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	//! Extracting the NARF keypoints
	pcl::RangeImageBorderExtractor ran_img_border;
  	pcl::NarfKeypoint narf_keypt_det (&ran_img_border);
  	narf_keypt_det.setRangeImage(&range_image);
  	narf_keypt_det.getParameters().support_size = support_size;
  	narf_keypt_det.getParameters().add_points_on_straight_edges = true;
  	narf_keypt_det.getParameters().distance_for_additional_points = 0.5;

  	pcl::PointCloud<int> keypoint_indices;
  	narf_keypt_det.compute(keypoint_indices);
  	std::cout << "Number of keypoints found: " << keypoint_indices.size() << "\n";

  	//! show the keypoints in 3D viewer
  	pcl::PointCloud<pt_type>::Ptr keypoints_ptr (new pcl::PointCloud<pt_type>);
	pcl::PointCloud<pt_type>& keypoints = *keypoints_ptr;
	keypoints.points.resize(keypoint_indices.size());
	for (std::size_t i=0; i<keypoint_indices.points.size(); i++)
	{
		keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
	}
	pcl::visualization::PointCloudColorHandlerCustom<pt_type> keypoints_color_handler (keypoints_ptr,0, 255, 0);
	viewer.addPointCloud<pt_type> (keypoints_ptr, keypoints_color_handler, "Keypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "Keypoints");

	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
	}


	return 0;
}