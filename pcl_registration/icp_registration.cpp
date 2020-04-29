#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointT_N;
typedef pcl::PointCloud<PointT> p_c;

//! global params
pcl::visualization::PCLVisualizer *viz;
// viewports
int vp_1, vp_2;

//! struct to hold the name of file and its pointcloud
struct pcd_couple
{
	p_c::Ptr cloud;
	std::string f_name;

	pcd_couple() : cloud(new p_c) {};
};

//! LOAD data from the pcd files
void load_pcd(int argc, char** argv, std::vector<pcd_couple, Eigen::aligned_allocator<pcd_couple> >& pcd_data)
{
	std::string extension = ".pcd";
	//! taking the files from the arguments
	for (int i=1; i<argc; i++)
	{
		std::string fname = argv[i];
		std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);

		// check if argument is a pcd file
		if (fname.compare(fname.size()-extension.size(), extension.size(), extension) == 0)
		{
			// load the cloud and save it into the vector
			pcd_couple m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile(argv[i], *m.cloud);

			pcd_data.push_back(m);
		}
	}
}

void viz_clouds_r(const p_c::Ptr target_cloud, const p_c::Ptr source_cloud)
{
	viz->removePointCloud("vp_1_target");
	viz->removePointCloud("vp_1_source");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> target_h (target_cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> source_h (source_cloud, 255, 0, 0);

	viz->addPointCloud(target_cloud, target_h, "vp_1_target", vp_1);
	viz->addPointCloud(source_cloud, source_h, "vp_1_source", vp_1);

	PCL_INFO("\nPress q to begin the iterations! \n");
	viz->spin();
}

//! Downsampling the input cloud using voxel grid filter
void downsample(const p_c::Ptr& cloud)
{
	pcl::VoxelGrid<PointT> v_filter;
	v_filter.setLeafSize(0.1f, 0.1f, 0.1f);
	v_filter.setInputCloud(cloud);
	v_filter.filter(*cloud);
}

//! dispaly source and target on the right viz
void viz_clouds_l(const pcl::PointCloud<PointT_N>::Ptr target_pts_normals, const pcl::PointCloud<PointT_N>::Ptr source_pts_normals)
{
	viz->removePointCloud("source");
	viz->removePointCloud("target");

	pcl::visualization::PointCloudColorHandlerCustom<PointT_N> target_h (target_pts_normals, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT_N> source_h (source_pts_normals, 255, 0, 0);

	viz->addPointCloud(target_pts_normals, target_h, "target", vp_2);
	viz->addPointCloud(source_pts_normals, source_h, "source", vp_2);

	viz->spinOnce();
}

//! aligns the pointcloud pair and give us the resultant cloud and the resultant transform
void icp_align(const p_c::Ptr source_cloud, const p_c::Ptr target_cloud, p_c::Ptr& res_cloud, Eigen::Matrix4f& T_final, bool down_sample = false)
{
	//! downsampling the clouds
	if (down_sample)
	{
		downsample(source_cloud);
		downsample(target_cloud);
	}

	//! compute the surface normals and curvature
	pcl::PointCloud<PointT_N>::Ptr source_pts_normals (new pcl::PointCloud<PointT_N>);
	pcl::PointCloud<PointT_N>::Ptr target_pts_normals (new pcl::PointCloud<PointT_N>);

	pcl::NormalEstimation<PointT, PointT_N> N_estimation;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	N_estimation.setSearchMethod(tree);
	N_estimation.setKSearch(30);

	N_estimation.setInputCloud(source_cloud);
	N_estimation.compute(*source_pts_normals);
	pcl::copyPointCloud(*source_cloud, *source_pts_normals);

	N_estimation.setInputCloud(target_cloud);
	N_estimation.compute(*target_pts_normals);
	pcl::copyPointCloud(*target_cloud, *target_pts_normals);

	//! ICP Aligning!
	pcl::IterativeClosestPointNonLinear<PointT_N, PointT_N> reg;
	reg.setTransformationEpsilon(1e-6);
	//! --> settting the maximum distance between the correspondences (source <--> target)
	reg.setMaxCorrespondenceDistance(0.5);
	
	reg.setInputSource(source_pts_normals);
	reg.setInputTarget(target_pts_normals);
	//reg.setPointRepresentation (std::make_shared<const point_rep> (pt_rep));

	// Run the optimization in a loop 
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f prev, target_to_source;
	pcl::PointCloud<PointT_N>::Ptr reg_result = source_pts_normals;

	reg.setMaximumIterations(2);

	for (int i=0; i<25; i++)
	{
		PCL_INFO ("Iteration no: %d\n", i+1);
		source_pts_normals = reg_result;

		// Estimate
		reg.setInputSource(source_pts_normals);
		reg.align(*reg_result);

		// accumulating the transforms between each iterations
		Ti = reg.getFinalTransformation() * Ti;

		// if the difference between this one and the previous transform is less
		// than the previous iteration, refine the process by reducting the
		// maximal correspondence distance
		if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
		{
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		}
		prev = reg.getLastIncrementalTransformation();

		// viz
		viz_clouds_l(target_pts_normals, source_pts_normals);
	}
	// transformation from the target to the source
	target_to_source = Ti.inverse();

	// tranform back to the source frame
	pcl::transformPointCloud(*target_cloud, *res_cloud, target_to_source);

	viz->removePointCloud("source");
	viz->removePointCloud("target");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> target_cloud_h (res_cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_h (source_cloud, 255, 0, 0);

	viz->addPointCloud(res_cloud, target_cloud_h, "target", vp_2);
	viz->addPointCloud(source_cloud, source_cloud_h, "source", vp_2);

	PCL_INFO("Press q to continue cloud registration!\n");
	viz->spin();

	viz->removePointCloud("source");
	viz->removePointCloud("target");

	// add the source cloud to the transformed 
	*res_cloud += *source_cloud;

	T_final = target_to_source;
}

//! write cloud to file
void write_to_file(p_c::Ptr cloud_filtered, std::string pcd_file_out)
{
	pcl::io::savePCDFileASCII (pcd_file_out.c_str(), *cloud_filtered);
}

int main(int argc, char** argv)
{
	std::cout << "Pairwise pointcloud registration using ICP!\n";
	std::cout << "Usage: ./exe_name [pcd_files_input] \nExample: ./icp_reg pcd_files/* \n";

	//! reading in the pointcloud data from the files
	std::vector<pcd_couple, Eigen::aligned_allocator<pcd_couple> > pcd_data;
	load_pcd(argc, argv, pcd_data);

	if (pcd_data.empty())
	{
		PCL_ERROR("No pcd files have been given as input! \n");
		PCL_ERROR("Usage: ./exe_name pcd_file_names... \n");
		return -1;
	}
	PCL_INFO("Number of pcd files loaded: %d \n", (int)pcd_data.size());

	//! visualizer
	viz = new pcl::visualization::PCLVisualizer (argc, argv, "Pair-wise incremental scan matching!");
	viz->createViewPort(0.0, 0.0, 0.5, 1.0, vp_1);
	viz->createViewPort(0.5, 0.0, 1.0, 1.0, vp_2);

	p_c::Ptr result (new p_c);
	p_c::Ptr source, target;
	Eigen::Matrix4f T_global = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f T_pair;

	for (size_t i=1; i<pcd_data.size(); i++)
	{
		source = pcd_data[i-1].cloud;
		target = pcd_data[i].cloud;

		// visualization of the input clouds
		viz_clouds_r(source, target);

		p_c::Ptr temp (new p_c);
		PCL_INFO ("Aligning with %s (%d) with %s (%d) \n", pcd_data[i-1].f_name.c_str(), source->points.size(), pcd_data[i].f_name.c_str(), target->points.size());
		icp_align(source, target, temp, T_pair, true);
		
		std::string file_out = "aligned_cloud.pcd"; 
		write_to_file(temp, file_out);
		std::cout << "The aligned cloud saved to file as: " << file_out << "\n";
	}

	/*
	if (argc != 3)
	{
		std::cerr << "Usage: ./exe_name [1]pcd_1 [2]pcd_2 \n";
		return -1;
	}

	pcl::PointCloud<PointT>::Ptr cloud_1 (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_2 (new pcl::PointCloud<PointT>);

	// load in the pointcloud data from file
	std::string file_in_1 = argv[1];
	std::string file_in_2 = argv[2];
	if (pcl::io::loadpcd_coupleFile<PointT> (file_in_1.c_str(), *cloud_1) == -1 or pcl::io::loadpcd_coupleFile<PointT> (file_in_2.c_str(), *cloud_2) == -1)
	{
		PCL_ERROR("Can't read both the cloud files! \n");
		return -1;
	}
	std::cout << "Cloud 1 size: " << cloud_1->size() << "\n";
	std::cout << "Cloud 2 size: " << cloud_2->size() << "\n\n";

	// downsampling the clouds
	downsample(cloud_1);
	downsample(cloud_2);
	std::cout << "Size after downsampling the clouds:\n";
	std::cout << "Cloud 1 size: " << cloud_1->size() << "\n";
	std::cout << "Cloud 2 size: " << cloud_2->size() << "\n\n";

	// performing simple rigid transform on the clouds
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(cloud_1);
	icp.setInputTarget(cloud_2);

	pcl::PointCloud<PointT> result;
	icp.align(result);

	std::cout << "The matching has converged: " << icp.hasConverged() << " with a score: " << icp.getFitnessScore() << "\n\n";
	std::cout << "Final transformation matrix is: \n" << icp.getFinalTransformation() << "\n";
	*/

	return 0;
}