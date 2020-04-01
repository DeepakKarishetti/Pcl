#include "pcl_tools.h"
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
	std::cout << "\n Tutorial on statistical outliers removal!" << "\n";
	std::cout << "Usage: ./exe_name [1]pcd_file_in \n";

	if (argc != 2)
	{
		std::cerr << "Check the usage for valid input arguments! \n";
		exit(0);
	}

	// define pcl objects
	pcl::PointCloud<pt_typ>::Ptr cloud (new pcl::PointCloud<pt_typ>);
	pcl::PointCloud<pt_typ>::Ptr filtered_cloud (new pcl::PointCloud<pt_typ>);

	// PCD reader - read in the pointcloud
	std::string pcd_file_in = argv[1]; // "lab_cloud.pcd";
	pcl::PCDReader reader;
	reader.read<pt_typ> (pcd_file_in.c_str(), *cloud);

	std::cout << "Cloud before filtering: \n" << *cloud << "\n";

	// filtering objects
	pcl::StatisticalOutlierRemoval<pt_typ> stat_rem;
	stat_rem.setInputCloud(cloud);
	stat_rem.setMeanK(100); // check with 100 neighbouring points
	stat_rem.setStddevMulThresh(1.0); // std dev = 0.3
	stat_rem.filter(*filtered_cloud);

	std::cout << "Cloud after filtering: \n" << *filtered_cloud << "\n";

	// write the filtered cloud to file
	std::string file_out_filt = "filt_filtered_cloud_out.pcd";
	pcl::PCDWriter writer;
	writer.write<pt_typ>(file_out_filt.c_str(), *filtered_cloud, false);

	// see the outliers that are filtered
	stat_rem.setNegative(true);
	stat_rem.filter(*filtered_cloud);

	std::string file_outliers = "filt_stat_outliers.pcd";
	writer.write<pt_typ>(file_outliers.c_str(), *filtered_cloud);

	return 0;
}