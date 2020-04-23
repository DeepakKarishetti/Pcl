#include "pcl_tools.h"
#include <pcl/features/normal_3d.h>

/*	The neighbouring points of a query point can be used to estimate the local feature estimation that
	captures the geometry of the undelying sampled surface around that query point.
*/

int main(int argc, char** argv)
{
	std::cout << "Introduction tutorial on 3d features estimation methods in PCL!" << "\n"; 
	std::cout << "Usage: ./<exe> <pcd_file> \n";
	if (argc != 2)
	{
		std::cerr << "Invalid arguments! \n";
		exit(0);
	}
	// http://pointclouds.org/documentation/tutorials/how_features_work.php#how-3d-features-work

	// ***** Estimate a set of surface normals for all the points in the input dataset *****

	// read in from a pcd file
	pcl::PointCloud<pt_typ>::Ptr cloud (new pcl::PointCloud<pt_typ>);

	std::string pcd_file_in = argv[1];
	pcl::PCDReader reader;
	reader.read<pt_typ>(pcd_file_in.c_str(), *cloud);

	std::cout << "Data from the input cloud: " << *cloud << "\n";

	// create a normal estimation class and pass the input dataset to it
	pcl::NormalEstimation<pt_typ, pcl::Normal> nor_est;
	nor_est.setInputCloud(cloud);

	// create an emoty kd tree and give it to the above object
	// tree will be filled inside the object based on the input cloud data
	pcl::search::KdTree<pt_typ>::Ptr tree (new pcl::search::KdTree<pt_typ> ());
	nor_est.setSearchMethod(tree);

	// output cloud
	pcl::PointCloud<pcl::Normal>::Ptr output_cloud (new pcl::PointCloud<pcl::Normal>);

	// get all the neighbours present in a sphere radius of 3 cm
	nor_est.setRadiusSearch(0.03);

	// compute the features
	nor_est.compute(*output_cloud);

	std::cout << "Output cloud: " << *output_cloud << "\n";
 

	return 0;
}


