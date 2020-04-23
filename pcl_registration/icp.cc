//! Source: http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ pt_typ;

int main(int argc, char** argv)
{
	std::cout << "Iterative closest point usage demo!" << "\n";

	// cloud definitions
	pcl::PointCloud<pt_typ>::Ptr cloud_in (new pcl::PointCloud<pt_typ> (5,1));
	pcl::PointCloud<pt_typ>::Ptr cloud_out (new pcl::PointCloud<pt_typ>);

	// fill in the data
	for (auto& i : *cloud_in)
	{
		i.x = 1024 * rand() / (RAND_MAX + 1.0f);
		i.y = 1024 * rand() / (RAND_MAX + 1.0f);
		i.z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cout << "\nThe points created in one cloud: \n";
	for (auto& i : *cloud_in)
	{
		std::cout << i.x << " " << i.y << " " << i.z << "\n";
	}

	*cloud_out = *cloud_in;

	for (auto& i : *cloud_out)
	{
		i.x += 0.7f;
	}
	std::cout << "\n";

	std::cout << "The points created in the other cloud: \n";
	for (auto &i : *cloud_out)
	{
		std::cout << i << "\n";
	}

	// perform simple rigid transform on the pointcloud
	pcl::IterativeClosestPoint<pt_typ, pt_typ> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);

	pcl::PointCloud<pt_typ> result;
	icp.align(result);

	std::cout << "\nThe matching has converged " << icp.hasConverged() 
			  << " with a score of: " << icp.getFitnessScore() << "\n\n";
	std::cout << "The final transformation matrix is: \n" << icp.getFinalTransformation() << "\n";

	return 0;
}