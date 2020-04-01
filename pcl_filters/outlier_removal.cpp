#include "pcl_tools.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

// RADIUS OUTLIER REMOVAL FILTER and CONDITIONAL REMOVAL FILTER
int main(int argc, char** argv)
{
	std::cout << "Tutorial for radial outliers removal filter!" << "\n";
	std::cout << "Usage: ./exe_name [1]pcd_file [2]filter_arg: \n";

	if (argc != 3)
	{
		std::cerr << "(1) Specify the pcd file to load in! \n";
		std::cerr << "(2) Specify the argument to be used: -r or -c " << "\n";
		exit(0);
	}

	pcl::PointCloud<pt_typ>::Ptr cloud (new pcl::PointCloud<pt_typ>);
	pcl::PointCloud<pt_typ>::Ptr cloud_filtered (new pcl::PointCloud<pt_typ>);

	// read in the cloud data
	std::string pcd_file_in = argv[1]; // = "lab_cloud.pcd";

	pcl::PCDReader reader;
	reader.read<pt_typ>(pcd_file_in, *cloud);

	std::cout << "Cloud before filtering: " << *cloud << "\n";

	if (strcmp(argv[2], "-r") == 0)
	{
		pcl::RadiusOutlierRemoval<pt_typ> r_filter;
		r_filter.setInputCloud(cloud);
		r_filter.setRadiusSearch(1.5);
		r_filter.setMinNeighborsInRadius(2);
		r_filter.filter(*cloud_filtered); // filter
	}
	else if (strcmp(argv[2], "-c") == 0)
	{
		// filter conditions to check between the range 0.0 and 1.5
		pcl::ConditionAnd<pt_typ>::Ptr range_condition (new pcl::ConditionAnd<pt_typ> ());
		range_condition->addComparison (pcl::FieldComparison<pt_typ>::ConstPtr (new pcl::FieldComparison<pt_typ> ("z", pcl::ComparisonOps::GT, 0.0)));
		range_condition->addComparison (pcl::FieldComparison<pt_typ>::ConstPtr (new pcl::FieldComparison<pt_typ> ("z", pcl::ComparisonOps::LT, 1.5)));
		// filter implementation
		pcl::ConditionalRemoval<pt_typ> cond_rem_filter;
		cond_rem_filter.setCondition(range_condition);
		cond_rem_filter.setInputCloud(cloud);
		cond_rem_filter.setKeepOrganized(true);
		cond_rem_filter.filter(*cloud_filtered); // filter
	}
	else
	{
		std::cerr << "Enter valid filter arguments to be used! \n";
	}

	show_cloud(cloud_filtered);

	return 0;
}