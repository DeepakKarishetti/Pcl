#include <iostream>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <glob.h>
#include <vector>

typedef pcl::PointXYZ pt_type;

std::vector<std::string> globVector(const std::string& path)
{
    glob_t glob_result;
    glob(path.c_str(),GLOB_TILDE,NULL,&glob_result);
    std::vector<std::string> files;
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        files.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return files;
}

int main(int argc, char** argv)
{
	std::cout << "PCD file to csv file!\n";

	std::vector<std::string> files = globVector("pcd_csv/pcd_scans/*");
	
	for (int i=0; i<files.size(); i++)
	{
		pcl::PointCloud<pt_type>::Ptr cloud (new pcl::PointCloud<pt_type>);
		std::string pcd_file_in = files[i]; 
		std::cout << "Reading the file " << pcd_file_in << "\n";

		if (pcl::io::loadPCDFile<pt_type> (pcd_file_in.c_str(), *cloud) == -1)
		{
			PCL_ERROR (" Cant read in the file \n");
			return -1;	
		}

	    std::filebuf fb;
		std::stringstream ss;
		ss << "/home/karishetti/M3Robotics/AlphaAR_Fusion/build-qt_proj-Desktop-Debug/scans/pts_output_" << i << ".csv";

	  	fb.open(ss.str(),std::ios::out);
	  	std::ostream os(&fb);

		for (int i=0; i<cloud->points.size(); i++)
		{
			os << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << std::endl;
		}

		fb.close();
	}

	return 0;
}