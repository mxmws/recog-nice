#include "Processing.h"
#include <stdio.h>
#include <tchar.h>


#include <stdlib.h>
#include <iostream>             // for std::cout
#include <string>
#include <fstream>
#include <algorithm>
#include <pcl/io/pcd_io.h>      // header that contains the definitions for PCD I/O operations
#include <iterator>
#include <math.h>
#include <pcl/io/ply_io.h>
using namespace std;

// Loading PLY file into PointCloud object and saves it as PLY-Kopie
void Processing::plyReader()
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	pcl::PLYReader Reader;

	Reader.read("DeoPLY.ply", *cloud_ptr);

	string writePath = "DeoKOPIE.ply";

	pcl::io::savePLYFileBinary(writePath, *cloud_ptr);
}

void Processing::removeBackground()
{

}

