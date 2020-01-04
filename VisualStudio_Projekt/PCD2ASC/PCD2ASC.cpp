// PCDtoASC.cpp : Definiert den Einstiegspunkt f�r die Konsolenanwendung.
//
#include "ReferenceModel.h"
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
#include "Processing.h"
#include <pcl/filters/extract_indices.h>		//header to remove points with indices


using namespace std;


//int _tmain(int argc, _TCHAR* argv[])
int main ()
{
	//import
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PLYReader Reader;
	Reader.read("noObject.ply", *source_cloud);

	//processing
	Processing readWrite;
	//source_cloud = readWrite.transformationMatrix(source_cloud);
	
	readWrite.plyReader();
	readWrite.removeBackground();
	//readWrite.cropItembox();

	// ReferenceModel and ICP
	ReferenceModel refModel("ball_1", "ball_1_filtered");
	refModel.scoreSimilarity();

	//pcl::io::savePLYFileBinary("noObjectTransformed.ply", *source_cloud);
}

