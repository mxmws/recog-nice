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
	//processing
	Processing process;
	//source_cloud = readWrite.transformationMatrix(source_cloud);

	//import
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = process.plyReader("ball_2.ply");

	source_cloud = process.transformationMatrix(source_cloud);

	//remove background
	process.removeBackground(source_cloud);
	//process.cropItembox();



	// ReferenceModel and ICP
	pcl::PointCloud<pcl::PointXYZ>::Ptr ball_1_cloud = process.plyReader("ball_1.ply");
	ReferenceModel refModel1(ball_1_cloud);
	//refModel.scoreSimilarity();

	//pcl::io::savePLYFileBinary("noObjectTransformed.ply", *source_cloud);
}

