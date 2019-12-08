// PCDtoASC.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//


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

using namespace std;


//int _tmain(int argc, _TCHAR* argv[])
int main (int argc, char  *argv[])
{
//Stefans Testcode
#pragma region Stefans Testcode

	//string sIn;
	//float line_stepsize = 3.0;


	//if (argc == 1)
	//{
	//	cout << "Input File: ";
	//	cin >> sIn;
	//}
	//else if (argc == 2)
	//{
	//	sIn = (char*)&(*argv[1]);
	//}
	//else
	//{
	//	cout << "Error in Command Line!" << endl;
	//	cout << "usage: PCDtoASC.exe input-file" << endl;
	//	return -1;
	//}

	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointNormal>);
	//fstream file;
	//string s;

	//std::cout << std::endl << "Open file \'" << sIn << "\'" << std::endl;

	//int res = pcl::io::loadPCDFile<pcl::PointNormal>(sIn.c_str(), *cloud_ptr);

	//if (res == -1)
	//{
	//	cout << "Unable to open PCD_File! Abort." << endl;
	//	return -1;
	//}

	//string sOut;
	//std::size_t found = sIn.find(".");
	//std::size_t found_last;
	//if (found!=std::string::npos)
	//{
	//	found_last = found;
	//	found = sIn.find(".", found+1);
	//	while(found!=std::string::npos)
	//	{
	//		found_last = found;
	//		found = sIn.find(".", found+1);
	//	}
	//	sOut = sIn.substr (0,found_last) + ".asc";
	//}
	//else
	//{
	//	sOut = sIn + ".asc";
	//}	
	////	pcl::io::savePCDFileBinary(sOut, cloud);
	//file.open(sOut.c_str(), ios::out);

	//file << "# 3D ASCII (X Y Z)" << std::endl;

	//for (int i=0; i < (int)cloud_ptr->points.size(); i++)
	//{
	//	//file << cloud.points[i].x <<"\t" << cloud.points[i].y << "\t" << cloud.points[i].z << std::endl;
	//	file.width(11);
	//	file << cloud_ptr->points[i].x << " ";
	//	file.width(11);
	//	file << cloud_ptr->points[i].y << " ";
	//	file.width(11);
	//	file << cloud_ptr->points[i].z << " ";
	//	file.width(11);
	//	file << cloud_ptr->points[i].normal_x << " ";
	//	file.width(11);
	//	file << cloud_ptr->points[i].normal_y << " ";
	//	file.width(10);
	//	file << cloud_ptr->points[i].normal_z << std::endl;
	//}

	//file.close();

	//std::cerr << endl << "Saved " << cloud_ptr->points.size () << " data points to " << sOut << endl;

	//return 0;
#pragma endregion


// Moved PLY Reader/Writer to "Processing" class
	Processing readWrite;
	readWrite.plyReader();
}

