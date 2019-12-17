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
#include <pcl/filters/extract_indices.h>		//header to remove points with indices


using namespace std;


//int _tmain(int argc, _TCHAR* argv[])
int main (int argc, char  *argv[])
{

	Processing readWrite;
	readWrite.plyReader();
	readWrite.removeBackground();
	readWrite.compareToReferences();

}

