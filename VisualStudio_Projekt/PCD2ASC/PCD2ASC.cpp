#include "ReferenceModel.h"
#include "Processing.h"
#include <iostream>							// for std::cout
#include <pcl/io/pcd_io.h>					// header that contains the definitions for PCD I/O operations
#include <pcl/filters/extract_indices.h>	// header to remove points with indices
#include <vector>
#include <utility> 
using namespace std;

void startScanning()
{
	system("/home/pi/librealsense/build/examples/pointcloud/rs-pointcloud");
}

int main ()
{
	// Processing
	Processing process;

	process.positioning();

	string sourceCloudFile = "test1_filter15.ply";

	cout << "Press enter to scan an object" << endl;
	cin.get();
	startScanning();
	pcl::PointCloud<pcl::PointXYZ>::Ptr scannedObject = process.plyReader(sourceCloudFile);
	cout << "Done..." << endl;

	
	scannedObject = process.transformationMatrix(scannedObject);
	scannedObject = process.uptRemoveBackground(scannedObject);
	pcl::io::savePLYFileBinary("scannedObjectCut.ply", *scannedObject);
	
	cin.get();
}

