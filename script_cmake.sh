#!/bin/bash

#Zielpfad 
BUILDPATH=/home/pi/librealsense

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

cd $BUILDPATH
rm -r swe_project
mkdir -p swe_project
cd $BUILDPATH/swe_project

cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/PCD2ASC.cpp /$BUILDPATH/swe_project
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/Processing.cpp /$BUILDPATH/swe_project
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/ReferenceModel.cpp /$BUILDPATH/swe_project
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/CRealsenseScan.cpp /$BUILDPATH/swe_project
cp $SCRIPTPATH/example.hpp /$BUILDPATH/swe_project
cp $SCRIPTPATH/CMakelists.txt /$BUILDPATH

cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/Processing.h /$BUILDPATH/swe_project
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/ReferenceModel.h /$BUILDPATH/swe_project
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/CRealsenseScan.h /$BUILDPATH/swe_project

#CMakeLists.txt wird erstellt
echo '
# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(PCD2ASC)


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (PCD2ASC CRealsenseScan.cpp CRealsenseScan.h PCD2ASC.cpp Processing.cpp Processing.h ReferenceModel.cpp ReferenceModel.h example.hpp)
set_property(TARGET PCD2ASC PROPERTY CXX_STANDARD 11)
target_link_libraries (PCD2ASC ${PCL_LIBRARIES} ${DEPENDENCIES})
include_directories(rs-pointcloud ../examples/)

' > CMakeLists.txt

cd $BUILDPATH

cmake .. 
make

