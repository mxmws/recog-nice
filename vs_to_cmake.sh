#!/bin/bash

#Zielordner hier angeben
BUILDPATH=/home/pi

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

cd $BUILDPATH
mkdir -p ply_reader_test
cd $BUILDPATH/ply_reader_test
mkdir -p build

#cpp wird kopiert und mit dem raspi kompatibel gemacht
cp -R $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/PCD2ASC.cpp /$BUILDPATH/ply_reader_test
sed -i '/#include <tchar.h>/d' PCD2ASC.cpp
COPYPATH=$SCRIPTPATH:~/RefModelle/DeoPLY.ply
sed -i 's|C:/Users/Minh/Desktop/repository/team5/RefModelle/DeoPLY.ply|$COPYPATH|g' PCD2ASC.cpp
sed -i 's|C:/Users/Minh/Desktop/repository/team5/RefModelle/DeoKOPIE.ply|/home/pi/DeoKOPIE.ply|g' PCD2ASC.cpp


#CMakeLists.txt wird erstellt
echo '
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCD2ASC)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PCD2ASC PCD2ASC.cpp)
target_link_libraries (PCD2ASC ${PCL_LIBRARIES})
' > CMakeLists.txt

cd $BUILDPATH/ply_reader_test/build

#cmake ..
#make