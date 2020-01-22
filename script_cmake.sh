#!/bin/bash

#Zielpfad hier angeben
BUILDPATH=/home/pi

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

cd $BUILDPATH
rm -r final_exe
mkdir -p final_exe
cd $BUILDPATH/final_exe
rm -r build
mkdir -p build

cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/PCD2ASC.cpp /$BUILDPATH/final_exe
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/Processing.cpp /$BUILDPATH/final_exe
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/ReferenceModel.cpp /$BUILDPATH/final_exe
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/CRealsenseScan.cpp /$BUILDPATH/final_exe

cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/Processing.h /$BUILDPATH/final_exe
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/ReferenceModel.h /$BUILDPATH/final_exe
cp $SCRIPTPATH/VisualStudio_Projekt/PCD2ASC/CRealsenseScan.h /$BUILDPATH/final_exe

#CMakeLists.txt wird erstellt
echo '
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCD2ASC)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (PCD2ASC PCD2ASC.cpp Processing.cpp ReferenceModel.cpp Processing.h ReferenceModel.h CRealsenseScan.cpp CRealsenseScan.h)
target_link_libraries (PCD2ASC ${PCL_LIBRARIES})
' > CMakeLists.txt

cd $BUILDPATH/final_exe/build


cmake ..
make
