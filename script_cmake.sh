#!/bin/bash

#Zielpfad hier angeben
BUILDPATH=/home/pi/"SPEICHERORT DES REPOSITORIES ANGEBEN"/team5/Examples

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

cd $BUILDPATH
mkdir -p final_exe
cd $BUILDPATH/final_exe
rm -r build
mkdir -p build


#CMakeLists.txt wird erstellt
echo '
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCD2ASC)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES
    /home/pi/Desktop/repository/team5/VisualStudio_Projekt/PCD2ASC/*.h
    /home/pi/Desktop/repository/team5/VisualStudio_Projekt/PCD2ASC/*.cpp
)


add_executable (PCD2ASC ${SOURCES})
target_link_libraries (PCD2ASC ${PCL_LIBRARIES})
' > CMakeLists.txt

cd $BUILDPATH/final_exe/build

cmake ..
make


