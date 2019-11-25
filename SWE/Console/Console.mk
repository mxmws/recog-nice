##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Console
ConfigurationName      :=Debug
WorkspacePath          :="/home/pi/Desktop/SWE Projekt/team5/SWE"
ProjectPath            :="/home/pi/Desktop/SWE Projekt/team5/SWE/Console"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=
Date                   :=11/25/19
CodeLitePath           :=/home/pi/.codelite
LinkerName             :=/usr/bin/g++
SharedObjectLinkerName :=/usr/bin/g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="Console.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch)/usr/include/pcl-1.9 $(IncludeSwitch)/usr/include/pcl-1.9/pcl $(IncludeSwitch)/usr/include/pcl-1.9/pcl/2d $(IncludeSwitch)/usr/include/pcl-1.9/pcl/apps $(IncludeSwitch)/usr/include/pcl-1.9/pcl/common $(IncludeSwitch)/usr/include/pcl-1.9/pcl/compression $(IncludeSwitch)/usr/include/pcl-1.9/pcl/console $(IncludeSwitch)/usr/include/pcl-1.9/pcl/features $(IncludeSwitch)/usr/include/pcl-1.9/pcl/filters $(IncludeSwitch)/usr/include/pcl-1.9/pcl/geometry $(IncludeSwitch)/usr/include/pcl-1.9/pcl/impl $(IncludeSwitch)/usr/include/pcl-1.9/pcl/in_hand_scanner $(IncludeSwitch)/usr/include/pcl-1.9/pcl/io $(IncludeSwitch)/usr/include/pcl-1.9/pcl/kdtree $(IncludeSwitch)/usr/include/pcl-1.9/pcl/keypoints $(IncludeSwitch)/usr/include/pcl-1.9/pcl/ml $(IncludeSwitch)/usr/include/pcl-1.9/pcl/octree $(IncludeSwitch)/usr/include/pcl-1.9/pcl/outofcore $(IncludeSwitch)/usr/include/pcl-1.9/pcl/people $(IncludeSwitch)/usr/include/pcl-1.9/pcl/range_image $(IncludeSwitch)/usr/include/pcl-1.9/pcl/recognition $(IncludeSwitch)/usr/include/pcl-1.9/pcl/registration $(IncludeSwitch)/usr/include/pcl-1.9/pcl/sample_consensus $(IncludeSwitch)/usr/include/pcl-1.9/pcl/search $(IncludeSwitch)/usr/include/pcl-1.9/pcl/segmentation $(IncludeSwitch)/usr/include/pcl-1.9/pcl/stereo $(IncludeSwitch)/usr/include/pcl-1.9/pcl/surface $(IncludeSwitch)/usr/include/pcl-1.9/pcl/tracking $(IncludeSwitch)/usr/include/pcl-1.9/pcl/visualization 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)pcl_apps $(LibrarySwitch)pcl_common $(LibrarySwitch)pcl_features $(LibrarySwitch)pcl_filters $(LibrarySwitch)pcl_io $(LibrarySwitch)pcl_io_ply $(LibrarySwitch)pcl_kdtree $(LibrarySwitch)pcl_keypoints $(LibrarySwitch)pcl_ml $(LibrarySwitch)pcl_octree $(LibrarySwitch)pcl_outofcore $(LibrarySwitch)pcl_people $(LibrarySwitch)pcl_recognition $(LibrarySwitch)pcl_registration $(LibrarySwitch)pcl_sample_consensus $(LibrarySwitch)pcl_search $(LibrarySwitch)pcl_segmentation $(LibrarySwitch)pcl_stereo $(LibrarySwitch)pcl_surface $(LibrarySwitch)pcl_tracking $(LibrarySwitch)pcl_visualization 
ArLibs                 :=  "libpcl_apps.so" "libpcl_common.so" "libpcl_features.so" "libpcl_filters.so" "libpcl_io.so" "libpcl_io_ply.so" "libpcl_kdtree.so" "libpcl_keypoints.so" "libpcl_ml.so" "libpcl_octree.so" "libpcl_outofcore.so" "libpcl_people.so" "libpcl_recognition.so" "libpcl_registration.so" "libpcl_sample_consensus.so" "libpcl_search.so" "libpcl_segmentation.so" "libpcl_stereo.so" "libpcl_surface.so" "libpcl_tracking.so" "libpcl_visualization.so" 
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)/usr/lib/arm-linux-gnueabihf 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/ar rcu
CXX      := /usr/bin/g++
CC       := /usr/bin/gcc
CXXFLAGS :=  -g -O0 -Wall $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/as


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/ReferenceModel.cpp$(ObjectSuffix) $(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IntermediateDirectory)/Console.cpp$(ObjectSuffix) $(IntermediateDirectory)/Processing.cpp$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@test -d ./Debug || $(MakeDirCommand) ./Debug


$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/ReferenceModel.cpp$(ObjectSuffix): ReferenceModel.cpp $(IntermediateDirectory)/ReferenceModel.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/Desktop/SWE Projekt/team5/SWE/Console/ReferenceModel.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/ReferenceModel.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/ReferenceModel.cpp$(DependSuffix): ReferenceModel.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/ReferenceModel.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/ReferenceModel.cpp$(DependSuffix) -MM ReferenceModel.cpp

$(IntermediateDirectory)/ReferenceModel.cpp$(PreprocessSuffix): ReferenceModel.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/ReferenceModel.cpp$(PreprocessSuffix) ReferenceModel.cpp

$(IntermediateDirectory)/main.cpp$(ObjectSuffix): main.cpp $(IntermediateDirectory)/main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/Desktop/SWE Projekt/team5/SWE/Console/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main.cpp$(DependSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/main.cpp$(DependSuffix) -MM main.cpp

$(IntermediateDirectory)/main.cpp$(PreprocessSuffix): main.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main.cpp$(PreprocessSuffix) main.cpp

$(IntermediateDirectory)/Console.cpp$(ObjectSuffix): Console.cpp $(IntermediateDirectory)/Console.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/Desktop/SWE Projekt/team5/SWE/Console/Console.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/Console.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Console.cpp$(DependSuffix): Console.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Console.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/Console.cpp$(DependSuffix) -MM Console.cpp

$(IntermediateDirectory)/Console.cpp$(PreprocessSuffix): Console.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Console.cpp$(PreprocessSuffix) Console.cpp

$(IntermediateDirectory)/Processing.cpp$(ObjectSuffix): Processing.cpp $(IntermediateDirectory)/Processing.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/Desktop/SWE Projekt/team5/SWE/Console/Processing.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/Processing.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Processing.cpp$(DependSuffix): Processing.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Processing.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/Processing.cpp$(DependSuffix) -MM Processing.cpp

$(IntermediateDirectory)/Processing.cpp$(PreprocessSuffix): Processing.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Processing.cpp$(PreprocessSuffix) Processing.cpp


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


