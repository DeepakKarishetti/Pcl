# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/karishetti/pcl_stuff/pcl_filters

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/karishetti/pcl_stuff/pcl_filters/build

# Include any dependencies generated for this target.
include CMakeFiles/proj_inliers.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/proj_inliers.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/proj_inliers.dir/flags.make

CMakeFiles/proj_inliers.dir/project_inliers.cpp.o: CMakeFiles/proj_inliers.dir/flags.make
CMakeFiles/proj_inliers.dir/project_inliers.cpp.o: ../project_inliers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/karishetti/pcl_stuff/pcl_filters/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/proj_inliers.dir/project_inliers.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/proj_inliers.dir/project_inliers.cpp.o -c /home/karishetti/pcl_stuff/pcl_filters/project_inliers.cpp

CMakeFiles/proj_inliers.dir/project_inliers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/proj_inliers.dir/project_inliers.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/karishetti/pcl_stuff/pcl_filters/project_inliers.cpp > CMakeFiles/proj_inliers.dir/project_inliers.cpp.i

CMakeFiles/proj_inliers.dir/project_inliers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/proj_inliers.dir/project_inliers.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/karishetti/pcl_stuff/pcl_filters/project_inliers.cpp -o CMakeFiles/proj_inliers.dir/project_inliers.cpp.s

CMakeFiles/proj_inliers.dir/project_inliers.cpp.o.requires:

.PHONY : CMakeFiles/proj_inliers.dir/project_inliers.cpp.o.requires

CMakeFiles/proj_inliers.dir/project_inliers.cpp.o.provides: CMakeFiles/proj_inliers.dir/project_inliers.cpp.o.requires
	$(MAKE) -f CMakeFiles/proj_inliers.dir/build.make CMakeFiles/proj_inliers.dir/project_inliers.cpp.o.provides.build
.PHONY : CMakeFiles/proj_inliers.dir/project_inliers.cpp.o.provides

CMakeFiles/proj_inliers.dir/project_inliers.cpp.o.provides.build: CMakeFiles/proj_inliers.dir/project_inliers.cpp.o


# Object files for target proj_inliers
proj_inliers_OBJECTS = \
"CMakeFiles/proj_inliers.dir/project_inliers.cpp.o"

# External object files for target proj_inliers
proj_inliers_EXTERNAL_OBJECTS =

proj_inliers: CMakeFiles/proj_inliers.dir/project_inliers.cpp.o
proj_inliers: CMakeFiles/proj_inliers.dir/build.make
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_system.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_thread.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_regex.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libpthread.so
proj_inliers: /usr/local/lib/libpcl_common.so
proj_inliers: /usr/local/lib/libpcl_ml.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
proj_inliers: /usr/local/lib/libpcl_kdtree.so
proj_inliers: /usr/local/lib/libpcl_octree.so
proj_inliers: /usr/local/lib/libpcl_search.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libqhull.so
proj_inliers: /usr/local/lib/libpcl_surface.so
proj_inliers: /usr/lib/libOpenNI.so
proj_inliers: /usr/lib/libOpenNI2.so
proj_inliers: /usr/local/lib/libpcl_io.so
proj_inliers: /usr/local/lib/libpcl_sample_consensus.so
proj_inliers: /usr/local/lib/libpcl_filters.so
proj_inliers: /usr/local/lib/libpcl_features.so
proj_inliers: /usr/local/lib/libpcl_registration.so
proj_inliers: /usr/local/lib/libpcl_recognition.so
proj_inliers: /usr/local/lib/libpcl_segmentation.so
proj_inliers: /usr/local/lib/libpcl_visualization.so
proj_inliers: /usr/local/lib/libpcl_keypoints.so
proj_inliers: /usr/local/lib/libpcl_tracking.so
proj_inliers: /usr/local/lib/libpcl_stereo.so
proj_inliers: /usr/local/lib/libpcl_outofcore.so
proj_inliers: /usr/local/lib/libpcl_people.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_system.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_thread.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libboost_regex.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libpthread.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libqhull.so
proj_inliers: /usr/lib/libOpenNI.so
proj_inliers: /usr/lib/libOpenNI2.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
proj_inliers: /usr/local/lib/libvtkIOSQL-7.1.so.1
proj_inliers: /usr/local/lib/libvtksqlite-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOParallelXML-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingStatistics-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingMorphological-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersSelection-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersPoints-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOTecplotTable-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingStencil-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOParallel-7.1.so.1
proj_inliers: /usr/local/lib/libvtkjsoncpp-7.1.so.1
proj_inliers: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
proj_inliers: /usr/local/lib/libvtkViewsInfovis-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingLabel-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOExodus-7.1.so.1
proj_inliers: /usr/local/lib/libvtkInteractionImage-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOAMR-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOLSDyna-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersTexture-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOMINC-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersHyperTree-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersFlowPaths-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOInfovis-7.1.so.1
proj_inliers: /usr/local/lib/libvtkGeovisCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOMovie-7.1.so.1
proj_inliers: /usr/local/lib/libvtkoggtheora-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersSMP-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOExport-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOPLY-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersParallelImaging-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOVideo-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersVerdict-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOImport-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersProgrammable-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOEnSight-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersGeneric-7.1.so.1
proj_inliers: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingImage-7.1.so.1
proj_inliers: /usr/local/lib/libpcl_common.so
proj_inliers: /usr/local/lib/libpcl_ml.so
proj_inliers: /usr/local/lib/libpcl_kdtree.so
proj_inliers: /usr/local/lib/libpcl_octree.so
proj_inliers: /usr/local/lib/libpcl_search.so
proj_inliers: /usr/local/lib/libpcl_surface.so
proj_inliers: /usr/local/lib/libpcl_io.so
proj_inliers: /usr/local/lib/libpcl_sample_consensus.so
proj_inliers: /usr/local/lib/libpcl_filters.so
proj_inliers: /usr/local/lib/libpcl_features.so
proj_inliers: /usr/local/lib/libpcl_registration.so
proj_inliers: /usr/local/lib/libpcl_recognition.so
proj_inliers: /usr/local/lib/libpcl_segmentation.so
proj_inliers: /usr/local/lib/libpcl_visualization.so
proj_inliers: /usr/local/lib/libpcl_keypoints.so
proj_inliers: /usr/local/lib/libpcl_tracking.so
proj_inliers: /usr/local/lib/libpcl_stereo.so
proj_inliers: /usr/local/lib/libpcl_outofcore.so
proj_inliers: /usr/local/lib/libpcl_people.so
proj_inliers: /usr/local/lib/libvtkImagingMath-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIONetCDF-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOGeometry-7.1.so.1
proj_inliers: /usr/local/lib/libvtkChartsCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkexoIIc-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersAMR-7.1.so.1
proj_inliers: /usr/local/lib/libvtkNetCDF_cxx-7.1.so.1
proj_inliers: /usr/local/lib/libvtkNetCDF-7.1.so.1
proj_inliers: /usr/local/lib/libvtkhdf5_hl-7.1.so.1
proj_inliers: /usr/local/lib/libvtkhdf5-7.1.so.1
proj_inliers: /usr/local/lib/libvtklibxml2-7.1.so.1
proj_inliers: /usr/local/lib/libvtkInfovisLayout-7.1.so.1
proj_inliers: /usr/local/lib/libvtkInfovisCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkViewsCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingColor-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOXML-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
proj_inliers: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
proj_inliers: /usr/local/lib/libvtkproj4-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
proj_inliers: /usr/local/lib/libvtkfreetype-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-7.1.so.1
proj_inliers: /usr/local/lib/libvtkgl2ps-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersImaging-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingSources-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersParallel-7.1.so.1
proj_inliers: /usr/local/lib/libvtkParallelCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingFourier-7.1.so.1
proj_inliers: /usr/local/lib/libvtkalglib-7.1.so.1
proj_inliers: /usr/local/lib/libvtkverdict-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOImage-7.1.so.1
proj_inliers: /usr/local/lib/libvtkDICOMParser-7.1.so.1
proj_inliers: /usr/local/lib/libvtkmetaio-7.1.so.1
proj_inliers: /usr/local/lib/libvtkpng-7.1.so.1
proj_inliers: /usr/local/lib/libvtktiff-7.1.so.1
proj_inliers: /usr/local/lib/libvtkjpeg-7.1.so.1
proj_inliers: /usr/lib/x86_64-linux-gnu/libm.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libSM.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libICE.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libX11.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libXext.so
proj_inliers: /usr/lib/x86_64-linux-gnu/libXt.so
proj_inliers: /usr/local/lib/libvtkglew-7.1.so.1
proj_inliers: /usr/local/lib/libvtkDomainsChemistry-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
proj_inliers: /usr/local/lib/libvtkexpat-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOLegacy-7.1.so.1
proj_inliers: /usr/local/lib/libvtkIOCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkzlib-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
proj_inliers: /usr/local/lib/libvtkRenderingCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonColor-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersSources-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
proj_inliers: /usr/local/lib/libvtkFiltersCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkImagingCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonMisc-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonMath-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonSystem-7.1.so.1
proj_inliers: /usr/local/lib/libvtkCommonCore-7.1.so.1
proj_inliers: /usr/local/lib/libvtksys-7.1.so.1
proj_inliers: CMakeFiles/proj_inliers.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/karishetti/pcl_stuff/pcl_filters/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable proj_inliers"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/proj_inliers.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/proj_inliers.dir/build: proj_inliers

.PHONY : CMakeFiles/proj_inliers.dir/build

CMakeFiles/proj_inliers.dir/requires: CMakeFiles/proj_inliers.dir/project_inliers.cpp.o.requires

.PHONY : CMakeFiles/proj_inliers.dir/requires

CMakeFiles/proj_inliers.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/proj_inliers.dir/cmake_clean.cmake
.PHONY : CMakeFiles/proj_inliers.dir/clean

CMakeFiles/proj_inliers.dir/depend:
	cd /home/karishetti/pcl_stuff/pcl_filters/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/karishetti/pcl_stuff/pcl_filters /home/karishetti/pcl_stuff/pcl_filters /home/karishetti/pcl_stuff/pcl_filters/build /home/karishetti/pcl_stuff/pcl_filters/build /home/karishetti/pcl_stuff/pcl_filters/build/CMakeFiles/proj_inliers.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/proj_inliers.dir/depend

