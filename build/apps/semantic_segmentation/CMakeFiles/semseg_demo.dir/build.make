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
CMAKE_SOURCE_DIR = /home/victor/software/v4r

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/victor/software/v4r/build

# Include any dependencies generated for this target.
include apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/depend.make

# Include the progress variables for this target.
include apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/progress.make

# Include the compile flags for this target's objects.
include apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/flags.make

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o: apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/flags.make
apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o: ../apps/semantic_segmentation/semseg_demo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/victor/software/v4r/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o"
	cd /home/victor/software/v4r/build/apps/semantic_segmentation && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o -c /home/victor/software/v4r/apps/semantic_segmentation/semseg_demo.cpp

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/semseg_demo.dir/semseg_demo.cpp.i"
	cd /home/victor/software/v4r/build/apps/semantic_segmentation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/victor/software/v4r/apps/semantic_segmentation/semseg_demo.cpp > CMakeFiles/semseg_demo.dir/semseg_demo.cpp.i

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/semseg_demo.dir/semseg_demo.cpp.s"
	cd /home/victor/software/v4r/build/apps/semantic_segmentation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/victor/software/v4r/apps/semantic_segmentation/semseg_demo.cpp -o CMakeFiles/semseg_demo.dir/semseg_demo.cpp.s

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o.requires:

.PHONY : apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o.requires

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o.provides: apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o.requires
	$(MAKE) -f apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/build.make apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o.provides.build
.PHONY : apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o.provides

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o.provides.build: apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o


# Object files for target semseg_demo
semseg_demo_OBJECTS = \
"CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o"

# External object files for target semseg_demo
semseg_demo_EXTERNAL_OBJECTS =

bin/semseg_demo: apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o
bin/semseg_demo: apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/build.make
bin/semseg_demo: lib/libv4r_semantic_segmentation.so.2.0.6
bin/semseg_demo: lib/libv4r_core.so.2.0.6
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
bin/semseg_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
bin/semseg_demo: /usr/local/lib/libpcl_cuda_segmentation.so
bin/semseg_demo: /usr/local/lib/libpcl_cuda_features.so
bin/semseg_demo: /usr/local/lib/libpcl_cuda_sample_consensus.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_regex.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libpthread.so
bin/semseg_demo: /usr/local/lib/libpcl_common.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
bin/semseg_demo: /usr/local/lib/libpcl_kdtree.so
bin/semseg_demo: /usr/local/lib/libpcl_octree.so
bin/semseg_demo: /usr/local/lib/libpcl_search.so
bin/semseg_demo: /usr/local/lib/libpcl_sample_consensus.so
bin/semseg_demo: /usr/local/lib/libpcl_filters.so
bin/semseg_demo: /usr/lib/libOpenNI.so
bin/semseg_demo: /usr/lib/libOpenNI2.so
bin/semseg_demo: /usr/local/lib/libvtkIOImport-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOMovie-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkoggtheora-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersSMP-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingMath-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOAMR-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersAMR-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingImage-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkInteractionImage-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersParallelImaging-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOExport-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkgl2ps-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkGUISupportQtSQL-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOSQL-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtksqlite-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOVideo-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersPoints-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersHyperTree-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersVerdict-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkverdict-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingMorphological-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOParallelXML-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingQt-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersTexture-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOTecplotTable-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOParallel-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersParallel-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkParallelCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkjsoncpp-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIONetCDF-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOGeometry-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOPLY-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkViewsQt-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkViewsInfovis-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersImaging-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkChartsCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingLabel-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkGUISupportQt-7.1.so.1
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1
bin/semseg_demo: /usr/local/lib/libvtkIOExodus-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkexoIIc-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOLSDyna-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersSelection-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersGeneric-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingStencil-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkGeovisCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkInfovisLayout-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkViewsCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkfreetype-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingColor-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkproj4-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOMINC-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingSources-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkNetCDF_cxx-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkNetCDF-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkhdf5_hl-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkhdf5-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOEnSight-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersProgrammable-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOInfovis-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkInfovisCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingFourier-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkalglib-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOXML-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtklibxml2-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOImage-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkDICOMParser-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkmetaio-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkpng-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtktiff-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkjpeg-7.1.so.1
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libm.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libSM.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libICE.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libX11.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libXext.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libXt.so
bin/semseg_demo: /usr/local/lib/libvtkglew-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkDomainsChemistry-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkRenderingCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonColor-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkexpat-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOLegacy-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingStatistics-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkImagingCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersFlowPaths-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersSources-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkFiltersCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkIOCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonMisc-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonMath-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonSystem-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkCommonCore-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtksys-7.1.so.1
bin/semseg_demo: /usr/local/lib/libvtkzlib-7.1.so.1
bin/semseg_demo: /usr/local/lib/libpcl_io.so
bin/semseg_demo: /usr/local/lib/libpcl_visualization.so
bin/semseg_demo: /usr/local/lib/libpcl_features.so
bin/semseg_demo: /usr/local/lib/libpcl_ml.so
bin/semseg_demo: /usr/local/lib/libpcl_segmentation.so
bin/semseg_demo: /usr/local/lib/libpcl_gpu_containers.so
bin/semseg_demo: /usr/local/lib/libpcl_gpu_utils.so
bin/semseg_demo: /usr/local/lib/libpcl_gpu_octree.so
bin/semseg_demo: /usr/local/lib/libpcl_gpu_segmentation.so
bin/semseg_demo: /usr/local/lib/libpcl_gpu_kinfu.so
bin/semseg_demo: /usr/local/lib/libpcl_gpu_features.so
bin/semseg_demo: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libqhull.so
bin/semseg_demo: /usr/local/lib/libpcl_surface.so
bin/semseg_demo: /usr/local/lib/libpcl_keypoints.so
bin/semseg_demo: /usr/local/lib/libpcl_outofcore.so
bin/semseg_demo: /usr/local/lib/libpcl_stereo.so
bin/semseg_demo: /usr/local/lib/libpcl_registration.so
bin/semseg_demo: /usr/local/lib/libpcl_tracking.so
bin/semseg_demo: /usr/local/lib/libpcl_recognition.so
bin/semseg_demo: /usr/local/lib/libpcl_apps.so
bin/semseg_demo: /usr/local/lib/libpcl_3d_rec_framework.so
bin/semseg_demo: /usr/local/lib/libpcl_people.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libGL.so
bin/semseg_demo: /usr/lib/x86_64-linux-gnu/libGLEW.so
bin/semseg_demo: /usr/local/lib/libpcl_simulation.so
bin/semseg_demo: apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/victor/software/v4r/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/semseg_demo"
	cd /home/victor/software/v4r/build/apps/semantic_segmentation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/semseg_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/build: bin/semseg_demo

.PHONY : apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/build

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/requires: apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/semseg_demo.cpp.o.requires

.PHONY : apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/requires

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/clean:
	cd /home/victor/software/v4r/build/apps/semantic_segmentation && $(CMAKE_COMMAND) -P CMakeFiles/semseg_demo.dir/cmake_clean.cmake
.PHONY : apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/clean

apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/depend:
	cd /home/victor/software/v4r/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/victor/software/v4r /home/victor/software/v4r/apps/semantic_segmentation /home/victor/software/v4r/build /home/victor/software/v4r/build/apps/semantic_segmentation /home/victor/software/v4r/build/apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/semantic_segmentation/CMakeFiles/semseg_demo.dir/depend

