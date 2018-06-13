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
include apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/depend.make

# Include the progress variables for this target.
include apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/progress.make

# Include the compile flags for this target's objects.
include apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/flags.make

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o: apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/flags.make
apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o: ../apps/AttentionSegmentation/segmentIncremental.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/victor/software/v4r/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o"
	cd /home/victor/software/v4r/build/apps/AttentionSegmentation && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o -c /home/victor/software/v4r/apps/AttentionSegmentation/segmentIncremental.cpp

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.i"
	cd /home/victor/software/v4r/build/apps/AttentionSegmentation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/victor/software/v4r/apps/AttentionSegmentation/segmentIncremental.cpp > CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.i

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.s"
	cd /home/victor/software/v4r/build/apps/AttentionSegmentation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/victor/software/v4r/apps/AttentionSegmentation/segmentIncremental.cpp -o CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.s

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o.requires:

.PHONY : apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o.requires

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o.provides: apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o.requires
	$(MAKE) -f apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/build.make apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o.provides.build
.PHONY : apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o.provides

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o.provides.build: apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o


# Object files for target segmentIncremental
segmentIncremental_OBJECTS = \
"CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o"

# External object files for target segmentIncremental
segmentIncremental_EXTERNAL_OBJECTS =

bin/segmentIncremental: apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o
bin/segmentIncremental: apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/build.make
bin/segmentIncremental: lib/libv4r_attention_segmentation.so.2.0.6
bin/segmentIncremental: lib/libv4r_common.so.2.0.6
bin/segmentIncremental: lib/libv4r_io.so.2.0.6
bin/segmentIncremental: lib/libv4r_core.so.2.0.6
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libglog.so
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
bin/segmentIncremental: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
bin/segmentIncremental: 3rdparty/pcl_1_8/install/lib/libpcl_1_8.a
bin/segmentIncremental: /usr/local/lib/libpcl_cuda_segmentation.so
bin/segmentIncremental: /usr/local/lib/libpcl_cuda_features.so
bin/segmentIncremental: /usr/local/lib/libpcl_cuda_sample_consensus.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_regex.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libpthread.so
bin/segmentIncremental: /usr/local/lib/libpcl_common.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
bin/segmentIncremental: /usr/local/lib/libpcl_kdtree.so
bin/segmentIncremental: /usr/local/lib/libpcl_octree.so
bin/segmentIncremental: /usr/local/lib/libpcl_search.so
bin/segmentIncremental: /usr/local/lib/libpcl_sample_consensus.so
bin/segmentIncremental: /usr/local/lib/libpcl_filters.so
bin/segmentIncremental: /usr/lib/libOpenNI.so
bin/segmentIncremental: /usr/lib/libOpenNI2.so
bin/segmentIncremental: /usr/local/lib/libvtkIOImport-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOMovie-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkoggtheora-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersSMP-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingMath-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOAMR-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersAMR-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingImage-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkInteractionImage-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersParallelImaging-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOExport-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkgl2ps-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkGUISupportQtSQL-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOSQL-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtksqlite-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOVideo-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersPoints-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersHyperTree-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersVerdict-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkverdict-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingMorphological-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOParallelXML-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingQt-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersTexture-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOTecplotTable-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOParallel-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersParallel-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkParallelCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkjsoncpp-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIONetCDF-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOGeometry-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOPLY-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkViewsQt-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkViewsInfovis-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersImaging-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkChartsCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingLabel-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkGUISupportQt-7.1.so.1
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1
bin/segmentIncremental: /usr/local/lib/libvtkIOExodus-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkexoIIc-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOLSDyna-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersSelection-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersGeneric-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingStencil-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkGeovisCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkInfovisLayout-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkViewsCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkfreetype-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingColor-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkproj4-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOMINC-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingSources-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkNetCDF_cxx-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkNetCDF-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkhdf5_hl-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkhdf5-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOEnSight-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersProgrammable-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOInfovis-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkInfovisCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingFourier-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkalglib-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOXML-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtklibxml2-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOImage-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkDICOMParser-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkmetaio-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkpng-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtktiff-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkjpeg-7.1.so.1
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libm.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libSM.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libICE.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libX11.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libXext.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libXt.so
bin/segmentIncremental: /usr/local/lib/libvtkglew-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkDomainsChemistry-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkRenderingCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonColor-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkexpat-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOLegacy-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingStatistics-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkImagingCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersFlowPaths-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersSources-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkFiltersCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkIOCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonMisc-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonMath-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonSystem-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkCommonCore-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtksys-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libvtkzlib-7.1.so.1
bin/segmentIncremental: /usr/local/lib/libpcl_io.so
bin/segmentIncremental: /usr/local/lib/libpcl_visualization.so
bin/segmentIncremental: /usr/local/lib/libpcl_features.so
bin/segmentIncremental: /usr/local/lib/libpcl_ml.so
bin/segmentIncremental: /usr/local/lib/libpcl_segmentation.so
bin/segmentIncremental: /usr/local/lib/libpcl_gpu_containers.so
bin/segmentIncremental: /usr/local/lib/libpcl_gpu_utils.so
bin/segmentIncremental: /usr/local/lib/libpcl_gpu_octree.so
bin/segmentIncremental: /usr/local/lib/libpcl_gpu_segmentation.so
bin/segmentIncremental: /usr/local/lib/libpcl_gpu_kinfu.so
bin/segmentIncremental: /usr/local/lib/libpcl_gpu_features.so
bin/segmentIncremental: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libqhull.so
bin/segmentIncremental: /usr/local/lib/libpcl_surface.so
bin/segmentIncremental: /usr/local/lib/libpcl_keypoints.so
bin/segmentIncremental: /usr/local/lib/libpcl_outofcore.so
bin/segmentIncremental: /usr/local/lib/libpcl_stereo.so
bin/segmentIncremental: /usr/local/lib/libpcl_registration.so
bin/segmentIncremental: /usr/local/lib/libpcl_tracking.so
bin/segmentIncremental: /usr/local/lib/libpcl_recognition.so
bin/segmentIncremental: /usr/local/lib/libpcl_apps.so
bin/segmentIncremental: /usr/local/lib/libpcl_3d_rec_framework.so
bin/segmentIncremental: /usr/local/lib/libpcl_people.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libGL.so
bin/segmentIncremental: /usr/lib/x86_64-linux-gnu/libGLEW.so
bin/segmentIncremental: /usr/local/lib/libpcl_simulation.so
bin/segmentIncremental: 3rdparty/opennurbs/install/lib/libopennurbs.a
bin/segmentIncremental: apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/victor/software/v4r/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/segmentIncremental"
	cd /home/victor/software/v4r/build/apps/AttentionSegmentation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segmentIncremental.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/build: bin/segmentIncremental

.PHONY : apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/build

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/requires: apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/segmentIncremental.cpp.o.requires

.PHONY : apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/requires

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/clean:
	cd /home/victor/software/v4r/build/apps/AttentionSegmentation && $(CMAKE_COMMAND) -P CMakeFiles/segmentIncremental.dir/cmake_clean.cmake
.PHONY : apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/clean

apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/depend:
	cd /home/victor/software/v4r/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/victor/software/v4r /home/victor/software/v4r/apps/AttentionSegmentation /home/victor/software/v4r/build /home/victor/software/v4r/build/apps/AttentionSegmentation /home/victor/software/v4r/build/apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/AttentionSegmentation/CMakeFiles/segmentIncremental.dir/depend

