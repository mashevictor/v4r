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
CMAKE_SOURCE_DIR = /home/victor/software/v4r/build/3rdparty/ceres/downloader

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/victor/software/v4r/build/3rdparty/ceres/downloader

# Utility rule file for ceres.

# Include the progress variables for this target.
include CMakeFiles/ceres.dir/progress.make

CMakeFiles/ceres: CMakeFiles/ceres-complete


CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-install
CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-mkdir
CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-download
CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-update
CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-patch
CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-configure
CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-build
CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-install
CMakeFiles/ceres-complete: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-ROBUST_PATCH
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'ceres'"
	/usr/bin/cmake -E make_directory /home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles
	/usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles/ceres-complete
	/usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-done

/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-install: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'ceres'"
	cd /home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build && $(MAKE) install
	cd /home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build && /usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-install

/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'ceres'"
	/usr/bin/cmake -E make_directory /home/victor/software/v4r/build/3rdparty/ceres/src
	/usr/bin/cmake -E make_directory /home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build
	/usr/bin/cmake -E make_directory /home/victor/software/v4r/build/3rdparty/ceres/install
	/usr/bin/cmake -E make_directory /home/victor/software/v4r/build/3rdparty/ceres/tmp
	/usr/bin/cmake -E make_directory /home/victor/software/v4r/build/3rdparty/ceres/stamp
	/usr/bin/cmake -E make_directory /home/victor/software/v4r/build/3rdparty/download
	/usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-mkdir

/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-download: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-urlinfo.txt
/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-download: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (download, verify and extract) for 'ceres'"
	cd /home/victor/software/v4r/build/3rdparty/ceres && /usr/bin/cmake -P /home/victor/software/v4r/build/3rdparty/ceres/stamp/download-ceres.cmake
	cd /home/victor/software/v4r/build/3rdparty/ceres && /usr/bin/cmake -P /home/victor/software/v4r/build/3rdparty/ceres/stamp/verify-ceres.cmake
	cd /home/victor/software/v4r/build/3rdparty/ceres && /usr/bin/cmake -P /home/victor/software/v4r/build/3rdparty/ceres/stamp/extract-ceres.cmake
	cd /home/victor/software/v4r/build/3rdparty/ceres && /usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-download

/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-update: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No update step for 'ceres'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-update

/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-patch: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'ceres'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-patch

/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-configure: /home/victor/software/v4r/build/3rdparty/ceres/tmp/ceres-cfgcmd.txt
/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-configure: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-update
/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-configure: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'ceres'"
	cd /home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build && /usr/bin/cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/home/victor/software/v4r/build/3rdparty/ceres/install -DGLOG_INCLUDE_DIR=/usr/include -DGLOG_LIBRARY=/usr/lib/x86_64-linux-gnu/libglog.so "-GUnix Makefiles" /home/victor/software/v4r/build/3rdparty/ceres/src
	cd /home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build && /usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-configure

/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-build: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'ceres'"
	cd /home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build && $(MAKE)
	cd /home/victor/software/v4r/build/3rdparty/ceres/downloader/ceres-prefix/src/ceres-build && /usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-build

/home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-ROBUST_PATCH: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Performing robust patch step"
	/usr/bin/cmake -DWORKING_DIR=/home/victor/software/v4r/build/3rdparty/ceres/src -DPATCHES_DIR=/home/victor/software/v4r/3rdparty/ceres -P /home/victor/software/v4r/cmake/scripts/ApplyPatches.cmake
	/usr/bin/cmake -E touch /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-ROBUST_PATCH

ceres: CMakeFiles/ceres
ceres: CMakeFiles/ceres-complete
ceres: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-install
ceres: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-mkdir
ceres: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-download
ceres: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-update
ceres: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-patch
ceres: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-configure
ceres: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-build
ceres: /home/victor/software/v4r/build/3rdparty/ceres/stamp/ceres-ROBUST_PATCH
ceres: CMakeFiles/ceres.dir/build.make

.PHONY : ceres

# Rule to build all files generated by this target.
CMakeFiles/ceres.dir/build: ceres

.PHONY : CMakeFiles/ceres.dir/build

CMakeFiles/ceres.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ceres.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ceres.dir/clean

CMakeFiles/ceres.dir/depend:
	cd /home/victor/software/v4r/build/3rdparty/ceres/downloader && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/victor/software/v4r/build/3rdparty/ceres/downloader /home/victor/software/v4r/build/3rdparty/ceres/downloader /home/victor/software/v4r/build/3rdparty/ceres/downloader /home/victor/software/v4r/build/3rdparty/ceres/downloader /home/victor/software/v4r/build/3rdparty/ceres/downloader/CMakeFiles/ceres.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ceres.dir/depend

