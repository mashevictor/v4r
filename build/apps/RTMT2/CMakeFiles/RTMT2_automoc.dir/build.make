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

# Utility rule file for RTMT2_automoc.

# Include the progress variables for this target.
include apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/progress.make

apps/RTMT2/CMakeFiles/RTMT2_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/victor/software/v4r/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc and uic for target RTMT2"
	cd /home/victor/software/v4r/build/apps/RTMT2 && /usr/bin/cmake -E cmake_autogen /home/victor/software/v4r/build/apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/ Release

RTMT2_automoc: apps/RTMT2/CMakeFiles/RTMT2_automoc
RTMT2_automoc: apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/build.make

.PHONY : RTMT2_automoc

# Rule to build all files generated by this target.
apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/build: RTMT2_automoc

.PHONY : apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/build

apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/clean:
	cd /home/victor/software/v4r/build/apps/RTMT2 && $(CMAKE_COMMAND) -P CMakeFiles/RTMT2_automoc.dir/cmake_clean.cmake
.PHONY : apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/clean

apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/depend:
	cd /home/victor/software/v4r/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/victor/software/v4r /home/victor/software/v4r/apps/RTMT2 /home/victor/software/v4r/build /home/victor/software/v4r/build/apps/RTMT2 /home/victor/software/v4r/build/apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/RTMT2/CMakeFiles/RTMT2_automoc.dir/depend

