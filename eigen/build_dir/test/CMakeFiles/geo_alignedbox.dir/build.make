# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/chenjie/桌面/eigen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chenjie/桌面/eigen/build_dir

# Utility rule file for geo_alignedbox.

# Include the progress variables for this target.
include test/CMakeFiles/geo_alignedbox.dir/progress.make

test/CMakeFiles/geo_alignedbox:

geo_alignedbox: test/CMakeFiles/geo_alignedbox
geo_alignedbox: test/CMakeFiles/geo_alignedbox.dir/build.make
.PHONY : geo_alignedbox

# Rule to build all files generated by this target.
test/CMakeFiles/geo_alignedbox.dir/build: geo_alignedbox
.PHONY : test/CMakeFiles/geo_alignedbox.dir/build

test/CMakeFiles/geo_alignedbox.dir/clean:
	cd /home/chenjie/桌面/eigen/build_dir/test && $(CMAKE_COMMAND) -P CMakeFiles/geo_alignedbox.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/geo_alignedbox.dir/clean

test/CMakeFiles/geo_alignedbox.dir/depend:
	cd /home/chenjie/桌面/eigen/build_dir && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chenjie/桌面/eigen /home/chenjie/桌面/eigen/test /home/chenjie/桌面/eigen/build_dir /home/chenjie/桌面/eigen/build_dir/test /home/chenjie/桌面/eigen/build_dir/test/CMakeFiles/geo_alignedbox.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/geo_alignedbox.dir/depend

