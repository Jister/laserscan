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

# Include any dependencies generated for this target.
include test/CMakeFiles/linearstructure_1.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/linearstructure_1.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/linearstructure_1.dir/flags.make

test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o: test/CMakeFiles/linearstructure_1.dir/flags.make
test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o: ../test/linearstructure.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chenjie/桌面/eigen/build_dir/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o"
	cd /home/chenjie/桌面/eigen/build_dir/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o -c /home/chenjie/桌面/eigen/test/linearstructure.cpp

test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linearstructure_1.dir/linearstructure.cpp.i"
	cd /home/chenjie/桌面/eigen/build_dir/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chenjie/桌面/eigen/test/linearstructure.cpp > CMakeFiles/linearstructure_1.dir/linearstructure.cpp.i

test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linearstructure_1.dir/linearstructure.cpp.s"
	cd /home/chenjie/桌面/eigen/build_dir/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chenjie/桌面/eigen/test/linearstructure.cpp -o CMakeFiles/linearstructure_1.dir/linearstructure.cpp.s

test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o.requires:
.PHONY : test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o.requires

test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o.provides: test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/linearstructure_1.dir/build.make test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o.provides.build
.PHONY : test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o.provides

test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o.provides.build: test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o

# Object files for target linearstructure_1
linearstructure_1_OBJECTS = \
"CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o"

# External object files for target linearstructure_1
linearstructure_1_EXTERNAL_OBJECTS =

test/linearstructure_1: test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o
test/linearstructure_1: test/CMakeFiles/linearstructure_1.dir/build.make
test/linearstructure_1: test/CMakeFiles/linearstructure_1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable linearstructure_1"
	cd /home/chenjie/桌面/eigen/build_dir/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/linearstructure_1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/linearstructure_1.dir/build: test/linearstructure_1
.PHONY : test/CMakeFiles/linearstructure_1.dir/build

test/CMakeFiles/linearstructure_1.dir/requires: test/CMakeFiles/linearstructure_1.dir/linearstructure.cpp.o.requires
.PHONY : test/CMakeFiles/linearstructure_1.dir/requires

test/CMakeFiles/linearstructure_1.dir/clean:
	cd /home/chenjie/桌面/eigen/build_dir/test && $(CMAKE_COMMAND) -P CMakeFiles/linearstructure_1.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/linearstructure_1.dir/clean

test/CMakeFiles/linearstructure_1.dir/depend:
	cd /home/chenjie/桌面/eigen/build_dir && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chenjie/桌面/eigen /home/chenjie/桌面/eigen/test /home/chenjie/桌面/eigen/build_dir /home/chenjie/桌面/eigen/build_dir/test /home/chenjie/桌面/eigen/build_dir/test/CMakeFiles/linearstructure_1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/linearstructure_1.dir/depend

