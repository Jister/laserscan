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
include test/CMakeFiles/diagonal_2.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/diagonal_2.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/diagonal_2.dir/flags.make

test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o: test/CMakeFiles/diagonal_2.dir/flags.make
test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o: ../test/diagonal.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chenjie/桌面/eigen/build_dir/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o"
	cd /home/chenjie/桌面/eigen/build_dir/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/diagonal_2.dir/diagonal.cpp.o -c /home/chenjie/桌面/eigen/test/diagonal.cpp

test/CMakeFiles/diagonal_2.dir/diagonal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/diagonal_2.dir/diagonal.cpp.i"
	cd /home/chenjie/桌面/eigen/build_dir/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chenjie/桌面/eigen/test/diagonal.cpp > CMakeFiles/diagonal_2.dir/diagonal.cpp.i

test/CMakeFiles/diagonal_2.dir/diagonal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/diagonal_2.dir/diagonal.cpp.s"
	cd /home/chenjie/桌面/eigen/build_dir/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chenjie/桌面/eigen/test/diagonal.cpp -o CMakeFiles/diagonal_2.dir/diagonal.cpp.s

test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o.requires:
.PHONY : test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o.requires

test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o.provides: test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/diagonal_2.dir/build.make test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o.provides.build
.PHONY : test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o.provides

test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o.provides.build: test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o

# Object files for target diagonal_2
diagonal_2_OBJECTS = \
"CMakeFiles/diagonal_2.dir/diagonal.cpp.o"

# External object files for target diagonal_2
diagonal_2_EXTERNAL_OBJECTS =

test/diagonal_2: test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o
test/diagonal_2: test/CMakeFiles/diagonal_2.dir/build.make
test/diagonal_2: test/CMakeFiles/diagonal_2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable diagonal_2"
	cd /home/chenjie/桌面/eigen/build_dir/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/diagonal_2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/diagonal_2.dir/build: test/diagonal_2
.PHONY : test/CMakeFiles/diagonal_2.dir/build

test/CMakeFiles/diagonal_2.dir/requires: test/CMakeFiles/diagonal_2.dir/diagonal.cpp.o.requires
.PHONY : test/CMakeFiles/diagonal_2.dir/requires

test/CMakeFiles/diagonal_2.dir/clean:
	cd /home/chenjie/桌面/eigen/build_dir/test && $(CMAKE_COMMAND) -P CMakeFiles/diagonal_2.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/diagonal_2.dir/clean

test/CMakeFiles/diagonal_2.dir/depend:
	cd /home/chenjie/桌面/eigen/build_dir && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chenjie/桌面/eigen /home/chenjie/桌面/eigen/test /home/chenjie/桌面/eigen/build_dir /home/chenjie/桌面/eigen/build_dir/test /home/chenjie/桌面/eigen/build_dir/test/CMakeFiles/diagonal_2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/diagonal_2.dir/depend

