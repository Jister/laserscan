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
include doc/snippets/CMakeFiles/compile_Cwise_pow.dir/depend.make

# Include the progress variables for this target.
include doc/snippets/CMakeFiles/compile_Cwise_pow.dir/progress.make

# Include the compile flags for this target's objects.
include doc/snippets/CMakeFiles/compile_Cwise_pow.dir/flags.make

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o: doc/snippets/CMakeFiles/compile_Cwise_pow.dir/flags.make
doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o: doc/snippets/compile_Cwise_pow.cpp
doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o: ../doc/snippets/Cwise_pow.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chenjie/桌面/eigen/build_dir/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o"
	cd /home/chenjie/桌面/eigen/build_dir/doc/snippets && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o -c /home/chenjie/桌面/eigen/build_dir/doc/snippets/compile_Cwise_pow.cpp

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.i"
	cd /home/chenjie/桌面/eigen/build_dir/doc/snippets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chenjie/桌面/eigen/build_dir/doc/snippets/compile_Cwise_pow.cpp > CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.i

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.s"
	cd /home/chenjie/桌面/eigen/build_dir/doc/snippets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chenjie/桌面/eigen/build_dir/doc/snippets/compile_Cwise_pow.cpp -o CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.s

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o.requires:
.PHONY : doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o.requires

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o.provides: doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o.requires
	$(MAKE) -f doc/snippets/CMakeFiles/compile_Cwise_pow.dir/build.make doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o.provides.build
.PHONY : doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o.provides

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o.provides.build: doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o

# Object files for target compile_Cwise_pow
compile_Cwise_pow_OBJECTS = \
"CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o"

# External object files for target compile_Cwise_pow
compile_Cwise_pow_EXTERNAL_OBJECTS =

doc/snippets/compile_Cwise_pow: doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o
doc/snippets/compile_Cwise_pow: doc/snippets/CMakeFiles/compile_Cwise_pow.dir/build.make
doc/snippets/compile_Cwise_pow: doc/snippets/CMakeFiles/compile_Cwise_pow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable compile_Cwise_pow"
	cd /home/chenjie/桌面/eigen/build_dir/doc/snippets && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/compile_Cwise_pow.dir/link.txt --verbose=$(VERBOSE)
	cd /home/chenjie/桌面/eigen/build_dir/doc/snippets && ./compile_Cwise_pow >/home/chenjie/桌面/eigen/build_dir/doc/snippets/Cwise_pow.out

# Rule to build all files generated by this target.
doc/snippets/CMakeFiles/compile_Cwise_pow.dir/build: doc/snippets/compile_Cwise_pow
.PHONY : doc/snippets/CMakeFiles/compile_Cwise_pow.dir/build

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/requires: doc/snippets/CMakeFiles/compile_Cwise_pow.dir/compile_Cwise_pow.cpp.o.requires
.PHONY : doc/snippets/CMakeFiles/compile_Cwise_pow.dir/requires

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/clean:
	cd /home/chenjie/桌面/eigen/build_dir/doc/snippets && $(CMAKE_COMMAND) -P CMakeFiles/compile_Cwise_pow.dir/cmake_clean.cmake
.PHONY : doc/snippets/CMakeFiles/compile_Cwise_pow.dir/clean

doc/snippets/CMakeFiles/compile_Cwise_pow.dir/depend:
	cd /home/chenjie/桌面/eigen/build_dir && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chenjie/桌面/eigen /home/chenjie/桌面/eigen/doc/snippets /home/chenjie/桌面/eigen/build_dir /home/chenjie/桌面/eigen/build_dir/doc/snippets /home/chenjie/桌面/eigen/build_dir/doc/snippets/CMakeFiles/compile_Cwise_pow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : doc/snippets/CMakeFiles/compile_Cwise_pow.dir/depend

