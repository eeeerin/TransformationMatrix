# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /home/pqy/下载/clion-2017.3.4/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/pqy/下载/clion-2017.3.4/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pqy/CLionProjects/PA2TransformationMatrix

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pqy/CLionProjects/PA2TransformationMatrix/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/PA2TransformationMatrix.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/PA2TransformationMatrix.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/PA2TransformationMatrix.dir/flags.make

CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o: CMakeFiles/PA2TransformationMatrix.dir/flags.make
CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pqy/CLionProjects/PA2TransformationMatrix/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o -c /home/pqy/CLionProjects/PA2TransformationMatrix/main.cpp

CMakeFiles/PA2TransformationMatrix.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PA2TransformationMatrix.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pqy/CLionProjects/PA2TransformationMatrix/main.cpp > CMakeFiles/PA2TransformationMatrix.dir/main.cpp.i

CMakeFiles/PA2TransformationMatrix.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PA2TransformationMatrix.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pqy/CLionProjects/PA2TransformationMatrix/main.cpp -o CMakeFiles/PA2TransformationMatrix.dir/main.cpp.s

CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o.requires

CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o.provides: CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/PA2TransformationMatrix.dir/build.make CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o.provides

CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o.provides.build: CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o


# Object files for target PA2TransformationMatrix
PA2TransformationMatrix_OBJECTS = \
"CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o"

# External object files for target PA2TransformationMatrix
PA2TransformationMatrix_EXTERNAL_OBJECTS =

PA2TransformationMatrix: CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o
PA2TransformationMatrix: CMakeFiles/PA2TransformationMatrix.dir/build.make
PA2TransformationMatrix: CMakeFiles/PA2TransformationMatrix.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pqy/CLionProjects/PA2TransformationMatrix/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable PA2TransformationMatrix"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PA2TransformationMatrix.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/PA2TransformationMatrix.dir/build: PA2TransformationMatrix

.PHONY : CMakeFiles/PA2TransformationMatrix.dir/build

CMakeFiles/PA2TransformationMatrix.dir/requires: CMakeFiles/PA2TransformationMatrix.dir/main.cpp.o.requires

.PHONY : CMakeFiles/PA2TransformationMatrix.dir/requires

CMakeFiles/PA2TransformationMatrix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PA2TransformationMatrix.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PA2TransformationMatrix.dir/clean

CMakeFiles/PA2TransformationMatrix.dir/depend:
	cd /home/pqy/CLionProjects/PA2TransformationMatrix/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pqy/CLionProjects/PA2TransformationMatrix /home/pqy/CLionProjects/PA2TransformationMatrix /home/pqy/CLionProjects/PA2TransformationMatrix/cmake-build-debug /home/pqy/CLionProjects/PA2TransformationMatrix/cmake-build-debug /home/pqy/CLionProjects/PA2TransformationMatrix/cmake-build-debug/CMakeFiles/PA2TransformationMatrix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PA2TransformationMatrix.dir/depend

