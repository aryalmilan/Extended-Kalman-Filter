# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.15.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.15.4/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/saziena/Desktop/Ekf

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/saziena/Desktop/Ekf/build

# Include any dependencies generated for this target.
include CMakeFiles/ExtendedKF.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ExtendedKF.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ExtendedKF.dir/flags.make

CMakeFiles/ExtendedKF.dir/src/main.cpp.o: CMakeFiles/ExtendedKF.dir/flags.make
CMakeFiles/ExtendedKF.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/saziena/Desktop/Ekf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ExtendedKF.dir/src/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ExtendedKF.dir/src/main.cpp.o -c /Users/saziena/Desktop/Ekf/src/main.cpp

CMakeFiles/ExtendedKF.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ExtendedKF.dir/src/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/saziena/Desktop/Ekf/src/main.cpp > CMakeFiles/ExtendedKF.dir/src/main.cpp.i

CMakeFiles/ExtendedKF.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ExtendedKF.dir/src/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/saziena/Desktop/Ekf/src/main.cpp -o CMakeFiles/ExtendedKF.dir/src/main.cpp.s

CMakeFiles/ExtendedKF.dir/src/tools.cpp.o: CMakeFiles/ExtendedKF.dir/flags.make
CMakeFiles/ExtendedKF.dir/src/tools.cpp.o: ../src/tools.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/saziena/Desktop/Ekf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ExtendedKF.dir/src/tools.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ExtendedKF.dir/src/tools.cpp.o -c /Users/saziena/Desktop/Ekf/src/tools.cpp

CMakeFiles/ExtendedKF.dir/src/tools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ExtendedKF.dir/src/tools.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/saziena/Desktop/Ekf/src/tools.cpp > CMakeFiles/ExtendedKF.dir/src/tools.cpp.i

CMakeFiles/ExtendedKF.dir/src/tools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ExtendedKF.dir/src/tools.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/saziena/Desktop/Ekf/src/tools.cpp -o CMakeFiles/ExtendedKF.dir/src/tools.cpp.s

CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.o: CMakeFiles/ExtendedKF.dir/flags.make
CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.o: ../src/FusionEKF.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/saziena/Desktop/Ekf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.o -c /Users/saziena/Desktop/Ekf/src/FusionEKF.cpp

CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/saziena/Desktop/Ekf/src/FusionEKF.cpp > CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.i

CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/saziena/Desktop/Ekf/src/FusionEKF.cpp -o CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.s

CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.o: CMakeFiles/ExtendedKF.dir/flags.make
CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.o: ../src/kalman_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/saziena/Desktop/Ekf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.o -c /Users/saziena/Desktop/Ekf/src/kalman_filter.cpp

CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/saziena/Desktop/Ekf/src/kalman_filter.cpp > CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.i

CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/saziena/Desktop/Ekf/src/kalman_filter.cpp -o CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.s

# Object files for target ExtendedKF
ExtendedKF_OBJECTS = \
"CMakeFiles/ExtendedKF.dir/src/main.cpp.o" \
"CMakeFiles/ExtendedKF.dir/src/tools.cpp.o" \
"CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.o" \
"CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.o"

# External object files for target ExtendedKF
ExtendedKF_EXTERNAL_OBJECTS =

ExtendedKF: CMakeFiles/ExtendedKF.dir/src/main.cpp.o
ExtendedKF: CMakeFiles/ExtendedKF.dir/src/tools.cpp.o
ExtendedKF: CMakeFiles/ExtendedKF.dir/src/FusionEKF.cpp.o
ExtendedKF: CMakeFiles/ExtendedKF.dir/src/kalman_filter.cpp.o
ExtendedKF: CMakeFiles/ExtendedKF.dir/build.make
ExtendedKF: CMakeFiles/ExtendedKF.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/saziena/Desktop/Ekf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable ExtendedKF"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ExtendedKF.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ExtendedKF.dir/build: ExtendedKF

.PHONY : CMakeFiles/ExtendedKF.dir/build

CMakeFiles/ExtendedKF.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ExtendedKF.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ExtendedKF.dir/clean

CMakeFiles/ExtendedKF.dir/depend:
	cd /Users/saziena/Desktop/Ekf/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/saziena/Desktop/Ekf /Users/saziena/Desktop/Ekf /Users/saziena/Desktop/Ekf/build /Users/saziena/Desktop/Ekf/build /Users/saziena/Desktop/Ekf/build/CMakeFiles/ExtendedKF.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ExtendedKF.dir/depend

