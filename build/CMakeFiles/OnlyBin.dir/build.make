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
CMAKE_SOURCE_DIR = /home/alessio/dev/GitRepo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alessio/dev/GitRepo/build

# Include any dependencies generated for this target.
include CMakeFiles/OnlyBin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/OnlyBin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OnlyBin.dir/flags.make

CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o: CMakeFiles/OnlyBin.dir/flags.make
CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o: ../src/only_bin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alessio/dev/GitRepo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o -c /home/alessio/dev/GitRepo/src/only_bin.cpp

CMakeFiles/OnlyBin.dir/src/only_bin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OnlyBin.dir/src/only_bin.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alessio/dev/GitRepo/src/only_bin.cpp > CMakeFiles/OnlyBin.dir/src/only_bin.cpp.i

CMakeFiles/OnlyBin.dir/src/only_bin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OnlyBin.dir/src/only_bin.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alessio/dev/GitRepo/src/only_bin.cpp -o CMakeFiles/OnlyBin.dir/src/only_bin.cpp.s

CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o.requires:

.PHONY : CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o.requires

CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o.provides: CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o.requires
	$(MAKE) -f CMakeFiles/OnlyBin.dir/build.make CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o.provides.build
.PHONY : CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o.provides

CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o.provides.build: CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o


CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o: CMakeFiles/OnlyBin.dir/flags.make
CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o: ../src/binsegmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alessio/dev/GitRepo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o -c /home/alessio/dev/GitRepo/src/binsegmentation.cpp

CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alessio/dev/GitRepo/src/binsegmentation.cpp > CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.i

CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alessio/dev/GitRepo/src/binsegmentation.cpp -o CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.s

CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o.requires:

.PHONY : CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o.requires

CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o.provides: CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/OnlyBin.dir/build.make CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o.provides.build
.PHONY : CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o.provides

CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o.provides.build: CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o


# Object files for target OnlyBin
OnlyBin_OBJECTS = \
"CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o" \
"CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o"

# External object files for target OnlyBin
OnlyBin_EXTERNAL_OBJECTS =

OnlyBin: CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o
OnlyBin: CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o
OnlyBin: CMakeFiles/OnlyBin.dir/build.make
OnlyBin: /usr/local/lib/libpcl_stereo.so
OnlyBin: /usr/local/lib/libpcl_surface.so
OnlyBin: /usr/local/lib/libpcl_recognition.so
OnlyBin: /usr/local/lib/libpcl_keypoints.so
OnlyBin: /usr/local/lib/libpcl_people.so
OnlyBin: /usr/local/lib/libpcl_tracking.so
OnlyBin: /usr/local/lib/libpcl_outofcore.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_system.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_regex.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libqhull.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_system.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libboost_regex.so
OnlyBin: /usr/local/lib/libpcl_registration.so
OnlyBin: /usr/local/lib/libpcl_segmentation.so
OnlyBin: /usr/local/lib/libpcl_features.so
OnlyBin: /usr/local/lib/libvtkChartsCore-7.1.so.1
OnlyBin: /usr/local/lib/libvtkInfovisCore-7.1.so.1
OnlyBin: /usr/local/lib/libvtkIOGeometry-7.1.so.1
OnlyBin: /usr/local/lib/libvtkIOLegacy-7.1.so.1
OnlyBin: /usr/local/lib/libvtkIOPLY-7.1.so.1
OnlyBin: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
OnlyBin: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
OnlyBin: /usr/local/lib/libvtkViewsCore-7.1.so.1
OnlyBin: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
OnlyBin: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
OnlyBin: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
OnlyBin: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
OnlyBin: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
OnlyBin: /usr/local/lib/libvtkImagingFourier-7.1.so.1
OnlyBin: /usr/local/lib/libvtkalglib-7.1.so.1
OnlyBin: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
OnlyBin: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
OnlyBin: /usr/local/lib/libvtkImagingSources-7.1.so.1
OnlyBin: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
OnlyBin: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
OnlyBin: /usr/local/lib/libvtkImagingColor-7.1.so.1
OnlyBin: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
OnlyBin: /usr/local/lib/libvtkIOXML-7.1.so.1
OnlyBin: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
OnlyBin: /usr/local/lib/libvtkIOCore-7.1.so.1
OnlyBin: /usr/local/lib/libvtkexpat-7.1.so.1
OnlyBin: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
OnlyBin: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
OnlyBin: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
OnlyBin: /usr/local/lib/libvtkfreetype-7.1.so.1
OnlyBin: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
OnlyBin: /usr/local/lib/libvtkImagingCore-7.1.so.1
OnlyBin: /usr/local/lib/libvtkRenderingCore-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonColor-7.1.so.1
OnlyBin: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
OnlyBin: /usr/local/lib/libvtkFiltersSources-7.1.so.1
OnlyBin: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
OnlyBin: /usr/local/lib/libvtkFiltersCore-7.1.so.1
OnlyBin: /usr/local/lib/libvtkIOImage-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonMisc-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonMath-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonSystem-7.1.so.1
OnlyBin: /usr/local/lib/libvtkCommonCore-7.1.so.1
OnlyBin: /usr/local/lib/libvtksys-7.1.so.1
OnlyBin: /usr/local/lib/libvtkDICOMParser-7.1.so.1
OnlyBin: /usr/local/lib/libvtkmetaio-7.1.so.1
OnlyBin: /usr/local/lib/libvtkpng-7.1.so.1
OnlyBin: /usr/local/lib/libvtktiff-7.1.so.1
OnlyBin: /usr/local/lib/libvtkzlib-7.1.so.1
OnlyBin: /usr/local/lib/libvtkjpeg-7.1.so.1
OnlyBin: /usr/lib/x86_64-linux-gnu/libm.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libSM.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libICE.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libX11.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libXext.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libXt.so
OnlyBin: /usr/local/lib/libvtkglew-7.1.so.1
OnlyBin: /usr/local/lib/libpcl_ml.so
OnlyBin: /usr/local/lib/libpcl_filters.so
OnlyBin: /usr/local/lib/libpcl_sample_consensus.so
OnlyBin: /usr/local/lib/libpcl_visualization.so
OnlyBin: /usr/local/lib/libpcl_search.so
OnlyBin: /usr/local/lib/libpcl_kdtree.so
OnlyBin: /usr/local/lib/libpcl_io.so
OnlyBin: /usr/local/lib/libpcl_octree.so
OnlyBin: /usr/local/lib/libpcl_common.so
OnlyBin: /usr/lib/x86_64-linux-gnu/libqhull.so
OnlyBin: CMakeFiles/OnlyBin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alessio/dev/GitRepo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable OnlyBin"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OnlyBin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OnlyBin.dir/build: OnlyBin

.PHONY : CMakeFiles/OnlyBin.dir/build

CMakeFiles/OnlyBin.dir/requires: CMakeFiles/OnlyBin.dir/src/only_bin.cpp.o.requires
CMakeFiles/OnlyBin.dir/requires: CMakeFiles/OnlyBin.dir/src/binsegmentation.cpp.o.requires

.PHONY : CMakeFiles/OnlyBin.dir/requires

CMakeFiles/OnlyBin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OnlyBin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OnlyBin.dir/clean

CMakeFiles/OnlyBin.dir/depend:
	cd /home/alessio/dev/GitRepo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alessio/dev/GitRepo /home/alessio/dev/GitRepo /home/alessio/dev/GitRepo/build /home/alessio/dev/GitRepo/build /home/alessio/dev/GitRepo/build/CMakeFiles/OnlyBin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OnlyBin.dir/depend

