# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /media/nlk/D/share/code/slam/视觉slam/Pangolin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/nlk/D/share/code/slam/视觉slam/Pangolin/build

# Include any dependencies generated for this target.
include examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/depend.make

# Include the progress variables for this target.
include examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/progress.make

# Include the compile flags for this target's objects.
include examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/flags.make

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o: examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/flags.make
examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o: ../examples/HelloPangolinThreads/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/nlk/D/share/code/slam/视觉slam/Pangolin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o"
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/examples/HelloPangolinThreads && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HelloPangolinThreads.dir/main.cpp.o -c /media/nlk/D/share/code/slam/视觉slam/Pangolin/examples/HelloPangolinThreads/main.cpp

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HelloPangolinThreads.dir/main.cpp.i"
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/examples/HelloPangolinThreads && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/nlk/D/share/code/slam/视觉slam/Pangolin/examples/HelloPangolinThreads/main.cpp > CMakeFiles/HelloPangolinThreads.dir/main.cpp.i

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HelloPangolinThreads.dir/main.cpp.s"
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/examples/HelloPangolinThreads && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/nlk/D/share/code/slam/视觉slam/Pangolin/examples/HelloPangolinThreads/main.cpp -o CMakeFiles/HelloPangolinThreads.dir/main.cpp.s

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o.requires:

.PHONY : examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o.requires

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o.provides: examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o.requires
	$(MAKE) -f examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/build.make examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o.provides.build
.PHONY : examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o.provides

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o.provides.build: examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o


# Object files for target HelloPangolinThreads
HelloPangolinThreads_OBJECTS = \
"CMakeFiles/HelloPangolinThreads.dir/main.cpp.o"

# External object files for target HelloPangolinThreads
HelloPangolinThreads_EXTERNAL_OBJECTS =

examples/HelloPangolinThreads/HelloPangolinThreads: examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o
examples/HelloPangolinThreads/HelloPangolinThreads: examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/build.make
examples/HelloPangolinThreads/HelloPangolinThreads: src/libpangolin.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libOpenGL.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libGLX.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libEGL.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libSM.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libICE.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libX11.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libXext.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libOpenGL.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libGLX.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libEGL.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libSM.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libICE.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libX11.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libXext.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libdc1394.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libavcodec.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libavformat.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libavutil.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libswscale.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libavdevice.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/libOpenNI.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/libOpenNI2.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libpng.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libz.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libtiff.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/libIlmImf.so
examples/HelloPangolinThreads/HelloPangolinThreads: /usr/lib/x86_64-linux-gnu/liblz4.so
examples/HelloPangolinThreads/HelloPangolinThreads: examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/nlk/D/share/code/slam/视觉slam/Pangolin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable HelloPangolinThreads"
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/examples/HelloPangolinThreads && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HelloPangolinThreads.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/build: examples/HelloPangolinThreads/HelloPangolinThreads

.PHONY : examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/build

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/requires: examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/main.cpp.o.requires

.PHONY : examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/requires

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/clean:
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/examples/HelloPangolinThreads && $(CMAKE_COMMAND) -P CMakeFiles/HelloPangolinThreads.dir/cmake_clean.cmake
.PHONY : examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/clean

examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/depend:
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/nlk/D/share/code/slam/视觉slam/Pangolin /media/nlk/D/share/code/slam/视觉slam/Pangolin/examples/HelloPangolinThreads /media/nlk/D/share/code/slam/视觉slam/Pangolin/build /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/examples/HelloPangolinThreads /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/HelloPangolinThreads/CMakeFiles/HelloPangolinThreads.dir/depend

