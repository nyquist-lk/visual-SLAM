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

# Utility rule file for register-mime-info-plotter.

# Include the progress variables for this target.
include tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/progress.make

tools/Plotter/CMakeFiles/register-mime-info-plotter: tools/Plotter/Plotter
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && mkdir -p /home/nlk/.local/share/mime/packages/
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && mkdir -p /home/nlk/.local/share/applications/
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && mkdir -p /home/nlk/.local/share/icons/hicolor/scalable/mimetypes/
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && cp /media/nlk/D/share/code/slam/视觉slam/Pangolin/tools/Plotter/application-x-pangoplot.xml /home/nlk/.local/share/mime/packages/
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && cp /media/nlk/D/share/code/slam/视觉slam/Pangolin/tools/Plotter/../VideoViewer/application-x-pango.svg /home/nlk/.local/share/icons/hicolor/scalable/mimetypes/
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && cp /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter/pangoplot.desktop /home/nlk/.local/share/applications/
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && gtk-update-icon-cache /home/nlk/.local/share/icons/hicolor -f -t
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && update-mime-database /home/nlk/.local/share/mime
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && update-desktop-database /home/nlk/.local/share/applications

register-mime-info-plotter: tools/Plotter/CMakeFiles/register-mime-info-plotter
register-mime-info-plotter: tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/build.make

.PHONY : register-mime-info-plotter

# Rule to build all files generated by this target.
tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/build: register-mime-info-plotter

.PHONY : tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/build

tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/clean:
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter && $(CMAKE_COMMAND) -P CMakeFiles/register-mime-info-plotter.dir/cmake_clean.cmake
.PHONY : tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/clean

tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/depend:
	cd /media/nlk/D/share/code/slam/视觉slam/Pangolin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/nlk/D/share/code/slam/视觉slam/Pangolin /media/nlk/D/share/code/slam/视觉slam/Pangolin/tools/Plotter /media/nlk/D/share/code/slam/视觉slam/Pangolin/build /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter /media/nlk/D/share/code/slam/视觉slam/Pangolin/build/tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/Plotter/CMakeFiles/register-mime-info-plotter.dir/depend

