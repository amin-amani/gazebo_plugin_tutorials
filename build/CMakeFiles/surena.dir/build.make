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
CMAKE_SOURCE_DIR = /home/ala/gazebo_plugin_tutorials

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ala/gazebo_plugin_tutorials/build

# Include any dependencies generated for this target.
include CMakeFiles/surena.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/surena.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/surena.dir/flags.make

CMakeFiles/surena.dir/surena.cc.o: CMakeFiles/surena.dir/flags.make
CMakeFiles/surena.dir/surena.cc.o: ../surena.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ala/gazebo_plugin_tutorials/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/surena.dir/surena.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/surena.dir/surena.cc.o -c /home/ala/gazebo_plugin_tutorials/surena.cc

CMakeFiles/surena.dir/surena.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/surena.dir/surena.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ala/gazebo_plugin_tutorials/surena.cc > CMakeFiles/surena.dir/surena.cc.i

CMakeFiles/surena.dir/surena.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/surena.dir/surena.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ala/gazebo_plugin_tutorials/surena.cc -o CMakeFiles/surena.dir/surena.cc.s

CMakeFiles/surena.dir/surena.cc.o.requires:

.PHONY : CMakeFiles/surena.dir/surena.cc.o.requires

CMakeFiles/surena.dir/surena.cc.o.provides: CMakeFiles/surena.dir/surena.cc.o.requires
	$(MAKE) -f CMakeFiles/surena.dir/build.make CMakeFiles/surena.dir/surena.cc.o.provides.build
.PHONY : CMakeFiles/surena.dir/surena.cc.o.provides

CMakeFiles/surena.dir/surena.cc.o.provides.build: CMakeFiles/surena.dir/surena.cc.o


# Object files for target surena
surena_OBJECTS = \
"CMakeFiles/surena.dir/surena.cc.o"

# External object files for target surena
surena_EXTERNAL_OBJECTS =

libsurena.so: CMakeFiles/surena.dir/surena.cc.o
libsurena.so: CMakeFiles/surena.dir/build.make
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libsurena.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libsurena.so: CMakeFiles/surena.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ala/gazebo_plugin_tutorials/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libsurena.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/surena.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/surena.dir/build: libsurena.so

.PHONY : CMakeFiles/surena.dir/build

CMakeFiles/surena.dir/requires: CMakeFiles/surena.dir/surena.cc.o.requires

.PHONY : CMakeFiles/surena.dir/requires

CMakeFiles/surena.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/surena.dir/cmake_clean.cmake
.PHONY : CMakeFiles/surena.dir/clean

CMakeFiles/surena.dir/depend:
	cd /home/ala/gazebo_plugin_tutorials/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ala/gazebo_plugin_tutorials /home/ala/gazebo_plugin_tutorials /home/ala/gazebo_plugin_tutorials/build /home/ala/gazebo_plugin_tutorials/build /home/ala/gazebo_plugin_tutorials/build/CMakeFiles/surena.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/surena.dir/depend

