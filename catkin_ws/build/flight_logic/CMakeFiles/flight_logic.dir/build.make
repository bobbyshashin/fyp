# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/bobby/fyp/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bobby/fyp/catkin_ws/build

# Include any dependencies generated for this target.
include flight_logic/CMakeFiles/flight_logic.dir/depend.make

# Include the progress variables for this target.
include flight_logic/CMakeFiles/flight_logic.dir/progress.make

# Include the compile flags for this target's objects.
include flight_logic/CMakeFiles/flight_logic.dir/flags.make

flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o: flight_logic/CMakeFiles/flight_logic.dir/flags.make
flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o: /home/bobby/fyp/catkin_ws/src/flight_logic/src/flight_logic.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bobby/fyp/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o"
	cd /home/bobby/fyp/catkin_ws/build/flight_logic && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o -c /home/bobby/fyp/catkin_ws/src/flight_logic/src/flight_logic.cpp

flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flight_logic.dir/src/flight_logic.cpp.i"
	cd /home/bobby/fyp/catkin_ws/build/flight_logic && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bobby/fyp/catkin_ws/src/flight_logic/src/flight_logic.cpp > CMakeFiles/flight_logic.dir/src/flight_logic.cpp.i

flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flight_logic.dir/src/flight_logic.cpp.s"
	cd /home/bobby/fyp/catkin_ws/build/flight_logic && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bobby/fyp/catkin_ws/src/flight_logic/src/flight_logic.cpp -o CMakeFiles/flight_logic.dir/src/flight_logic.cpp.s

flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o.requires:
.PHONY : flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o.requires

flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o.provides: flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o.requires
	$(MAKE) -f flight_logic/CMakeFiles/flight_logic.dir/build.make flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o.provides.build
.PHONY : flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o.provides

flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o.provides.build: flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o

# Object files for target flight_logic
flight_logic_OBJECTS = \
"CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o"

# External object files for target flight_logic
flight_logic_EXTERNAL_OBJECTS =

/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: flight_logic/CMakeFiles/flight_logic.dir/build.make
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/libactionlib.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /home/bobby/fyp/catkin_ws/devel/lib/libdji_sdk_lib.a
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/libroscpp.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/librosconsole.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/liblog4cxx.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/librostime.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /opt/ros/indigo/lib/libcpp_common.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic: flight_logic/CMakeFiles/flight_logic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic"
	cd /home/bobby/fyp/catkin_ws/build/flight_logic && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/flight_logic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
flight_logic/CMakeFiles/flight_logic.dir/build: /home/bobby/fyp/catkin_ws/devel/lib/flight_logic/flight_logic
.PHONY : flight_logic/CMakeFiles/flight_logic.dir/build

flight_logic/CMakeFiles/flight_logic.dir/requires: flight_logic/CMakeFiles/flight_logic.dir/src/flight_logic.cpp.o.requires
.PHONY : flight_logic/CMakeFiles/flight_logic.dir/requires

flight_logic/CMakeFiles/flight_logic.dir/clean:
	cd /home/bobby/fyp/catkin_ws/build/flight_logic && $(CMAKE_COMMAND) -P CMakeFiles/flight_logic.dir/cmake_clean.cmake
.PHONY : flight_logic/CMakeFiles/flight_logic.dir/clean

flight_logic/CMakeFiles/flight_logic.dir/depend:
	cd /home/bobby/fyp/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bobby/fyp/catkin_ws/src /home/bobby/fyp/catkin_ws/src/flight_logic /home/bobby/fyp/catkin_ws/build /home/bobby/fyp/catkin_ws/build/flight_logic /home/bobby/fyp/catkin_ws/build/flight_logic/CMakeFiles/flight_logic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : flight_logic/CMakeFiles/flight_logic.dir/depend

