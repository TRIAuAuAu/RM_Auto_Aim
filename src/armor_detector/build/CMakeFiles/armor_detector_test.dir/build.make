# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/auauau/RM_Auto_Aim/src/armor_detector

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/auauau/RM_Auto_Aim/src/armor_detector/build

# Include any dependencies generated for this target.
include CMakeFiles/armor_detector_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/armor_detector_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/armor_detector_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/armor_detector_test.dir/flags.make

CMakeFiles/armor_detector_test.dir/test/test.cpp.o: CMakeFiles/armor_detector_test.dir/flags.make
CMakeFiles/armor_detector_test.dir/test/test.cpp.o: ../test/test.cpp
CMakeFiles/armor_detector_test.dir/test/test.cpp.o: CMakeFiles/armor_detector_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/auauau/RM_Auto_Aim/src/armor_detector/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/armor_detector_test.dir/test/test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/armor_detector_test.dir/test/test.cpp.o -MF CMakeFiles/armor_detector_test.dir/test/test.cpp.o.d -o CMakeFiles/armor_detector_test.dir/test/test.cpp.o -c /home/auauau/RM_Auto_Aim/src/armor_detector/test/test.cpp

CMakeFiles/armor_detector_test.dir/test/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/armor_detector_test.dir/test/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/auauau/RM_Auto_Aim/src/armor_detector/test/test.cpp > CMakeFiles/armor_detector_test.dir/test/test.cpp.i

CMakeFiles/armor_detector_test.dir/test/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/armor_detector_test.dir/test/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/auauau/RM_Auto_Aim/src/armor_detector/test/test.cpp -o CMakeFiles/armor_detector_test.dir/test/test.cpp.s

# Object files for target armor_detector_test
armor_detector_test_OBJECTS = \
"CMakeFiles/armor_detector_test.dir/test/test.cpp.o"

# External object files for target armor_detector_test
armor_detector_test_EXTERNAL_OBJECTS =

armor_detector_test: CMakeFiles/armor_detector_test.dir/test/test.cpp.o
armor_detector_test: CMakeFiles/armor_detector_test.dir/build.make
armor_detector_test: libarmor_detector.so
armor_detector_test: /home/auauau/RM_Auto_Aim/install/auto_aim_interfaces/lib/libauto_aim_interfaces__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /home/auauau/RM_Auto_Aim/install/auto_aim_interfaces/lib/libauto_aim_interfaces__rosidl_typesupport_introspection_c.so
armor_detector_test: /home/auauau/RM_Auto_Aim/install/auto_aim_interfaces/lib/libauto_aim_interfaces__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /home/auauau/RM_Auto_Aim/install/auto_aim_interfaces/lib/libauto_aim_interfaces__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /home/auauau/RM_Auto_Aim/install/auto_aim_interfaces/lib/libauto_aim_interfaces__rosidl_typesupport_cpp.so
armor_detector_test: /home/auauau/RM_Auto_Aim/install/auto_aim_interfaces/lib/libauto_aim_interfaces__rosidl_generator_py.so
armor_detector_test: /home/auauau/RM_Auto_Aim/install/auto_aim_interfaces/lib/libauto_aim_interfaces__rosidl_typesupport_c.so
armor_detector_test: /home/auauau/RM_Auto_Aim/install/auto_aim_interfaces/lib/libauto_aim_interfaces__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libcomponent_manager.so
armor_detector_test: /opt/ros/humble/lib/libclass_loader.so
armor_detector_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libvisualization_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libvisualization_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libvisualization_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libvisualization_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libvisualization_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libvisualization_msgs__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libvisualization_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libvisualization_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libcv_bridge.so
armor_detector_test: /opt/ros/humble/lib/x86_64-linux-gnu/libimage_transport.so
armor_detector_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /usr/lib/x86_64-linux-gnu/liborocos-kdl.so
armor_detector_test: /opt/ros/humble/lib/libtf2_ros.so
armor_detector_test: /opt/ros/humble/lib/libmessage_filters.so
armor_detector_test: /opt/ros/humble/lib/libtf2.so
armor_detector_test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
armor_detector_test: /opt/ros/humble/lib/librclcpp_action.so
armor_detector_test: /opt/ros/humble/lib/librclcpp.so
armor_detector_test: /opt/ros/humble/lib/liblibstatistics_collector.so
armor_detector_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/librcl_action.so
armor_detector_test: /opt/ros/humble/lib/librcl.so
armor_detector_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/librcl_yaml_param_parser.so
armor_detector_test: /opt/ros/humble/lib/libyaml.so
armor_detector_test: /opt/ros/humble/lib/libtracetools.so
armor_detector_test: /opt/ros/humble/lib/librmw_implementation.so
armor_detector_test: /opt/ros/humble/lib/libament_index_cpp.so
armor_detector_test: /opt/ros/humble/lib/librcl_logging_spdlog.so
armor_detector_test: /opt/ros/humble/lib/librcl_logging_interface.so
armor_detector_test: /usr/lib/x86_64-linux-gnu/libfmt.so.8.1.1
armor_detector_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
armor_detector_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
armor_detector_test: /opt/ros/humble/lib/libfastcdr.so.1.0.24
armor_detector_test: /opt/ros/humble/lib/librmw.so
armor_detector_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
armor_detector_test: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
armor_detector_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
armor_detector_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
armor_detector_test: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_py.so
armor_detector_test: /usr/lib/x86_64-linux-gnu/libpython3.10.so
armor_detector_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/librosidl_typesupport_c.so
armor_detector_test: /opt/ros/humble/lib/librcpputils.so
armor_detector_test: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_c.so
armor_detector_test: /opt/ros/humble/lib/librosidl_runtime_c.so
armor_detector_test: /opt/ros/humble/lib/librcutils.so
armor_detector_test: /usr/local/lib/libopencv_highgui.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_ml.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_objdetect.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_photo.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_stitching.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_video.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_calib3d.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_dnn.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_features2d.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_flann.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_videoio.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_imgcodecs.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_imgproc.so.4.8.0
armor_detector_test: /usr/local/lib/libopencv_core.so.4.8.0
armor_detector_test: CMakeFiles/armor_detector_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/auauau/RM_Auto_Aim/src/armor_detector/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable armor_detector_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/armor_detector_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/armor_detector_test.dir/build: armor_detector_test
.PHONY : CMakeFiles/armor_detector_test.dir/build

CMakeFiles/armor_detector_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/armor_detector_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/armor_detector_test.dir/clean

CMakeFiles/armor_detector_test.dir/depend:
	cd /home/auauau/RM_Auto_Aim/src/armor_detector/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/auauau/RM_Auto_Aim/src/armor_detector /home/auauau/RM_Auto_Aim/src/armor_detector /home/auauau/RM_Auto_Aim/src/armor_detector/build /home/auauau/RM_Auto_Aim/src/armor_detector/build /home/auauau/RM_Auto_Aim/src/armor_detector/build/CMakeFiles/armor_detector_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/armor_detector_test.dir/depend

