# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_SOURCE_DIR = /home/alvaro/Documents/Projects/picoC/blinkTest/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alvaro/Documents/Projects/picoC/blinkTest/test

# Include any dependencies generated for this target.
include CMakeFiles/pico_arm.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pico_arm.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pico_arm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pico_arm.dir/flags.make

CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj: CMakeFiles/pico_arm.dir/flags.make
CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj: code/ArmHandler.c
CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj: CMakeFiles/pico_arm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/alvaro/Documents/Projects/picoC/blinkTest/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj"
	/home/alvaro/.pico-sdk/toolchain/13_2_Rel1/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj -MF CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj.d -o CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj -c /home/alvaro/Documents/Projects/picoC/blinkTest/test/code/ArmHandler.c

CMakeFiles/pico_arm.dir/code/ArmHandler.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/pico_arm.dir/code/ArmHandler.c.i"
	/home/alvaro/.pico-sdk/toolchain/13_2_Rel1/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/alvaro/Documents/Projects/picoC/blinkTest/test/code/ArmHandler.c > CMakeFiles/pico_arm.dir/code/ArmHandler.c.i

CMakeFiles/pico_arm.dir/code/ArmHandler.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/pico_arm.dir/code/ArmHandler.c.s"
	/home/alvaro/.pico-sdk/toolchain/13_2_Rel1/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/alvaro/Documents/Projects/picoC/blinkTest/test/code/ArmHandler.c -o CMakeFiles/pico_arm.dir/code/ArmHandler.c.s

# Object files for target pico_arm
pico_arm_OBJECTS = \
"CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj"

# External object files for target pico_arm
pico_arm_EXTERNAL_OBJECTS =

libpico_arm.a: CMakeFiles/pico_arm.dir/code/ArmHandler.c.obj
libpico_arm.a: CMakeFiles/pico_arm.dir/build.make
libpico_arm.a: CMakeFiles/pico_arm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/alvaro/Documents/Projects/picoC/blinkTest/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libpico_arm.a"
	$(CMAKE_COMMAND) -P CMakeFiles/pico_arm.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pico_arm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pico_arm.dir/build: libpico_arm.a
.PHONY : CMakeFiles/pico_arm.dir/build

CMakeFiles/pico_arm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pico_arm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pico_arm.dir/clean

CMakeFiles/pico_arm.dir/depend:
	cd /home/alvaro/Documents/Projects/picoC/blinkTest/test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alvaro/Documents/Projects/picoC/blinkTest/test /home/alvaro/Documents/Projects/picoC/blinkTest/test /home/alvaro/Documents/Projects/picoC/blinkTest/test /home/alvaro/Documents/Projects/picoC/blinkTest/test /home/alvaro/Documents/Projects/picoC/blinkTest/test/CMakeFiles/pico_arm.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/pico_arm.dir/depend

