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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liangzhu/stm32/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liangzhu/stm32/test/build

# Include any dependencies generated for this target.
include CMakeFiles/stm32f4.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stm32f4.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stm32f4.dir/flags.make

CMakeFiles/stm32f4.dir/Src/main.c.obj: CMakeFiles/stm32f4.dir/flags.make
CMakeFiles/stm32f4.dir/Src/main.c.obj: ../Src/main.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/stm32f4.dir/Src/main.c.obj"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/stm32f4.dir/Src/main.c.obj   -c /home/liangzhu/stm32/test/Src/main.c

CMakeFiles/stm32f4.dir/Src/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stm32f4.dir/Src/main.c.i"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Src/main.c > CMakeFiles/stm32f4.dir/Src/main.c.i

CMakeFiles/stm32f4.dir/Src/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stm32f4.dir/Src/main.c.s"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Src/main.c -o CMakeFiles/stm32f4.dir/Src/main.c.s

CMakeFiles/stm32f4.dir/Src/main.c.obj.requires:
.PHONY : CMakeFiles/stm32f4.dir/Src/main.c.obj.requires

CMakeFiles/stm32f4.dir/Src/main.c.obj.provides: CMakeFiles/stm32f4.dir/Src/main.c.obj.requires
	$(MAKE) -f CMakeFiles/stm32f4.dir/build.make CMakeFiles/stm32f4.dir/Src/main.c.obj.provides.build
.PHONY : CMakeFiles/stm32f4.dir/Src/main.c.obj.provides

CMakeFiles/stm32f4.dir/Src/main.c.obj.provides.build: CMakeFiles/stm32f4.dir/Src/main.c.obj

CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj: CMakeFiles/stm32f4.dir/flags.make
CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj: ../Src/stm32f4xx_hal_msp.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj   -c /home/liangzhu/stm32/test/Src/stm32f4xx_hal_msp.c

CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.i"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Src/stm32f4xx_hal_msp.c > CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.i

CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.s"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Src/stm32f4xx_hal_msp.c -o CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.s

CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj.requires:
.PHONY : CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj.requires

CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj.provides: CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj.requires
	$(MAKE) -f CMakeFiles/stm32f4.dir/build.make CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj.provides.build
.PHONY : CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj.provides

CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj.provides.build: CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj

CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj: CMakeFiles/stm32f4.dir/flags.make
CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj: ../Src/stm32f4xx_it.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj   -c /home/liangzhu/stm32/test/Src/stm32f4xx_it.c

CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.i"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Src/stm32f4xx_it.c > CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.i

CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.s"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Src/stm32f4xx_it.c -o CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.s

CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj.requires:
.PHONY : CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj.requires

CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj.provides: CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj.requires
	$(MAKE) -f CMakeFiles/stm32f4.dir/build.make CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj.provides.build
.PHONY : CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj.provides

CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj.provides.build: CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj

CMakeFiles/stm32f4.dir/Src/newlib.c.obj: CMakeFiles/stm32f4.dir/flags.make
CMakeFiles/stm32f4.dir/Src/newlib.c.obj: ../Src/newlib.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/stm32f4.dir/Src/newlib.c.obj"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/stm32f4.dir/Src/newlib.c.obj   -c /home/liangzhu/stm32/test/Src/newlib.c

CMakeFiles/stm32f4.dir/Src/newlib.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stm32f4.dir/Src/newlib.c.i"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Src/newlib.c > CMakeFiles/stm32f4.dir/Src/newlib.c.i

CMakeFiles/stm32f4.dir/Src/newlib.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stm32f4.dir/Src/newlib.c.s"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Src/newlib.c -o CMakeFiles/stm32f4.dir/Src/newlib.c.s

CMakeFiles/stm32f4.dir/Src/newlib.c.obj.requires:
.PHONY : CMakeFiles/stm32f4.dir/Src/newlib.c.obj.requires

CMakeFiles/stm32f4.dir/Src/newlib.c.obj.provides: CMakeFiles/stm32f4.dir/Src/newlib.c.obj.requires
	$(MAKE) -f CMakeFiles/stm32f4.dir/build.make CMakeFiles/stm32f4.dir/Src/newlib.c.obj.provides.build
.PHONY : CMakeFiles/stm32f4.dir/Src/newlib.c.obj.provides

CMakeFiles/stm32f4.dir/Src/newlib.c.obj.provides.build: CMakeFiles/stm32f4.dir/Src/newlib.c.obj

# Object files for target stm32f4
stm32f4_OBJECTS = \
"CMakeFiles/stm32f4.dir/Src/main.c.obj" \
"CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj" \
"CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj" \
"CMakeFiles/stm32f4.dir/Src/newlib.c.obj"

# External object files for target stm32f4
stm32f4_EXTERNAL_OBJECTS =

stm32f4: CMakeFiles/stm32f4.dir/Src/main.c.obj
stm32f4: CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj
stm32f4: CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj
stm32f4: CMakeFiles/stm32f4.dir/Src/newlib.c.obj
stm32f4: CMakeFiles/stm32f4.dir/build.make
stm32f4: Drivers/STM32F4xx_HAL_Driver/libHAL.a
stm32f4: Drivers/CMSIS/libCMSIS.a
stm32f4: RawOS/librtos.a
stm32f4: RawOS/Port/libPort.a
stm32f4: CMakeFiles/stm32f4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C executable stm32f4"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stm32f4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stm32f4.dir/build: stm32f4
.PHONY : CMakeFiles/stm32f4.dir/build

CMakeFiles/stm32f4.dir/requires: CMakeFiles/stm32f4.dir/Src/main.c.obj.requires
CMakeFiles/stm32f4.dir/requires: CMakeFiles/stm32f4.dir/Src/stm32f4xx_hal_msp.c.obj.requires
CMakeFiles/stm32f4.dir/requires: CMakeFiles/stm32f4.dir/Src/stm32f4xx_it.c.obj.requires
CMakeFiles/stm32f4.dir/requires: CMakeFiles/stm32f4.dir/Src/newlib.c.obj.requires
.PHONY : CMakeFiles/stm32f4.dir/requires

CMakeFiles/stm32f4.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stm32f4.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stm32f4.dir/clean

CMakeFiles/stm32f4.dir/depend:
	cd /home/liangzhu/stm32/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liangzhu/stm32/test /home/liangzhu/stm32/test /home/liangzhu/stm32/test/build /home/liangzhu/stm32/test/build /home/liangzhu/stm32/test/build/CMakeFiles/stm32f4.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stm32f4.dir/depend

