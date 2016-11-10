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
include Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/depend.make

# Include the progress variables for this target.
include Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/progress.make

# Include the compile flags for this target's objects.
include Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/croutine.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/croutine.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/croutine.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/croutine.c > CMakeFiles/freertos.dir/croutine.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/croutine.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/croutine.c -o CMakeFiles/freertos.dir/croutine.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/tasks.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/tasks.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/tasks.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/tasks.c > CMakeFiles/freertos.dir/tasks.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/tasks.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/tasks.c -o CMakeFiles/freertos.dir/tasks.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/event_groups.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/event_groups.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c > CMakeFiles/freertos.dir/event_groups.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/event_groups.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c -o CMakeFiles/freertos.dir/event_groups.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/list.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/list.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/list.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/list.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/list.c > CMakeFiles/freertos.dir/list.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/list.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/list.c -o CMakeFiles/freertos.dir/list.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/queue.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/queue.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/queue.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/queue.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/queue.c > CMakeFiles/freertos.dir/queue.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/queue.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/queue.c -o CMakeFiles/freertos.dir/queue.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/timers.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/timers.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/timers.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/timers.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/timers.c > CMakeFiles/freertos.dir/timers.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/timers.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/timers.c -o CMakeFiles/freertos.dir/timers.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c > CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c -o CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c > CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c -o CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/flags.make
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj: ../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liangzhu/stm32/test/build/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj   -c /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.i"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c > CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.i

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.s"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && /usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c -o CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.s

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj.requires:
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj.requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj.provides: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj.requires
	$(MAKE) -f Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj.provides.build
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj.provides

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj.provides.build: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj

# Object files for target freertos
freertos_OBJECTS = \
"CMakeFiles/freertos.dir/croutine.c.obj" \
"CMakeFiles/freertos.dir/tasks.c.obj" \
"CMakeFiles/freertos.dir/event_groups.c.obj" \
"CMakeFiles/freertos.dir/list.c.obj" \
"CMakeFiles/freertos.dir/queue.c.obj" \
"CMakeFiles/freertos.dir/timers.c.obj" \
"CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj" \
"CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj" \
"CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj"

# External object files for target freertos
freertos_EXTERNAL_OBJECTS =

Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build.make
Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C static library libfreertos.a"
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && $(CMAKE_COMMAND) -P CMakeFiles/freertos.dir/cmake_clean_target.cmake
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/freertos.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build: Middlewares/Third_Party/FreeRTOS/Source/libfreertos.a
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/build

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/croutine.c.obj.requires
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/tasks.c.obj.requires
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/event_groups.c.obj.requires
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/list.c.obj.requires
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/queue.c.obj.requires
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/timers.c.obj.requires
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/CMSIS_RTOS/cmsis_os.c.obj.requires
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/GCC/ARM_CM4F/port.c.obj.requires
Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires: Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/portable/MemMang/heap_4.c.obj.requires
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/requires

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/clean:
	cd /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source && $(CMAKE_COMMAND) -P CMakeFiles/freertos.dir/cmake_clean.cmake
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/clean

Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/depend:
	cd /home/liangzhu/stm32/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liangzhu/stm32/test /home/liangzhu/stm32/test/Middlewares/Third_Party/FreeRTOS/Source /home/liangzhu/stm32/test/build /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source /home/liangzhu/stm32/test/build/Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Middlewares/Third_Party/FreeRTOS/Source/CMakeFiles/freertos.dir/depend

