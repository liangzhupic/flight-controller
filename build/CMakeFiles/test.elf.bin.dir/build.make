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

# Utility rule file for test.elf.bin.

# Include the progress variables for this target.
include CMakeFiles/test.elf.bin.dir/progress.make

CMakeFiles/test.elf.bin: test.elf
	/home/liangzhu/compiler-arm-none-eabi/bin/arm-none-eabi-objcopy -Obinary test.elf test.elf.bin

test.elf.bin: CMakeFiles/test.elf.bin
test.elf.bin: CMakeFiles/test.elf.bin.dir/build.make
.PHONY : test.elf.bin

# Rule to build all files generated by this target.
CMakeFiles/test.elf.bin.dir/build: test.elf.bin
.PHONY : CMakeFiles/test.elf.bin.dir/build

CMakeFiles/test.elf.bin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.elf.bin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.elf.bin.dir/clean

CMakeFiles/test.elf.bin.dir/depend:
	cd /home/liangzhu/stm32/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liangzhu/stm32/test /home/liangzhu/stm32/test /home/liangzhu/stm32/test/build /home/liangzhu/stm32/test/build /home/liangzhu/stm32/test/build/CMakeFiles/test.elf.bin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.elf.bin.dir/depend

