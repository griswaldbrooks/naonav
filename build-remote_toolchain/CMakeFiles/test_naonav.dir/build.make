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
CMAKE_SOURCE_DIR = /home/crrl-user1/code_projects/nao_worktree/naonav

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/crrl-user1/code_projects/nao_worktree/naonav/build-remote_toolchain

# Include any dependencies generated for this target.
include CMakeFiles/test_naonav.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_naonav.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_naonav.dir/flags.make

CMakeFiles/test_naonav.dir/test.cpp.o: CMakeFiles/test_naonav.dir/flags.make
CMakeFiles/test_naonav.dir/test.cpp.o: ../test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/crrl-user1/code_projects/nao_worktree/naonav/build-remote_toolchain/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_naonav.dir/test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_naonav.dir/test.cpp.o -c /home/crrl-user1/code_projects/nao_worktree/naonav/test.cpp

CMakeFiles/test_naonav.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_naonav.dir/test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/crrl-user1/code_projects/nao_worktree/naonav/test.cpp > CMakeFiles/test_naonav.dir/test.cpp.i

CMakeFiles/test_naonav.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_naonav.dir/test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/crrl-user1/code_projects/nao_worktree/naonav/test.cpp -o CMakeFiles/test_naonav.dir/test.cpp.s

CMakeFiles/test_naonav.dir/test.cpp.o.requires:
.PHONY : CMakeFiles/test_naonav.dir/test.cpp.o.requires

CMakeFiles/test_naonav.dir/test.cpp.o.provides: CMakeFiles/test_naonav.dir/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_naonav.dir/build.make CMakeFiles/test_naonav.dir/test.cpp.o.provides.build
.PHONY : CMakeFiles/test_naonav.dir/test.cpp.o.provides

CMakeFiles/test_naonav.dir/test.cpp.o.provides.build: CMakeFiles/test_naonav.dir/test.cpp.o

# Object files for target test_naonav
test_naonav_OBJECTS = \
"CMakeFiles/test_naonav.dir/test.cpp.o"

# External object files for target test_naonav
test_naonav_EXTERNAL_OBJECTS =

sdk/bin/test_naonav: CMakeFiles/test_naonav.dir/test.cpp.o
sdk/bin/test_naonav: CMakeFiles/test_naonav.dir/build.make
sdk/bin/test_naonav: CMakeFiles/test_naonav.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable sdk/bin/test_naonav"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_naonav.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_naonav.dir/build: sdk/bin/test_naonav
.PHONY : CMakeFiles/test_naonav.dir/build

# Object files for target test_naonav
test_naonav_OBJECTS = \
"CMakeFiles/test_naonav.dir/test.cpp.o"

# External object files for target test_naonav
test_naonav_EXTERNAL_OBJECTS =

CMakeFiles/CMakeRelink.dir/test_naonav: CMakeFiles/test_naonav.dir/test.cpp.o
CMakeFiles/CMakeRelink.dir/test_naonav: CMakeFiles/test_naonav.dir/build.make
CMakeFiles/CMakeRelink.dir/test_naonav: CMakeFiles/test_naonav.dir/relink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable CMakeFiles/CMakeRelink.dir/test_naonav"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_naonav.dir/relink.txt --verbose=$(VERBOSE)

# Rule to relink during preinstall.
CMakeFiles/test_naonav.dir/preinstall: CMakeFiles/CMakeRelink.dir/test_naonav
.PHONY : CMakeFiles/test_naonav.dir/preinstall

CMakeFiles/test_naonav.dir/requires: CMakeFiles/test_naonav.dir/test.cpp.o.requires
.PHONY : CMakeFiles/test_naonav.dir/requires

CMakeFiles/test_naonav.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_naonav.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_naonav.dir/clean

CMakeFiles/test_naonav.dir/depend:
	cd /home/crrl-user1/code_projects/nao_worktree/naonav/build-remote_toolchain && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/crrl-user1/code_projects/nao_worktree/naonav /home/crrl-user1/code_projects/nao_worktree/naonav /home/crrl-user1/code_projects/nao_worktree/naonav/build-remote_toolchain /home/crrl-user1/code_projects/nao_worktree/naonav/build-remote_toolchain /home/crrl-user1/code_projects/nao_worktree/naonav/build-remote_toolchain/CMakeFiles/test_naonav.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_naonav.dir/depend

