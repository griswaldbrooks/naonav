cmake_minimum_required(VERSION 2.8)
project(naonav)

set(BOOST_DIR "/usr/include/")
# find_package( Boost 1.40 )
find_package(qibuild)

# Set project include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Bring Robotlib into the fold
set(ROBOTLIB_DIR /home/crrl-user1/code_projects/robotlib)
include_directories("${ROBOTLIB_DIR}/include")
link_directories("${ROBOTLIB_DIR}/lib")

# Get Boost
set(ROBOTLIB_BOOSTLIBS boost_system boost_thread pthread)

include_directories( ${Boost_INCLUDE_DIR} )

# Set variable for the sources to compile
set(_srcs
	main.cpp
	src/godzila.cpp
	src/naopp.cpp
)

# Create a executable named naonav
qi_create_bin(naonav ${_srcs})

# Add libraries
target_link_libraries(naonav ROBOTLIB ${ROBOTLIB_BOOSTLIBS})

# Qibuild Dependencies
qi_use_lib(naonav ALCOMMON ALPROXIES)

# Add a simple test:
enable_testing()
qi_create_test(test_naonav "test.cpp")