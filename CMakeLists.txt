cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# The OpenIGTLink include file directories.
SET(OpenIGTLink_INCLUDE_DIRS "/home/tobias/fuerte_workspace/OpenIGTLink/Source;/home/tobias/fuerte_workspace/OpenIGTLink/Source/igtlutil;/home/tobias/fuerte_workspace/OpenIGTLink-build")
 #The OpenIGTLink library directories.
SET(OpenIGTLink_LIBRARY_DIRS "/home/tobias/fuerte_workspace/OpenIGTLink-build/bin")
SET(OpenIGTLink_USE_FILE "/home/tobias/fuerte_workspace/OpenIGTLink-build/UseOpenIGTLink.cmake")

# OpenIGTLink directories
#find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})
include_directories(${OpenIGTLink_INCLUDE_DIRS})

# VTK directories ---------------------------------
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(${VTK_INCLUDE_DIRS})

set(INCLUDE_FILES  
		    	include/igtlnode.h
			include/IGTL_Server.h
                    )

set(SOURCE_FILES    src/igtlnode.cpp
		    src/main.cpp
		    src/IGTL_Server.cpp
		    
                    )

rosbuild_init()
rosbuild_add_boost_directories()

rosbuild_add_executable(igtlnode ${INCLUDE_FILES}  ${SOURCE_FILES})
rosbuild_link_boost(${PROJECT_NAME} system thread)
target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES})
target_link_libraries(igtlnode ${OpenIGTLink_LIBRARY_DIRS}/libOpenIGTLink.a)
target_link_libraries(igtlnode ${OpenIGTLink_LIBRARY_DIRS}/libigtlutil.a)
