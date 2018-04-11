# Copyright (C) 2017 Chalmers Revere
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

cmake_minimum_required(VERSION 2.8)

project(opendlv-proxy-lynx-can)

# Set the search path for .cmake files.
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake.Modules" ${CMAKE_MODULE_PATH})

# Add a local CMake module search path dependent on the desired installation destination.
# Thus, artifacts from the complete source build can be given precendence over any installed versions.
IF(UNIX)
    SET (CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})
ENDIF()
IF(WIN32)
    SET (CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/CMake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})
ENDIF()

###########################################################################
# Include flags for compiling.
INCLUDE (CompileFlags)

###########################################################################
# Find and configure CxxTest.
INCLUDE (CheckCxxTestEnvironment)

###########################################################################
# Find OpenDaVINCI.
FIND_PACKAGE (OpenDaVINCI REQUIRED)

###########################################################################
# Find ODVDOpenDLVData.
#set(CMAKE_MODULE_PATH "${ODVDOPENDLVDATA_DIR}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})
#find_package(ODVDOpenDLVData REQUIRED)

###########################################################################
# Find ODVDcfsd18.
find_package(ODVDcfsd18 REQUIRED)

###########################################################################
# Find librhinogw.
find_package(cfsd18gw REQUIRED)

###########################################################################
# Find AutomotiveData.
set(AUTOMOTIVEDATA_DIR "${OPENDAVINCI_DIR}")
find_package(AutomotiveData REQUIRED)

###########################################################################
# Find odcantools.
set(ODCANTOOLS_DIR "${OPENDAVINCI_DIR}")
find_package(odcantools REQUIRED)

###############################################################################
# Set header files from OpenDaVINCI.
INCLUDE_DIRECTORIES (SYSTEM ${OPENDAVINCI_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES (SYSTEM ${ODVDOPENDLVDATA_INCLUDE_DIRS})
INCLUDE_DIRECTORIES (SYSTEM ${ODCANTOOLS_INCLUDE_DIRS})
INCLUDE_DIRECTORIES (SYSTEM ${AUTOMOTIVEDATA_INCLUDE_DIRS})

INCLUDE_DIRECTORIES (SYSTEM ${ODVDCFSD18_INCLUDE_DIRS})
INCLUDE_DIRECTORIES (SYSTEM ${CFSD18GW_INCLUDE_DIRS})


# Set header files from OpenDLV.


# Set include directory.
INCLUDE_DIRECTORIES(include)

# Set libraries to link against.
set(LIBRARIES ${LIBRARIES}
  ${ODCANTOOLS_LIBRARIES}
  ${CFSD18GW_LIBRARIES}
  )





file(GLOB_RECURSE SOURCEFILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(${PROJECT_NAME}-static STATIC ${SOURCEFILES})
add_executable(${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/app/${PROJECT_NAME}.cpp")
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}-static ${LIBRARIES}) 

include(RunTests)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin COMPONENT ${CMAKE_PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}-static DESTINATION lib COMPONENT ${CMAKE_PROJECT_NAME})
install(FILES man/${PROJECT_NAME}.1 DESTINATION man/man1 COMPONENT ${CMAKE_PROJECT_NAME})
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION include/${CMAKE_PROJECT_NAME} COMPONENT ${CMAKE_PROJECT_NAME})
