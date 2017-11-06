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

project(opendlv-proxy-lynx)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake.Modules" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${OPENDAVINCI_DIR}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${ODVDOPENDLVSTANDARDMESSAGESET_DIR}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})

include(CompileFlags)
include(CheckCxxTestEnvironment)

find_package(OpenCV REQUIRED)
find_package(Wt REQUIRED)

find_package(OpenDaVINCI REQUIRED)
find_package(ODVDOpenDLVStandardMessageSet REQUIRED)
find_package(ODVDcfsd18 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(SYSTEM ${OPENDAVINCI_INCLUDE_DIRS})
include_directories(SYSTEM ${ODVDOPENDLVSTANDARDMESSAGESET_INCLUDE_DIRS})
include_directories(SYSTEM ${ODVDCFSD18_INCLUDE_DIRS})

set(LIBRARIES 
  ${OPENDAVINCI_LIBRARIES}
  ${Wt_LIBRARY} ${Wt_HTTP_LIBRARY} ${Wt_EXT_LIBRARY}  
  ${ODVDOPENDLVSTANDARDMESSAGESET_LIBRARIES}
  ${ODVDCFSD18_LIBRARIES})

### MICROSERVICE BEGIN ###
add_subdirectory(brake)
add_subdirectory(can)
add_subdirectory(steering)
### MICROSERVICE END ###

set(CHANGELOG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../ChangeLog")
include(CreatePackages)
