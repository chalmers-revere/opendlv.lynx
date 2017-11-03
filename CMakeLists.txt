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

project(opendlv.mantis)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake.Modules" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${OPENDAVINCI_DIR}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules" ${CMAKE_MODULE_PATH})

include(ExternalProject)

set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})

if(NOT("${DPKG_EXECUTABLE}" STREQUAL "DPKG_EXECUTABLE-NOTFOUND")
    AND NOT("${RPMBUILD_EXECUTABLE}" STREQUAL "RPMBUILD_EXECUTABLE-NOTFOUND")
    AND ("${PACKAGING_ENABLED}" STREQUAL "YES") )
  message(STATUS "Enabling building .deb and .rpm packages.")
  set(MAKE_PACKAGE package)
else()
  # If the platform does not provide dpkg and rpmbuild, make package is
  # replaced with make help to simply have a no-op.
  set(MAKE_PACKAGE help)
endif()

### NAMESPACE BEGIN ###
### NAMESPACE END ###
