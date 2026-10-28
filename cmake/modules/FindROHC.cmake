# SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
# SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI


# - Try to find ROHC library
#
# Once done this will define
#  ROHC_FOUND        - System has ROHC library installed
#  ROHC_INCLUDE_DIRS - The ROHC include directories
#  ROHC_LIBRARIES    - The ROHC library

find_package(PkgConfig REQUIRED)

# Let pkg-config also look in ROHC_DIR, it provides the version reported in the SBOM
set(_rohc_prefix_path ${CMAKE_PREFIX_PATH})
if (DEFINED ENV{ROHC_DIR})
    file(TO_CMAKE_PATH "$ENV{ROHC_DIR}" _rohc_dir)
    list(APPEND CMAKE_PREFIX_PATH "${_rohc_dir}")
endif (DEFINED ENV{ROHC_DIR})
pkg_check_modules(PC_ROHC rohc>=2.3.1)
set(CMAKE_PREFIX_PATH ${_rohc_prefix_path})
unset(_rohc_prefix_path)
unset(_rohc_dir)

if(NOT "$ENV{ROHC_DIR}" STREQUAL "")
  set(_ROHC_INCLUDE_HINTS "$ENV{ROHC_DIR}/include")
  set(_ROHC_LIB_HINTS     "$ENV{ROHC_DIR}/lib")
endif()

find_path(
    ROHC_INCLUDE_DIRS
    NAMES rohc/rohc.h
    HINTS ${_ROHC_INCLUDE_HINTS}
          ${PC_ROHC_INCLUDEDIR}
          ${CMAKE_INSTALL_PREFIX}/include
    PATHS /usr/local/include
          /usr/include
)

find_library(
    ROHC_LIBRARIES
    NAMES rohc
    HINTS ${_ROHC_LIB_HINTS}
          ${PC_ROHC_LIBDIR}
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

unset(_ROHC_INCLUDE_HINTS)
unset(_ROHC_LIB_HINTS)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ROHC DEFAULT_MSG ROHC_LIBRARIES ROHC_INCLUDE_DIRS)
mark_as_advanced(ROHC_LIBRARIES ROHC_INCLUDE_DIRS)

if(ROHC_FOUND)
  message(STATUS "ROHC LIBRARIES: " ${ROHC_LIBRARIES})
  message(STATUS "ROHC INCLUDE DIRS: " ${ROHC_INCLUDE_DIRS})
endif()
