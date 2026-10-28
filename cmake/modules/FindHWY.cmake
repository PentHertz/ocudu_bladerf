# SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
# SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI


# Try to find the Google Highway SIMD library.
# Once done this will define
#  HWY_FOUND - System has Highway
#  HWY_INCLUDE_DIRS - The Highway include directories
#  HWY_LIBRARIES - The libraries needed to use Highway
#  HWY_DEFINITIONS - Compiler switches required for using Highway
#  HWY_VERSION - The Highway version, if pkg-config knows it

find_package(PkgConfig QUIET)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(PC_HWY QUIET "libhwy >= 1.0")
endif (PKG_CONFIG_FOUND)
set(HWY_DEFINITIONS ${PC_HWY_CFLAGS_OTHER})
set(HWY_VERSION ${PC_HWY_VERSION})

if(NOT "$ENV{HWY_DIR}" STREQUAL "")
  set(_HWY_INCLUDE_HINTS "$ENV{HWY_DIR}/include")
  set(_HWY_LIB_HINTS     "$ENV{HWY_DIR}/lib")
endif()

find_path(HWY_INCLUDE_DIR
        NAMES hwy/highway.h
        HINTS ${PC_HWY_INCLUDEDIR} ${PC_HWY_INCLUDE_DIRS} ${_HWY_INCLUDE_HINTS}
        PATHS /usr/local/include
              /usr/include)

find_library(HWY_LIBRARY
        NAMES hwy libhwy
        HINTS ${PC_HWY_LIBDIR} ${PC_HWY_LIBRARY_DIRS} ${_HWY_LIB_HINTS}
        PATHS /usr/local/lib
              /usr/lib)

unset(_HWY_INCLUDE_HINTS)
unset(_HWY_LIB_HINTS)

set(HWY_LIBRARIES ${HWY_LIBRARY})
set(HWY_INCLUDE_DIRS ${HWY_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)

# Handle the QUIETLY and REQUIRED arguments and set HWY_FOUND to TRUE
# if all listed variables are TRUE.
find_package_handle_standard_args(HWY DEFAULT_MSG HWY_LIBRARY HWY_INCLUDE_DIR)

if (HWY_FOUND)
    MESSAGE(STATUS "Found Highway INCLUDE DIRS: ${HWY_INCLUDE_DIRS}")
    MESSAGE(STATUS "Found Highway LIBRARIES: ${HWY_LIBRARIES}")
endif (HWY_FOUND)

mark_as_advanced(HWY_INCLUDE_DIR HWY_LIBRARY)
