# SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
# SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI
# Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.


add_subdirectory(precoding)
add_subdirectory(transform_precoding)

# Initialise DFT sources and definitions for DFT libraries.
set(OCUDU_DFT_SOURCES dft_processor_generic_impl.cpp generic_functions_factories.cpp)
set(OCUDU_DFT_LIBRARIES "")
set(OCUDU_DFT_INCLUDE_DIRS "")
set(OCUDU_DFT_LIBRARY_DIRS "")
set(OCUDU_DFT_DEFINITIONS "")

if (ENABLE_MKL AND MKL_FOUND)
    # If MKL is enabled and found, then compile DFT FFTW API based and link it to MKL.
    list(APPEND OCUDU_DFT_LIBRARIES ${MKL_LIBRARIES})
    list(APPEND OCUDU_DFT_INCLUDE_DIRS ${MKL_INCLUDE_DIRS})
    list(APPEND OCUDU_DFT_LIBRARY_DIRS ${MKL_LIBRARY_DIRS})
elseif (ENABLE_ARMPL AND ARMPL_FOUND)
    # If ARMPL is enabled and found, then add FFTW source and libraries.
    list(APPEND OCUDU_DFT_LIBRARIES ${ARMPL_LIBRARIES})
    list(APPEND OCUDU_DFT_INCLUDE_DIRS ${ARMPL_INCLUDE_DIRS})
    list(APPEND OCUDU_DFT_LIBRARY_DIRS ${ARMPL_LIBRARY_DIRS})
elseif (ENABLE_FFTW AND FFTW3F_FOUND)
    # If FFTW is enabled and found, then add FFTW source and libraries.
    list(APPEND OCUDU_DFT_LIBRARIES ${FFTW3F_LIBRARIES})
    list(APPEND OCUDU_DFT_INCLUDE_DIRS ${FFTW3F_INCLUDE_DIRS})
    list(APPEND OCUDU_DFT_LIBRARY_DIRS ${FFTW3F_LIBRARY_DIRS})
endif (ENABLE_MKL AND MKL_FOUND)

if (ENABLE_FFTZ AND FFTZ_FOUND)
    # If FFTZ is enabled and found, then add FFTZ source and libraries.
    list(APPEND OCUDU_DFT_LIBRARIES ${FFTZ_LIBRARIES})
    list(APPEND OCUDU_DFT_INCLUDE_DIRS ${FFTZ_INCLUDE_DIRS})
    list(APPEND OCUDU_DFT_SOURCES dft_processor_fftz_impl.cpp)
    list(APPEND OCUDU_DFT_DEFINITIONS "HAVE_FFTZ")
endif (ENABLE_FFTZ AND FFTZ_FOUND)

if ((ENABLE_MKL AND MKL_FOUND) OR (ENABLE_ARMPL AND ARMPL_FOUND) OR (ENABLE_FFTW AND FFTW3F_FOUND))
    list(APPEND OCUDU_DFT_SOURCES dft_processor_fftw_impl.cpp)
    list(APPEND OCUDU_DFT_DEFINITIONS "HAVE_FFTW")
endif ((ENABLE_MKL AND MKL_FOUND) OR (ENABLE_ARMPL AND ARMPL_FOUND) OR (ENABLE_FFTW AND FFTW3F_FOUND))

if (ENABLE_HWY AND HWY_FOUND)
    # If Highway is enabled and found, then add the Highway DFT sources, in dft_hwy, and libraries. The passes of each
    # Highway target are in their own source file, so they compile in parallel.
    list(APPEND OCUDU_DFT_LIBRARIES ${HWY_LIBRARIES})
    list(APPEND OCUDU_DFT_DEFINITIONS "HAVE_HWY" ${HWY_DEFINITIONS})

    # The FMA contraction would make the targets give different results.
    set(OCUDU_DFT_HWY_OPTIONS "-ffp-contract=off")
    # The stack instrumentation of the address sanitizer makes the compilation of the unrolled passes take more than an
    # hour, so only the heap accesses are checked.
    if (ENABLE_ASAN AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
        list(APPEND OCUDU_DFT_HWY_OPTIONS "--param=asan-stack=0")
    elseif (ENABLE_ASAN AND (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
        list(APPEND OCUDU_DFT_HWY_OPTIONS "-mllvm" "-asan-stack=0")
    endif ()

    # The plans, the tables and the selection of the target use the project compiler options.
    list(APPEND OCUDU_DFT_SOURCES dft_hwy/dft_processor_hwy_impl.cpp)
    set_source_files_properties(dft_hwy/dft_processor_hwy_impl.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")

    if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
        # NEON, with the generic Armv8-A baseline.
        list(APPEND OCUDU_DFT_SOURCES dft_hwy/dft_processor_hwy_neon.cpp)
        set_source_files_properties(dft_hwy/dft_processor_hwy_neon.cpp PROPERTIES COMPILE_OPTIONS
                "-mcpu=generic;${OCUDU_DFT_HWY_OPTIONS}")
    else (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
        # AVX2 and AVX-512, each file selects its Highway target.
        set(OCUDU_DFT_HWY_X86_SOURCES dft_hwy/dft_processor_hwy_avx2.cpp dft_hwy/dft_processor_hwy_avx512.cpp)
        list(APPEND OCUDU_DFT_SOURCES ${OCUDU_DFT_HWY_X86_SOURCES})
        # The -march=x86-64 baseline replaces the -march of the project, which is native by default, for two reasons:
        # - Highway adds the instructions of the target of the file to the baseline, it does not remove any. With a
        #   native baseline on an AVX-512 processor, the AVX2 passes would contain AVX-512 instructions, and would crash
        #   on the AVX2 processors that select them.
        # - The files also compile inline functions and templates outside the passes, such as those of the standard
        #   library. The linker keeps a single copy of each for the whole program, which may come from any of these
        #   files, so it must not contain the instructions of any Highway target.
        set_source_files_properties(${OCUDU_DFT_HWY_X86_SOURCES} PROPERTIES COMPILE_OPTIONS
                "-march=x86-64;${OCUDU_DFT_HWY_OPTIONS}")
    endif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    # The native target, the static target of the project compiler options, keeps the project -march. Its inline code
    # has the same instructions as the rest of the program.
    list(APPEND OCUDU_DFT_SOURCES dft_hwy/dft_processor_hwy_native.cpp)
    set_source_files_properties(dft_hwy/dft_processor_hwy_native.cpp PROPERTIES COMPILE_OPTIONS
            "${OCUDU_DFT_HWY_OPTIONS}")
endif (ENABLE_HWY AND HWY_FOUND)

add_library(ocudu_dft STATIC ${OCUDU_DFT_SOURCES})
target_include_directories(ocudu_dft PRIVATE ${OCUDU_DFT_INCLUDE_DIRS})
if (ENABLE_HWY AND HWY_FOUND)
    # The Highway headers are system headers, so the warnings of the project do not apply to them.
    target_include_directories(ocudu_dft SYSTEM PRIVATE ${HWY_INCLUDE_DIRS})
endif (ENABLE_HWY AND HWY_FOUND)
target_link_directories(ocudu_dft PRIVATE ${OCUDU_DFT_LIBRARY_DIRS})
target_link_libraries(ocudu_dft PRIVATE ${OCUDU_DFT_LIBRARIES} ocuduvec ocudulog)
target_compile_definitions(ocudu_dft PRIVATE ${OCUDU_DFT_DEFINITIONS})

add_to_exported_libs(ocudu_dft)
