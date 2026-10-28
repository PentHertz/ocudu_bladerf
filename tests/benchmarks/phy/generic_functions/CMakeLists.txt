# SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
# SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI
# Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.


set_directory_properties(PROPERTIES LABELS "phy")

add_executable(dft_processor_benchmark dft_processor_benchmark.cpp)
target_link_libraries(dft_processor_benchmark ocudulog ocudu_dft)

add_test(dft_processor_generic_benchmark dft_processor_benchmark -D generic -R 10 -s)
if (ENABLE_FFTW AND FFTW3F_FOUND)
    add_test(dft_processor_fftw_benchmark dft_processor_benchmark -D fftw -R 10 -s)
endif (ENABLE_FFTW AND FFTW3F_FOUND)
if (ENABLE_FFTZ AND FFTZ_FOUND)
    add_test(dft_processor_fftz_benchmark dft_processor_benchmark -D fftz -R 10 -s)
endif (ENABLE_FFTZ AND FFTZ_FOUND)
if (ENABLE_HWY AND HWY_FOUND)
    add_test(dft_processor_hwy_benchmark dft_processor_benchmark -D hwy:auto -R 10 -s)
endif (ENABLE_HWY AND HWY_FOUND)
