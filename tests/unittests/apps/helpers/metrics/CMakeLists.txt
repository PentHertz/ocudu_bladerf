# SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
# SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI

add_executable(cu_cp_json_helper_test cu_cp_json_helper_test.cpp)
target_include_directories(cu_cp_json_helper_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(cu_cp_json_helper_test PRIVATE
        ocudu_cu_cp_metrics_json_generators_helpers
        ocudu_support
        GTest::gtest
        GTest::gtest_main)
gtest_discover_tests(cu_cp_json_helper_test)
