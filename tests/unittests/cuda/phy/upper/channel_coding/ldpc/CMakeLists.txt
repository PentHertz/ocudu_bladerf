# SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI
# Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

add_executable(ldpc_decoder_kernel_test ldpc_decoder_kernel_test.cpp)
target_link_libraries(ldpc_decoder_kernel_test
        ocudu_cuda_ldpc
        ocudu_cuda_adt
        ocudu_channel_coding
        ocudulog
        GTest::gtest
        GTest::gtest_main)
add_cuda_test(ldpc_decoder_kernel_test ldpc_decoder_kernel_test)

add_executable(ldpc_encoder_kernel_test ldpc_encoder_kernel_test.cpp)
target_link_libraries(ldpc_encoder_kernel_test
        ocudu_cuda_ldpc
        ocudu_cuda_adt
        ocudu_channel_coding
        ocudulog
        GTest::gtest
        GTest::gtest_main)
add_cuda_test(ldpc_encoder_kernel_test ldpc_encoder_kernel_test)

add_executable(ldpc_rate_matching_kernel_test ldpc_rate_matching_kernel_test.cpp)
target_link_libraries(ldpc_rate_matching_kernel_test
        ocudu_cuda_ldpc
        ocudu_cuda_adt
        ocudu_channel_coding
        ocudulog
        GTest::gtest
        GTest::gtest_main)
add_cuda_test(ldpc_rate_matching_kernel_test ldpc_rate_matching_kernel_test)
