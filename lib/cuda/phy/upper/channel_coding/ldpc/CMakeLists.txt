# SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI
# Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

add_library(ocudu_cuda_ldpc ldpc_decoder.cu ldpc_encoder.cu ldpc_rate_matching.cu)

target_link_libraries(ocudu_cuda_ldpc ocudu_cuda_adt ocudu_support CUDA::cudart)

add_to_exported_libs(ocudu_cuda_ldpc)
