// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief 5G NR modulation mapping and soft demodulation on the GPU.
///
/// The constellations and the Gray mapping follow TS 38.211, Section 5.1. The soft demodulator
/// uses the same piecewise linear approximation of the log likelihood ratio as the software
/// demodulation mapper of the library, so the two agree to within the approximation itself.

#include "ocudu/cuda/adt/cuda_error.h"
#include "ocudu/cuda/phy/upper/channel_modulation/modulation.h"
#include <cuda_fp16.h>

namespace ocudu {
namespace cuda {

// ============================================================================
// Device Constants - Constellation Points (Gray coded, normalized)
// ============================================================================

// QPSK constellation: 1/sqrt(2), Gray coded
// bits = b1b0 (LSB first): b0 determines I sign, b1 determines Q sign
// b0=0 -> I+, b0=1 -> I-, b1=0 -> Q+, b1=1 -> Q-
__device__ __constant__ float QPSK_I[4] = {0.7071068f, -0.7071068f, 0.7071068f, -0.7071068f}; ///< bits: 00, 01, 10, 11
__device__ __constant__ float QPSK_Q[4] = {0.7071068f, 0.7071068f, -0.7071068f, -0.7071068f};

// 16QAM constellation (normalized by 1/sqrt(10))
// Matches cuPHY: I uses bits 0,2; Q uses bits 1,3
// Table indexed by (bit0) | (bit2 << 1) for I
// Table indexed by (bit1) | (bit3 << 1) for Q
__device__ __constant__ float QAM16_TABLE[8] = {
    0.316227766f,  // index 0: +1
    -0.316227766f, // index 1: -1
    0.316227766f,  // index 2: +1 (same as 0, placeholder)
    -0.316227766f, // index 3: -1 (same as 1, placeholder)
    0.948683298f,  // index 4: +3
    -0.948683298f, // index 5: -3
    0.948683298f,  // index 6: +3 (same as 4, placeholder)
    -0.948683298f  // index 7: -3 (same as 5, placeholder)
};

// 64QAM constellation table (normalized by 1/sqrt(42))
// Matches cuPHY: I uses bits 0,2,4; Q uses bits 1,3,5
// Table indexed by (bit0) | (bit2 >> 1) | (bit4 >> 2)
__device__ __constant__ float QAM64_TABLE[8] = {
    0.462910049886276f,  // index 0: +3
    -0.462910049886276f, // index 1: -3
    0.77151674981046f,   // index 2: +5
    -0.77151674981046f,  // index 3: -5
    0.154303349962092f,  // index 4: +1
    -0.154303349962092f, // index 5: -1
    1.08012344973464f,   // index 6: +7
    -1.08012344973464f   // index 7: -7
};

// 256QAM constellation table (normalized by 1/sqrt(170))
// Matches cuPHY: I uses bits 0,2,4,6; Q uses bits 1,3,5,7
__device__ __constant__ float QAM256_TABLE[16] = {
    0.383482494f,  // index 0
    -0.383482494f, // index 1
    0.843661488f,  // index 2
    -0.843661488f, // index 3
    0.230089497f,  // index 4
    -0.230089497f, // index 5
    0.997054486f,  // index 6
    -0.997054486f, // index 7
    0.536875492f,  // index 8
    -0.536875492f, // index 9
    0.69026849f,   // index 10
    -0.69026849f,  // index 11
    0.076696499f,  // index 12
    -0.076696499f, // index 13
    1.150447483f,  // index 14
    -1.150447483f  // index 15
};

// ============================================================================
// INT8 Constellation Tables (unnormalized, for direct INT8 output)
// ============================================================================

// QPSK INT8: ±1 (unnormalized)
__device__ __constant__ int8_t QPSK_I8[4] = {1, -1, 1, -1};
__device__ __constant__ int8_t QPSK_Q8[4] = {1, 1, -1, -1};

// 16QAM INT8: ±1, ±3 (unnormalized)
__device__ __constant__ int8_t QAM16_I8[4] = {1, -1, 3, -3};
__device__ __constant__ int8_t QAM16_Q8[4] = {1, -1, 3, -3};

// 64QAM INT8: ±1, ±3, ±5, ±7 (unnormalized)
// Indexed by (bit0) | (bit2 >> 1) | (bit4 >> 2)
__device__ __constant__ int8_t QAM64_I8[8] = {3, -3, 5, -5, 1, -1, 7, -7};

// 256QAM INT8: ±1, ±3, ±5, ±7, ±9, ±11, ±13, ±15 (unnormalized)
// Indexed by (bit0) | (bit2 >> 1) | (bit4 >> 2) | (bit6 >> 3)
__device__ __constant__ int8_t QAM256_I8[16] = {5, -5, 11, -11, 3, -3, 13, -13, 7, -7, 9, -9, 1, -1, 15, -15};

// ============================================================================
// CPU-Compatible Interval-Based Soft Demodulation Constants
// These match the srsRAN CPU implementation exactly for numerical compatibility
// ============================================================================

// 16QAM Constants (from demodulation_mapper_qam16.cpp)
// Square root of 1/10
#define M_SQRT1_10 0.316227766f // 1/sqrt(10)

// 64QAM Constants (from demodulation_mapper_qam64.cpp)
// Square root of 1/42
#define M_SQRT1_42 0.154303350f // 1/sqrt(42)

// 64QAM Interval parameters
__device__ __constant__ float SLOPE_64QAM_01[8] = {16.0f * M_SQRT1_42,
                                                   12.0f * M_SQRT1_42,
                                                   8.0f * M_SQRT1_42,
                                                   4.0f * M_SQRT1_42,
                                                   4.0f * M_SQRT1_42,
                                                   8.0f * M_SQRT1_42,
                                                   12.0f * M_SQRT1_42,
                                                   16.0f * M_SQRT1_42};
__device__ __constant__ float INTERCEPT_64QAM_01[8] =
    {24.0f / 21.0f, 12.0f / 21.0f, 4.0f / 21.0f, 0.0f, 0.0f, -4.0f / 21.0f, -12.0f / 21.0f, -24.0f / 21.0f};

__device__ __constant__ float SLOPE_64QAM_23[8]     = {8.0f * M_SQRT1_42,
                                                       4.0f * M_SQRT1_42,
                                                       4.0f * M_SQRT1_42,
                                                       8.0f * M_SQRT1_42,
                                                       -8.0f * M_SQRT1_42,
                                                       -4.0f * M_SQRT1_42,
                                                       -4.0f * M_SQRT1_42,
                                                       -8.0f * M_SQRT1_42};
__device__ __constant__ float INTERCEPT_64QAM_23[8] = {20.0f / 21.0f,
                                                       8.0f / 21.0f,
                                                       8.0f / 21.0f,
                                                       12.0f / 21.0f,
                                                       12.0f / 21.0f,
                                                       8.0f / 21.0f,
                                                       8.0f / 21.0f,
                                                       20.0f / 21.0f};

__device__ __constant__ float SLOPE_64QAM_45[4]     = {4.0f * M_SQRT1_42,
                                                       -4.0f * M_SQRT1_42,
                                                       4.0f * M_SQRT1_42,
                                                       -4.0f * M_SQRT1_42};
__device__ __constant__ float INTERCEPT_64QAM_45[4] = {12.0f / 21.0f, -4.0f / 21.0f, -4.0f / 21.0f, 12.0f / 21.0f};

// 256QAM Constants (from demodulation_mapper_qam256.cpp)
// Square root of 1/170
#define M_SQRT1_170 0.076696499f // 1/sqrt(170)

// 256QAM Interval parameters for bits 0,1
__device__ __constant__ float SLOPE_256QAM_01[16]     = {32.0f * M_SQRT1_170,
                                                         28.0f * M_SQRT1_170,
                                                         24.0f * M_SQRT1_170,
                                                         20.0f * M_SQRT1_170,
                                                         16.0f * M_SQRT1_170,
                                                         12.0f * M_SQRT1_170,
                                                         8.0f * M_SQRT1_170,
                                                         4.0f * M_SQRT1_170,
                                                         4.0f * M_SQRT1_170,
                                                         8.0f * M_SQRT1_170,
                                                         12.0f * M_SQRT1_170,
                                                         16.0f * M_SQRT1_170,
                                                         20.0f * M_SQRT1_170,
                                                         24.0f * M_SQRT1_170,
                                                         28.0f * M_SQRT1_170,
                                                         32.0f * M_SQRT1_170};
__device__ __constant__ float INTERCEPT_256QAM_01[16] = {112.0f / 85.0f,
                                                         84.0f / 85.0f,
                                                         60.0f / 85.0f,
                                                         40.0f / 85.0f,
                                                         24.0f / 85.0f,
                                                         12.0f / 85.0f,
                                                         4.0f / 85.0f,
                                                         0.0f,
                                                         0.0f,
                                                         -4.0f / 85.0f,
                                                         -12.0f / 85.0f,
                                                         -24.0f / 85.0f,
                                                         -40.0f / 85.0f,
                                                         -60.0f / 85.0f,
                                                         -84.0f / 85.0f,
                                                         -112.0f / 85.0f};

// 256QAM Interval parameters for bits 2,3
__device__ __constant__ float SLOPE_256QAM_23[16]     = {16.0f * M_SQRT1_170,
                                                         12.0f * M_SQRT1_170,
                                                         8.0f * M_SQRT1_170,
                                                         4.0f * M_SQRT1_170,
                                                         4.0f * M_SQRT1_170,
                                                         8.0f * M_SQRT1_170,
                                                         12.0f * M_SQRT1_170,
                                                         16.0f * M_SQRT1_170,
                                                         -16.0f * M_SQRT1_170,
                                                         -12.0f * M_SQRT1_170,
                                                         -8.0f * M_SQRT1_170,
                                                         -4.0f * M_SQRT1_170,
                                                         -4.0f * M_SQRT1_170,
                                                         -8.0f * M_SQRT1_170,
                                                         -12.0f * M_SQRT1_170,
                                                         -16.0f * M_SQRT1_170};
__device__ __constant__ float INTERCEPT_256QAM_23[16] = {88.0f / 85.0f,
                                                         60.0f / 85.0f,
                                                         36.0f / 85.0f,
                                                         16.0f / 85.0f,
                                                         16.0f / 85.0f,
                                                         28.0f / 85.0f,
                                                         36.0f / 85.0f,
                                                         40.0f / 85.0f,
                                                         40.0f / 85.0f,
                                                         36.0f / 85.0f,
                                                         28.0f / 85.0f,
                                                         16.0f / 85.0f,
                                                         16.0f / 85.0f,
                                                         36.0f / 85.0f,
                                                         60.0f / 85.0f,
                                                         88.0f / 85.0f};

// 256QAM Interval parameters for bits 4,5
__device__ __constant__ float SLOPE_256QAM_45[16]     = {8.0f * M_SQRT1_170,
                                                         4.0f * M_SQRT1_170,
                                                         4.0f * M_SQRT1_170,
                                                         8.0f * M_SQRT1_170,
                                                         -8.0f * M_SQRT1_170,
                                                         -4.0f * M_SQRT1_170,
                                                         -4.0f * M_SQRT1_170,
                                                         -8.0f * M_SQRT1_170,
                                                         8.0f * M_SQRT1_170,
                                                         4.0f * M_SQRT1_170,
                                                         4.0f * M_SQRT1_170,
                                                         8.0f * M_SQRT1_170,
                                                         -8.0f * M_SQRT1_170,
                                                         -4.0f * M_SQRT1_170,
                                                         -4.0f * M_SQRT1_170,
                                                         -8.0f * M_SQRT1_170};
__device__ __constant__ float INTERCEPT_256QAM_45[16] = {52.0f / 85.0f,
                                                         24.0f / 85.0f,
                                                         24.0f / 85.0f,
                                                         44.0f / 85.0f,
                                                         -20.0f / 85.0f,
                                                         -8.0f / 85.0f,
                                                         -8.0f / 85.0f,
                                                         -12.0f / 85.0f,
                                                         -12.0f / 85.0f,
                                                         -8.0f / 85.0f,
                                                         -8.0f / 85.0f,
                                                         -20.0f / 85.0f,
                                                         44.0f / 85.0f,
                                                         24.0f / 85.0f,
                                                         24.0f / 85.0f,
                                                         52.0f / 85.0f};

// 256QAM Interval parameters for bits 6,7
__device__ __constant__ float SLOPE_256QAM_67[8]     = {4.0f * M_SQRT1_170,
                                                        -4.0f * M_SQRT1_170,
                                                        4.0f * M_SQRT1_170,
                                                        -4.0f * M_SQRT1_170,
                                                        4.0f * M_SQRT1_170,
                                                        -4.0f * M_SQRT1_170,
                                                        4.0f * M_SQRT1_170,
                                                        -4.0f * M_SQRT1_170};
__device__ __constant__ float INTERCEPT_256QAM_67[8] = {28.0f / 85.0f,
                                                        -20.0f / 85.0f,
                                                        12.0f / 85.0f,
                                                        -4.0f / 85.0f,
                                                        -4.0f / 85.0f,
                                                        12.0f / 85.0f,
                                                        -20.0f / 85.0f,
                                                        28.0f / 85.0f};

// ============================================================================
// CPU-Compatible Interval Function Helpers
// ============================================================================

/// \brief Compute interval index from value (matches CPU compute_interval_idx)
__device__ __forceinline__ int compute_interval_idx(float value, float interval_width, int nof_intervals)
{
  int idx = static_cast<int>(floorf(value / interval_width)) + nof_intervals / 2;
  return max(0, min(nof_intervals - 1, idx));
}

/// \brief 16QAM soft demod for bits 0,1 (sign bits) - CPU compatible
/// Matches demod_16QAM_symbol_01 from demodulation_mapper_qam16.cpp
__device__ __forceinline__ float soft_demod_16qam_01_cpu(float x, float rcp_noise)
{
  // GAIN_FIRST = 4 * M_SQRT1_10
  // 1.264911...
  const float GAIN_FIRST = 4.0f * M_SQRT1_10;
  // THRESHOLD = 2 * M_SQRT1_10
  // 0.632455...
  const float THRESHOLD = 2.0f * M_SQRT1_10;
  // CONST_0_8 = 0.8
  const float CONST_0_8 = 0.8f;

  float l_value = GAIN_FIRST * x;
  if (fabsf(x) > THRESHOLD) {
    // l_value = 2 * l_value - copysign(0.8, x)
    l_value = 2.0f * l_value - copysignf(CONST_0_8, x);
  }
  return l_value * rcp_noise;
}

/// \brief 16QAM soft demod for bits 2,3 (magnitude bits) - CPU compatible
/// Matches demod_16QAM_symbol_23 from demodulation_mapper_qam16.cpp
__device__ __forceinline__ float soft_demod_16qam_23_cpu(float x, float rcp_noise)
{
  // l_value = 0.8 - 4 * M_SQRT1_10 * |x|
  const float GAIN      = 4.0f * M_SQRT1_10;
  const float CONST_0_8 = 0.8f;

  float l_value = CONST_0_8 - GAIN * fabsf(x);
  return l_value * rcp_noise;
}

/// \brief 64QAM soft demod for bits 0,1 - CPU compatible interval function
__device__ __forceinline__ float soft_demod_64qam_01_cpu(float value, float rcp_noise)
{
  int   idx     = compute_interval_idx(value, 2.0f * M_SQRT1_42, 8);
  float l_value = SLOPE_64QAM_01[idx] * value + INTERCEPT_64QAM_01[idx];
  return l_value * rcp_noise;
}

/// \brief 64QAM soft demod for bits 2,3 - CPU compatible interval function
__device__ __forceinline__ float soft_demod_64qam_23_cpu(float value, float rcp_noise)
{
  int   idx     = compute_interval_idx(value, 2.0f * M_SQRT1_42, 8);
  float l_value = SLOPE_64QAM_23[idx] * value + INTERCEPT_64QAM_23[idx];
  return l_value * rcp_noise;
}

/// \brief 64QAM soft demod for bits 4,5 - CPU compatible interval function
__device__ __forceinline__ float soft_demod_64qam_45_cpu(float value, float rcp_noise)
{
  int   idx     = compute_interval_idx(value, 4.0f * M_SQRT1_42, 4);
  float l_value = SLOPE_64QAM_45[idx] * value + INTERCEPT_64QAM_45[idx];
  return l_value * rcp_noise;
}

/// \brief 256QAM soft demod for bits 0,1 - CPU compatible interval function
__device__ __forceinline__ float soft_demod_256qam_01_cpu(float value, float rcp_noise)
{
  int   idx     = compute_interval_idx(value, 2.0f * M_SQRT1_170, 16);
  float l_value = SLOPE_256QAM_01[idx] * value + INTERCEPT_256QAM_01[idx];
  return l_value * rcp_noise;
}

/// \brief 256QAM soft demod for bits 2,3 - CPU compatible interval function
__device__ __forceinline__ float soft_demod_256qam_23_cpu(float value, float rcp_noise)
{
  int   idx     = compute_interval_idx(value, 2.0f * M_SQRT1_170, 16);
  float l_value = SLOPE_256QAM_23[idx] * value + INTERCEPT_256QAM_23[idx];
  return l_value * rcp_noise;
}

/// \brief 256QAM soft demod for bits 4,5 - CPU compatible interval function
__device__ __forceinline__ float soft_demod_256qam_45_cpu(float value, float rcp_noise)
{
  int   idx     = compute_interval_idx(value, 2.0f * M_SQRT1_170, 16);
  float l_value = SLOPE_256QAM_45[idx] * value + INTERCEPT_256QAM_45[idx];
  return l_value * rcp_noise;
}

/// \brief 256QAM soft demod for bits 6,7 - CPU compatible interval function
__device__ __forceinline__ float soft_demod_256qam_67_cpu(float value, float rcp_noise)
{
  int   idx     = compute_interval_idx(value, 4.0f * M_SQRT1_170, 8);
  float l_value = SLOPE_256QAM_67[idx] * value + INTERCEPT_256QAM_67[idx];
  return l_value * rcp_noise;
}

// ============================================================================
// CUDA Kernels - Modulation
// ============================================================================

/// \brief BPSK modulation: bit -> {+1, -1}
__global__ void
modulate_bpsk_kernel(const uint32_t* __restrict__ d_bits, cuFloatComplex* __restrict__ d_symbols, int num_bits)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_bits)
    return;

  int      word_idx = idx / 32;
  int      bit_pos  = idx % 32;
  uint32_t bit      = (d_bits[word_idx] >> bit_pos) & 1u;

  // BPSK: 0 -> +1, 1 -> -1
  d_symbols[idx] = make_cuFloatComplex(bit ? -1.0f : 1.0f, 0.0f);
}

/// \brief QPSK modulation: 2 bits -> symbol
__global__ void
modulate_qpsk_kernel(const uint32_t* __restrict__ d_bits, cuFloatComplex* __restrict__ d_symbols, int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  int bit_idx  = idx * 2;
  int word_idx = bit_idx / 32;
  int bit_pos  = bit_idx % 32;

  // Extract 2 bits
  uint32_t bits;
  if (bit_pos <= 30) {
    bits = (d_bits[word_idx] >> bit_pos) & 0x3;
  } else {
    bits = ((d_bits[word_idx] >> bit_pos) | (d_bits[word_idx + 1] << (32 - bit_pos))) & 0x3;
  }

  d_symbols[idx] = make_cuFloatComplex(QPSK_I[bits], QPSK_Q[bits]);
}

/// \brief 16QAM modulation: 4 bits -> symbol
/// Matches cuPHY exactly: I uses bits 0,2; Q uses bits 1,3
/// Index = bits & 0x05 (keeps bits at positions 0 and 2, no shifting)
__global__ void
modulate_16qam_kernel(const uint32_t* __restrict__ d_bits, cuFloatComplex* __restrict__ d_symbols, int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  int bit_idx  = idx * 4;
  int word_idx = bit_idx / 32;
  int bit_pos  = bit_idx % 32;

  // Extract 4 bits
  uint32_t bits;
  if (bit_pos <= 28) {
    bits = (d_bits[word_idx] >> bit_pos) & 0xF;
  } else {
    bits = ((d_bits[word_idx] >> bit_pos) | (d_bits[word_idx + 1] << (32 - bit_pos))) & 0xF;
  }

  // cuPHY exact convention: index = bits & 0x05 (keeps bits 0 and 2 in place)
  // This gives indices 0, 1, 4, or 5
  // bits 0 and 2 for I
  int i_idx = bits & 0x5;
  // bits 1 and 3 for Q (shifted to positions 0 and 2)
  int q_idx = (bits >> 1) & 0x5;

  d_symbols[idx] = make_cuFloatComplex(QAM16_TABLE[i_idx], QAM16_TABLE[q_idx]);
}

/// \brief 64QAM modulation: 6 bits -> symbol
/// Matches cuPHY: I uses bits 0,2,4; Q uses bits 1,3,5
__global__ void
modulate_64qam_kernel(const uint32_t* __restrict__ d_bits, cuFloatComplex* __restrict__ d_symbols, int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  int bit_idx  = idx * 6;
  int word_idx = bit_idx / 32;
  int bit_pos  = bit_idx % 32;

  // Extract 6 bits
  uint32_t bits;
  if (bit_pos <= 26) {
    bits = (d_bits[word_idx] >> bit_pos) & 0x3F;
  } else {
    bits = ((d_bits[word_idx] >> bit_pos) | (d_bits[word_idx + 1] << (32 - bit_pos))) & 0x3F;
  }

  // cuPHY convention: I uses bits 0,2,4; Q uses bits 1,3,5
  // map_index_6bits: (index & 0x1) | ((index & 0x4) >> 1) | ((index & 0x10) >> 2)
  int i_idx = (bits & 0x1) | ((bits & 0x4) >> 1) | ((bits & 0x10) >> 2);
  int q_idx = ((bits >> 1) & 0x1) | (((bits >> 1) & 0x4) >> 1) | (((bits >> 1) & 0x10) >> 2);

  d_symbols[idx] = make_cuFloatComplex(QAM64_TABLE[i_idx], QAM64_TABLE[q_idx]);
}

/// \brief 256QAM modulation: 8 bits -> symbol
/// Matches cuPHY: I uses bits 0,2,4,6; Q uses bits 1,3,5,7
__global__ void
modulate_256qam_kernel(const uint32_t* __restrict__ d_bits, cuFloatComplex* __restrict__ d_symbols, int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  int bit_idx  = idx * 8;
  int word_idx = bit_idx / 32;
  int bit_pos  = bit_idx % 32;

  // Extract 8 bits
  uint32_t bits;
  if (bit_pos <= 24) {
    bits = (d_bits[word_idx] >> bit_pos) & 0xFF;
  } else {
    bits = ((d_bits[word_idx] >> bit_pos) | (d_bits[word_idx + 1] << (32 - bit_pos))) & 0xFF;
  }

  // cuPHY convention: I uses bits 0,2,4,6; Q uses bits 1,3,5,7
  // map_index_8bits: (index & 0x1) | ((index & 0x4) >> 1) | ((index & 0x10) >> 2) | ((index & 0x40) >> 3)
  int i_idx = (bits & 0x1) | ((bits & 0x4) >> 1) | ((bits & 0x10) >> 2) | ((bits & 0x40) >> 3);
  int q_idx =
      ((bits >> 1) & 0x1) | (((bits >> 1) & 0x4) >> 1) | (((bits >> 1) & 0x10) >> 2) | (((bits >> 1) & 0x40) >> 3);

  d_symbols[idx] = make_cuFloatComplex(QAM256_TABLE[i_idx], QAM256_TABLE[q_idx]);
}

// ============================================================================
// CUDA Kernels - INT8 Modulation (direct to ci8_t format)
// ============================================================================

/// \brief QPSK modulation to INT8: 2 bits -> ci8_t symbol
__global__ void
modulate_qpsk_int8_kernel(const uint32_t* __restrict__ d_bits, int8_t* __restrict__ d_symbols, int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  int bit_idx  = idx * 2;
  int word_idx = bit_idx / 32;
  int bit_pos  = bit_idx % 32;

  uint32_t bits          = (d_bits[word_idx] >> bit_pos) & 0x3;
  d_symbols[idx * 2]     = QPSK_I8[bits];
  d_symbols[idx * 2 + 1] = QPSK_Q8[bits];
}

/// \brief 16QAM modulation to INT8: 4 bits -> ci8_t symbol
__global__ void
modulate_16qam_int8_kernel(const uint32_t* __restrict__ d_bits, int8_t* __restrict__ d_symbols, int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  int bit_idx  = idx * 4;
  int word_idx = bit_idx / 32;
  int bit_pos  = bit_idx % 32;

  uint32_t bits;
  if (bit_pos <= 28) {
    bits = (d_bits[word_idx] >> bit_pos) & 0xF;
  } else {
    bits = ((d_bits[word_idx] >> bit_pos) | (d_bits[word_idx + 1] << (32 - bit_pos))) & 0xF;
  }

  // I uses bits 0,2; Q uses bits 1,3
  int i_idx = (bits & 0x1) | ((bits & 0x4) >> 1);
  int q_idx = ((bits >> 1) & 0x1) | (((bits >> 1) & 0x4) >> 1);

  d_symbols[idx * 2]     = QAM16_I8[i_idx];
  d_symbols[idx * 2 + 1] = QAM16_Q8[q_idx];
}

/// \brief 64QAM modulation to INT8: 6 bits -> ci8_t symbol
__global__ void
modulate_64qam_int8_kernel(const uint32_t* __restrict__ d_bits, int8_t* __restrict__ d_symbols, int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  int bit_idx  = idx * 6;
  int word_idx = bit_idx / 32;
  int bit_pos  = bit_idx % 32;

  uint32_t bits;
  if (bit_pos <= 26) {
    bits = (d_bits[word_idx] >> bit_pos) & 0x3F;
  } else {
    bits = ((d_bits[word_idx] >> bit_pos) | (d_bits[word_idx + 1] << (32 - bit_pos))) & 0x3F;
  }

  // I uses bits 0,2,4; Q uses bits 1,3,5
  int i_idx = (bits & 0x1) | ((bits & 0x4) >> 1) | ((bits & 0x10) >> 2);
  int q_idx = ((bits >> 1) & 0x1) | (((bits >> 1) & 0x4) >> 1) | (((bits >> 1) & 0x10) >> 2);

  d_symbols[idx * 2]     = QAM64_I8[i_idx];
  d_symbols[idx * 2 + 1] = QAM64_I8[q_idx];
}

/// \brief 256QAM modulation to INT8: 8 bits -> ci8_t symbol
__global__ void
modulate_256qam_int8_kernel(const uint32_t* __restrict__ d_bits, int8_t* __restrict__ d_symbols, int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  int bit_idx  = idx * 8;
  int word_idx = bit_idx / 32;
  int bit_pos  = bit_idx % 32;

  uint32_t bits;
  if (bit_pos <= 24) {
    bits = (d_bits[word_idx] >> bit_pos) & 0xFF;
  } else {
    bits = ((d_bits[word_idx] >> bit_pos) | (d_bits[word_idx + 1] << (32 - bit_pos))) & 0xFF;
  }

  // I uses bits 0,2,4,6; Q uses bits 1,3,5,7
  int i_idx = (bits & 0x1) | ((bits & 0x4) >> 1) | ((bits & 0x10) >> 2) | ((bits & 0x40) >> 3);
  int q_idx =
      ((bits >> 1) & 0x1) | (((bits >> 1) & 0x4) >> 1) | (((bits >> 1) & 0x10) >> 2) | (((bits >> 1) & 0x40) >> 3);

  d_symbols[idx * 2]     = QAM256_I8[i_idx];
  d_symbols[idx * 2 + 1] = QAM256_I8[q_idx];
}

// ============================================================================
// CUDA Kernels - Soft Demodulation
// ============================================================================

/// \brief BPSK soft demodulation
/// LLR = 2 * y / sigma^2 where y is received symbol (real part)
__global__ void soft_demod_bpsk_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                       float* __restrict__ d_llrs,
                                       int   num_symbols,
                                       float noise_var)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  float y = cuCrealf(d_symbols[idx]);
  // LLR = 2*y/sigma^2 (positive LLR means bit 0 more likely)
  d_llrs[idx] = 2.0f * y / noise_var;
}

/// \brief QPSK soft demodulation (exact LLRs)
__global__ void soft_demod_qpsk_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                       float* __restrict__ d_llrs,
                                       int   num_symbols,
                                       float noise_var)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y  = d_symbols[idx];
  float          yi = cuCrealf(y);
  float          yq = cuCimagf(y);

  // For QPSK with Gray coding:
  // bit 0: determined by I component, bit 1: determined by Q component
  // LLR_0 = 2*sqrt(2)*yi/sigma^2
  // LLR_1 = 2*sqrt(2)*yq/sigma^2
  float scale         = 2.0f * 1.4142136f / noise_var;
  d_llrs[idx * 2]     = scale * yi;
  d_llrs[idx * 2 + 1] = scale * yq;
}

/// \brief 16QAM soft demodulation (CPU-compatible interval approximation)
/// Matches cuPHY: I uses bits 0,2; Q uses bits 1,3 (interleaved)
///
/// Uses piecewise linear approximation matching srsRAN CPU implementation
__global__ void soft_demod_16qam_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                        float* __restrict__ d_llrs,
                                        int   num_symbols,
                                        float noise_var)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y  = d_symbols[idx];
  float          yi = cuCrealf(y);
  float          yq = cuCimagf(y);

  float rcp_noise = 1.0f / noise_var;

  // Use CPU-compatible interval-based soft demodulation
  // Bit 0 (I sign), Bit 1 (Q sign)
  d_llrs[idx * 4 + 0] = soft_demod_16qam_01_cpu(yi, rcp_noise);
  d_llrs[idx * 4 + 1] = soft_demod_16qam_01_cpu(yq, rcp_noise);
  // Bit 2 (I magnitude), Bit 3 (Q magnitude)
  d_llrs[idx * 4 + 2] = soft_demod_16qam_23_cpu(yi, rcp_noise);
  d_llrs[idx * 4 + 3] = soft_demod_16qam_23_cpu(yq, rcp_noise);
}

/// \brief 64QAM soft demodulation (CPU-compatible interval approximation)
/// Matches cuPHY: I uses bits 0,2,4; Q uses bits 1,3,5 (interleaved)
///
/// Uses interval-based approximation matching srsRAN CPU implementation
__global__ void soft_demod_64qam_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                        float* __restrict__ d_llrs,
                                        int   num_symbols,
                                        float noise_var)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y  = d_symbols[idx];
  float          yi = cuCrealf(y);
  float          yq = cuCimagf(y);

  float rcp_noise = 1.0f / noise_var;

  // Use CPU-compatible interval-based soft demodulation
  d_llrs[idx * 6 + 0] = soft_demod_64qam_01_cpu(yi, rcp_noise);
  d_llrs[idx * 6 + 1] = soft_demod_64qam_01_cpu(yq, rcp_noise);
  d_llrs[idx * 6 + 2] = soft_demod_64qam_23_cpu(yi, rcp_noise);
  d_llrs[idx * 6 + 3] = soft_demod_64qam_23_cpu(yq, rcp_noise);
  d_llrs[idx * 6 + 4] = soft_demod_64qam_45_cpu(yi, rcp_noise);
  d_llrs[idx * 6 + 5] = soft_demod_64qam_45_cpu(yq, rcp_noise);
}

/// \brief 256QAM soft demodulation (CPU-compatible interval approximation)
/// Matches cuPHY: I uses bits 0,2,4,6; Q uses bits 1,3,5,7 (interleaved)
///
/// Uses interval-based approximation matching srsRAN CPU implementation
__global__ void soft_demod_256qam_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                         float* __restrict__ d_llrs,
                                         int   num_symbols,
                                         float noise_var)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y  = d_symbols[idx];
  float          yi = cuCrealf(y);
  float          yq = cuCimagf(y);

  float rcp_noise = 1.0f / noise_var;

  // Use CPU-compatible interval-based soft demodulation
  d_llrs[idx * 8 + 0] = soft_demod_256qam_01_cpu(yi, rcp_noise);
  d_llrs[idx * 8 + 1] = soft_demod_256qam_01_cpu(yq, rcp_noise);
  d_llrs[idx * 8 + 2] = soft_demod_256qam_23_cpu(yi, rcp_noise);
  d_llrs[idx * 8 + 3] = soft_demod_256qam_23_cpu(yq, rcp_noise);
  d_llrs[idx * 8 + 4] = soft_demod_256qam_45_cpu(yi, rcp_noise);
  d_llrs[idx * 8 + 5] = soft_demod_256qam_45_cpu(yq, rcp_noise);
  d_llrs[idx * 8 + 6] = soft_demod_256qam_67_cpu(yi, rcp_noise);
  d_llrs[idx * 8 + 7] = soft_demod_256qam_67_cpu(yq, rcp_noise);
}

// ============================================================================
// CUDA Kernels - Soft Demodulation (Per-Symbol Noise Variance)
// ============================================================================

/// \brief BPSK soft demodulation with per-symbol noise variance
__global__ void soft_demod_bpsk_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                  const float* __restrict__ d_noise_vars,
                                                  float* __restrict__ d_llrs,
                                                  int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  float y         = cuCrealf(d_symbols[idx]);
  float noise_var = d_noise_vars[idx];
  // Handle invalid noise variance (zero, negative, or NaN)
  if (!(noise_var > 0.0f)) {
    d_llrs[idx] = 0.0f;
    return;
  }
  d_llrs[idx] = 2.0f * y / noise_var;
}

/// \brief QPSK soft demodulation with per-symbol noise variance
__global__ void soft_demod_qpsk_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                  const float* __restrict__ d_noise_vars,
                                                  float* __restrict__ d_llrs,
                                                  int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y         = d_symbols[idx];
  float          yi        = cuCrealf(y);
  float          yq        = cuCimagf(y);
  float          noise_var = d_noise_vars[idx];

  // Handle invalid noise variance
  if (!(noise_var > 0.0f)) {
    d_llrs[idx * 2]     = 0.0f;
    d_llrs[idx * 2 + 1] = 0.0f;
    return;
  }

  float scale         = 2.0f * 1.4142136f / noise_var;
  d_llrs[idx * 2]     = scale * yi;
  d_llrs[idx * 2 + 1] = scale * yq;
}

/// \brief 16QAM soft demodulation with per-symbol noise variance (CPU-compatible)
__global__ void soft_demod_16qam_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                   const float* __restrict__ d_noise_vars,
                                                   float* __restrict__ d_llrs,
                                                   int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y         = d_symbols[idx];
  float          yi        = cuCrealf(y);
  float          yq        = cuCimagf(y);
  float          noise_var = d_noise_vars[idx];

  // Handle invalid noise variance
  if (!(noise_var > 0.0f)) {
    d_llrs[idx * 4 + 0] = 0.0f;
    d_llrs[idx * 4 + 1] = 0.0f;
    d_llrs[idx * 4 + 2] = 0.0f;
    d_llrs[idx * 4 + 3] = 0.0f;
    return;
  }

  float rcp_noise = 1.0f / noise_var;

  // CPU-compatible interval-based soft demodulation
  d_llrs[idx * 4 + 0] = soft_demod_16qam_01_cpu(yi, rcp_noise);
  d_llrs[idx * 4 + 1] = soft_demod_16qam_01_cpu(yq, rcp_noise);
  d_llrs[idx * 4 + 2] = soft_demod_16qam_23_cpu(yi, rcp_noise);
  d_llrs[idx * 4 + 3] = soft_demod_16qam_23_cpu(yq, rcp_noise);
}

/// \brief 64QAM soft demodulation with per-symbol noise variance (CPU-compatible)
__global__ void soft_demod_64qam_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                   const float* __restrict__ d_noise_vars,
                                                   float* __restrict__ d_llrs,
                                                   int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y         = d_symbols[idx];
  float          yi        = cuCrealf(y);
  float          yq        = cuCimagf(y);
  float          noise_var = d_noise_vars[idx];

  // Handle invalid noise variance
  if (!(noise_var > 0.0f)) {
    for (int i = 0; i < 6; i++)
      d_llrs[idx * 6 + i] = 0.0f;
    return;
  }

  float rcp_noise = 1.0f / noise_var;

  // CPU-compatible interval-based soft demodulation
  d_llrs[idx * 6 + 0] = soft_demod_64qam_01_cpu(yi, rcp_noise);
  d_llrs[idx * 6 + 1] = soft_demod_64qam_01_cpu(yq, rcp_noise);
  d_llrs[idx * 6 + 2] = soft_demod_64qam_23_cpu(yi, rcp_noise);
  d_llrs[idx * 6 + 3] = soft_demod_64qam_23_cpu(yq, rcp_noise);
  d_llrs[idx * 6 + 4] = soft_demod_64qam_45_cpu(yi, rcp_noise);
  d_llrs[idx * 6 + 5] = soft_demod_64qam_45_cpu(yq, rcp_noise);
}

/// \brief 256QAM soft demodulation with per-symbol noise variance (CPU-compatible)
__global__ void soft_demod_256qam_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                    const float* __restrict__ d_noise_vars,
                                                    float* __restrict__ d_llrs,
                                                    int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y         = d_symbols[idx];
  float          yi        = cuCrealf(y);
  float          yq        = cuCimagf(y);
  float          noise_var = d_noise_vars[idx];

  // Handle invalid noise variance
  if (!(noise_var > 0.0f)) {
    for (int i = 0; i < 8; i++)
      d_llrs[idx * 8 + i] = 0.0f;
    return;
  }

  float rcp_noise = 1.0f / noise_var;

  // CPU-compatible interval-based soft demodulation
  d_llrs[idx * 8 + 0] = soft_demod_256qam_01_cpu(yi, rcp_noise);
  d_llrs[idx * 8 + 1] = soft_demod_256qam_01_cpu(yq, rcp_noise);
  d_llrs[idx * 8 + 2] = soft_demod_256qam_23_cpu(yi, rcp_noise);
  d_llrs[idx * 8 + 3] = soft_demod_256qam_23_cpu(yq, rcp_noise);
  d_llrs[idx * 8 + 4] = soft_demod_256qam_45_cpu(yi, rcp_noise);
  d_llrs[idx * 8 + 5] = soft_demod_256qam_45_cpu(yq, rcp_noise);
  d_llrs[idx * 8 + 6] = soft_demod_256qam_67_cpu(yi, rcp_noise);
  d_llrs[idx * 8 + 7] = soft_demod_256qam_67_cpu(yq, rcp_noise);
}

// ============================================================================
// CUDA Kernels - Half-precision (fp16) Soft Demodulation
// ============================================================================

/// \brief QPSK soft demodulation outputting fp16 LLRs
__global__ void soft_demod_qpsk_half_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                       const float* __restrict__ d_noise_vars,
                                                       __half* __restrict__ d_llrs,
                                                       int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y         = d_symbols[idx];
  float          yi        = cuCrealf(y);
  float          yq        = cuCimagf(y);
  float          noise_var = d_noise_vars[idx];

  if (!(noise_var > 0.0f)) {
    d_llrs[idx * 2]     = __float2half(0.0f);
    d_llrs[idx * 2 + 1] = __float2half(0.0f);
    return;
  }

  float scale         = 2.0f * 1.4142136f / noise_var;
  d_llrs[idx * 2]     = __float2half(scale * yi);
  d_llrs[idx * 2 + 1] = __float2half(scale * yq);
}

/// \brief 16QAM soft demodulation outputting fp16 LLRs (CPU-compatible)
__global__ void soft_demod_16qam_half_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                        const float* __restrict__ d_noise_vars,
                                                        __half* __restrict__ d_llrs,
                                                        int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y         = d_symbols[idx];
  float          yi        = cuCrealf(y);
  float          yq        = cuCimagf(y);
  float          noise_var = d_noise_vars[idx];

  if (!(noise_var > 0.0f)) {
    for (int i = 0; i < 4; i++)
      d_llrs[idx * 4 + i] = __float2half(0.0f);
    return;
  }

  float rcp_noise = 1.0f / noise_var;

  // CPU-compatible interval-based soft demodulation
  d_llrs[idx * 4 + 0] = __float2half(soft_demod_16qam_01_cpu(yi, rcp_noise));
  d_llrs[idx * 4 + 1] = __float2half(soft_demod_16qam_01_cpu(yq, rcp_noise));
  d_llrs[idx * 4 + 2] = __float2half(soft_demod_16qam_23_cpu(yi, rcp_noise));
  d_llrs[idx * 4 + 3] = __float2half(soft_demod_16qam_23_cpu(yq, rcp_noise));
}

/// \brief 64QAM soft demodulation outputting fp16 LLRs (CPU-compatible)
__global__ void soft_demod_64qam_half_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                        const float* __restrict__ d_noise_vars,
                                                        __half* __restrict__ d_llrs,
                                                        int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y         = d_symbols[idx];
  float          yi        = cuCrealf(y);
  float          yq        = cuCimagf(y);
  float          noise_var = d_noise_vars[idx];

  if (!(noise_var > 0.0f)) {
    for (int i = 0; i < 6; i++)
      d_llrs[idx * 6 + i] = __float2half(0.0f);
    return;
  }

  float rcp_noise = 1.0f / noise_var;

  // CPU-compatible interval-based soft demodulation
  d_llrs[idx * 6 + 0] = __float2half(soft_demod_64qam_01_cpu(yi, rcp_noise));
  d_llrs[idx * 6 + 1] = __float2half(soft_demod_64qam_01_cpu(yq, rcp_noise));
  d_llrs[idx * 6 + 2] = __float2half(soft_demod_64qam_23_cpu(yi, rcp_noise));
  d_llrs[idx * 6 + 3] = __float2half(soft_demod_64qam_23_cpu(yq, rcp_noise));
  d_llrs[idx * 6 + 4] = __float2half(soft_demod_64qam_45_cpu(yi, rcp_noise));
  d_llrs[idx * 6 + 5] = __float2half(soft_demod_64qam_45_cpu(yq, rcp_noise));
}

/// \brief 256QAM soft demodulation outputting fp16 LLRs (CPU-compatible)
__global__ void soft_demod_256qam_half_per_symbol_kernel(const cuFloatComplex* __restrict__ d_symbols,
                                                         const float* __restrict__ d_noise_vars,
                                                         __half* __restrict__ d_llrs,
                                                         int num_symbols)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_symbols)
    return;

  cuFloatComplex y         = d_symbols[idx];
  float          yi        = cuCrealf(y);
  float          yq        = cuCimagf(y);
  float          noise_var = d_noise_vars[idx];

  if (!(noise_var > 0.0f)) {
    for (int i = 0; i < 8; i++)
      d_llrs[idx * 8 + i] = __float2half(0.0f);
    return;
  }

  float rcp_noise = 1.0f / noise_var;

  // CPU-compatible interval-based soft demodulation
  d_llrs[idx * 8 + 0] = __float2half(soft_demod_256qam_01_cpu(yi, rcp_noise));
  d_llrs[idx * 8 + 1] = __float2half(soft_demod_256qam_01_cpu(yq, rcp_noise));
  d_llrs[idx * 8 + 2] = __float2half(soft_demod_256qam_23_cpu(yi, rcp_noise));
  d_llrs[idx * 8 + 3] = __float2half(soft_demod_256qam_23_cpu(yq, rcp_noise));
  d_llrs[idx * 8 + 4] = __float2half(soft_demod_256qam_45_cpu(yi, rcp_noise));
  d_llrs[idx * 8 + 5] = __float2half(soft_demod_256qam_45_cpu(yq, rcp_noise));
  d_llrs[idx * 8 + 6] = __float2half(soft_demod_256qam_67_cpu(yi, rcp_noise));
  d_llrs[idx * 8 + 7] = __float2half(soft_demod_256qam_67_cpu(yq, rcp_noise));
}

// ============================================================================
// Entry points
// ============================================================================

/// Number of threads a modulation or demodulation kernel runs per block.
static constexpr unsigned block_size = 256;

/// Number of blocks that covers the given number of symbols.
static unsigned blocks_for(unsigned nof_symbols)
{
  return (nof_symbols + block_size - 1) / block_size;
}

/// \brief Number of bits a symbol of the given scheme carries.
///
/// The enumerator of every scheme is its number of bits per symbol, except for pi/2-BPSK, which
/// carries 1 bit and is enumerated 0.
static unsigned bits_per_symbol(modulation_scheme scheme)
{
  return (scheme == modulation_scheme::PI_2_BPSK) ? 1 : static_cast<unsigned>(scheme);
}

cuda_result modulate(cuFloatComplex*    d_symbols,
                     const uint32_t*    d_bits,
                     unsigned           nof_bits,
                     modulation_scheme  scheme,
                     const cuda_stream& stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_bits == nullptr) || (d_symbols == nullptr) || (nof_bits == 0)) {
    return make_unexpected(std::string("The modulation call carries an invalid configuration"));
  }

  unsigned nof_bits_per_symbol = bits_per_symbol(scheme);
  unsigned nof_symbols         = (nof_bits + nof_bits_per_symbol - 1) / nof_bits_per_symbol;
  unsigned nof_blocks          = blocks_for(nof_symbols);

  switch (scheme) {
    case modulation_scheme::BPSK:
      modulate_bpsk_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_bits);
      break;
    case modulation_scheme::QPSK:
      modulate_qpsk_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_symbols);
      break;
    case modulation_scheme::QAM16:
      modulate_16qam_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_symbols);
      break;
    case modulation_scheme::QAM64:
      modulate_64qam_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_symbols);
      break;
    case modulation_scheme::QAM256:
      modulate_256qam_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_symbols);
      break;
    default:
      return make_unexpected("Modulation scheme " + to_string(scheme) + " is not implemented on the GPU");
  }

  return check_last_cuda_error("modulation mapping");
}

cuda_result modulate_int8(int8_t*            d_symbols,
                          const uint32_t*    d_bits,
                          unsigned           nof_bits,
                          modulation_scheme  scheme,
                          const cuda_stream& stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_bits == nullptr) || (d_symbols == nullptr) || (nof_bits == 0)) {
    return make_unexpected(std::string("The modulation call carries an invalid configuration"));
  }

  unsigned nof_bits_per_symbol = bits_per_symbol(scheme);
  unsigned nof_symbols         = (nof_bits + nof_bits_per_symbol - 1) / nof_bits_per_symbol;
  unsigned nof_blocks          = blocks_for(nof_symbols);

  switch (scheme) {
    case modulation_scheme::QPSK:
      modulate_qpsk_int8_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_symbols);
      break;
    case modulation_scheme::QAM16:
      modulate_16qam_int8_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_symbols);
      break;
    case modulation_scheme::QAM64:
      modulate_64qam_int8_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_symbols);
      break;
    case modulation_scheme::QAM256:
      modulate_256qam_int8_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_bits, d_symbols, nof_symbols);
      break;
    default:
      return make_unexpected("Modulation scheme " + to_string(scheme) +
                             " has no 8-bit integer constellation on the GPU");
  }

  return check_last_cuda_error("modulation mapping to 8-bit integers");
}

cuda_result soft_demodulate_constant_noise_var(float*                d_llrs,
                                               const cuFloatComplex* d_symbols,
                                               unsigned              nof_symbols,
                                               modulation_scheme     scheme,
                                               float                 noise_var,
                                               const cuda_stream&    stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_symbols == nullptr) || (d_llrs == nullptr) || (nof_symbols == 0)) {
    return make_unexpected(std::string("The demodulation call carries an invalid configuration"));
  }

  unsigned nof_blocks = blocks_for(nof_symbols);

  switch (scheme) {
    case modulation_scheme::BPSK:
      soft_demod_bpsk_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_symbols, d_llrs, nof_symbols, noise_var);
      break;
    case modulation_scheme::QPSK:
      soft_demod_qpsk_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_symbols, d_llrs, nof_symbols, noise_var);
      break;
    case modulation_scheme::QAM16:
      soft_demod_16qam_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_symbols, d_llrs, nof_symbols, noise_var);
      break;
    case modulation_scheme::QAM64:
      soft_demod_64qam_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_symbols, d_llrs, nof_symbols, noise_var);
      break;
    case modulation_scheme::QAM256:
      soft_demod_256qam_kernel<<<nof_blocks, block_size, 0, native_stream>>>(d_symbols, d_llrs, nof_symbols, noise_var);
      break;
    default:
      return make_unexpected("Modulation scheme " + to_string(scheme) + " is not implemented on the GPU");
  }

  return check_last_cuda_error("soft demodulation");
}

cuda_result soft_demodulate(float*                d_llrs,
                            const cuFloatComplex* d_symbols,
                            const float*          d_noise_vars,
                            unsigned              nof_symbols,
                            modulation_scheme     scheme,
                            const cuda_stream&    stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_symbols == nullptr) || (d_noise_vars == nullptr) || (d_llrs == nullptr) || (nof_symbols == 0)) {
    return make_unexpected(std::string("The demodulation call carries an invalid configuration"));
  }

  unsigned nof_blocks = blocks_for(nof_symbols);

  switch (scheme) {
    case modulation_scheme::BPSK:
      soft_demod_bpsk_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    case modulation_scheme::QPSK:
      soft_demod_qpsk_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    case modulation_scheme::QAM16:
      soft_demod_16qam_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    case modulation_scheme::QAM64:
      soft_demod_64qam_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    case modulation_scheme::QAM256:
      soft_demod_256qam_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    default:
      return make_unexpected("Modulation scheme " + to_string(scheme) + " is not implemented on the GPU");
  }

  return check_last_cuda_error("soft demodulation");
}

cuda_result soft_demodulate(__half*               d_llrs,
                            const cuFloatComplex* d_symbols,
                            const float*          d_noise_vars,
                            unsigned              nof_symbols,
                            modulation_scheme     scheme,
                            const cuda_stream&    stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_symbols == nullptr) || (d_noise_vars == nullptr) || (d_llrs == nullptr) || (nof_symbols == 0)) {
    return make_unexpected(std::string("The demodulation call carries an invalid configuration"));
  }

  unsigned nof_blocks = blocks_for(nof_symbols);

  switch (scheme) {
    case modulation_scheme::QPSK:
      soft_demod_qpsk_half_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    case modulation_scheme::QAM16:
      soft_demod_16qam_half_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    case modulation_scheme::QAM64:
      soft_demod_64qam_half_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    case modulation_scheme::QAM256:
      soft_demod_256qam_half_per_symbol_kernel<<<nof_blocks, block_size, 0, native_stream>>>(
          d_symbols, d_noise_vars, d_llrs, nof_symbols);
      break;
    default:
      return make_unexpected("Modulation scheme " + to_string(scheme) +
                             " has no half precision demodulator on the GPU");
  }

  return check_last_cuda_error("soft demodulation to half precision");
}

} // namespace cuda
} // namespace ocudu
