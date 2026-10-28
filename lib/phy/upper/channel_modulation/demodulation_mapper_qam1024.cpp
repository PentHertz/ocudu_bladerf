// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "demodulation_mapper_qam1024.h"
#include "demodulation_mapper_intervals.h"
#include "ocudu/phy/upper/log_likelihood_ratio.h"
#include "ocudu/support/math/complex_math_utils.h"

#if defined(__AVX512F__) && defined(__AVX512BW__) &&                                                                   \
    ((defined(__clang__) && (__clang_major__ > 4)) || (defined(__GNUC__) && (__GNUC__ > 9)))
#define HAVE_AVX512
#include "avx512_helpers.h"
#elif defined(__AVX2__)
#include "avx2_helpers.h"
#endif // __AVX2__

#ifdef __ARM_NEON
#include "neon_helpers.h"
#endif // __ARM_NEON

using namespace ocudu;

// Square root of 1/682.
static const float M_SQRT1_682 = 1.0F / std::sqrt(682.0F);

// Maximum (absolute) value considered for quantization. Larger values will be clipped.
static constexpr float                           RANGE_LIMIT_FLOAT = 20;
static const float                               INTERVAL_WIDTH_01 = 2.0F * M_SQRT1_682;
static constexpr unsigned                        NOF_INTERVALS_01  = 32;
static const std::array<float, NOF_INTERVALS_01> SLOPE_01          = {
    64.0F * M_SQRT1_682, 60.0F * M_SQRT1_682, 56.0F * M_SQRT1_682, 52.0F * M_SQRT1_682, 48.0F * M_SQRT1_682,
    44.0F * M_SQRT1_682, 40.0F * M_SQRT1_682, 36.0F * M_SQRT1_682, 32.0F * M_SQRT1_682, 28.0F * M_SQRT1_682,
    24.0F * M_SQRT1_682, 20.0F * M_SQRT1_682, 16.0F * M_SQRT1_682, 12.0F * M_SQRT1_682, 8.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,  4.0F * M_SQRT1_682,  8.0F * M_SQRT1_682,  12.0F * M_SQRT1_682, 16.0F * M_SQRT1_682,
    20.0F * M_SQRT1_682, 24.0F * M_SQRT1_682, 28.0F * M_SQRT1_682, 32.0F * M_SQRT1_682, 36.0F * M_SQRT1_682,
    40.0F * M_SQRT1_682, 44.0F * M_SQRT1_682, 48.0F * M_SQRT1_682, 52.0F * M_SQRT1_682, 56.0F * M_SQRT1_682,
    60.0F * M_SQRT1_682, 64.0F * M_SQRT1_682,
};
static constexpr std::array<float, NOF_INTERVALS_01> INTERCEPT_01 = {
    480.0F / 341.0F,
    420.0F / 341.0F,
    364.0F / 341.0F,
    312.0F / 341.0F,
    24.0F / 31.0F,
    20.0F / 31.0F,
    180.0F / 341.0F,
    144.0F / 341.0F,
    112.0F / 341.0F,
    84.0F / 341.0F,
    60.0F / 341.0F,
    40.0F / 341.0F,
    24.0F / 341.0F,
    12.0F / 341.0F,
    4.0F / 341.0F,
    0.0F,
    0.0F,
    -4.0F / 341.0F,
    -12.0F / 341.0F,
    -24.0F / 341.0F,
    -40.0F / 341.0F,
    -60.0F / 341.0F,
    -84.0F / 341.0F,
    -112.0F / 341.0F,
    -144.0F / 341.0F,
    -180.0F / 341.0F,
    -20.0F / 31.0F,
    -24.0F / 31.0F,
    -312.0F / 341.0F,
    -364.0F / 341.0F,
    -420.0F / 341.0F,
    -480.0F / 341.0F,
};

static const float                               INTERVAL_WIDTH_23 = 2.0F * M_SQRT1_682;
static constexpr unsigned                        NOF_INTERVALS_23  = 32;
static const std::array<float, NOF_INTERVALS_23> SLOPE_23          = {
    32.0F * M_SQRT1_682,  28.0F * M_SQRT1_682,  24.0F * M_SQRT1_682,  20.0F * M_SQRT1_682,  16.0F * M_SQRT1_682,
    12.0F * M_SQRT1_682,  8.0F * M_SQRT1_682,   4.0F * M_SQRT1_682,   4.0F * M_SQRT1_682,   8.0F * M_SQRT1_682,
    12.0F * M_SQRT1_682,  16.0F * M_SQRT1_682,  20.0F * M_SQRT1_682,  24.0F * M_SQRT1_682,  28.0F * M_SQRT1_682,
    32.0F * M_SQRT1_682,  -32.0F * M_SQRT1_682, -28.0F * M_SQRT1_682, -24.0F * M_SQRT1_682, -20.0F * M_SQRT1_682,
    -16.0F * M_SQRT1_682, -12.0F * M_SQRT1_682, -8.0F * M_SQRT1_682,  -4.0F * M_SQRT1_682,  -4.0F * M_SQRT1_682,
    -8.0F * M_SQRT1_682,  -12.0F * M_SQRT1_682, -16.0F * M_SQRT1_682, -20.0F * M_SQRT1_682, -24.0F * M_SQRT1_682,
    -28.0F * M_SQRT1_682, -32.0F * M_SQRT1_682,
};
static constexpr std::array<float, NOF_INTERVALS_23> INTERCEPT_23 = {
    368.0F / 341.0F, 28.0F / 31.0F,   252.0F / 341.0F, 200.0F / 341.0F, 152.0F / 341.0F, 108.0F / 341.0F,
    68.0F / 341.0F,  32.0F / 341.0F,  32.0F / 341.0F,  60.0F / 341.0F,  84.0F / 341.0F,  104.0F / 341.0F,
    120.0F / 341.0F, 12.0F / 31.0F,   140.0F / 341.0F, 144.0F / 341.0F, 144.0F / 341.0F, 140.0F / 341.0F,
    12.0F / 31.0F,   120.0F / 341.0F, 104.0F / 341.0F, 84.0F / 341.0F,  60.0F / 341.0F,  32.0F / 341.0F,
    32.0F / 341.0F,  68.0F / 341.0F,  108.0F / 341.0F, 152.0F / 341.0F, 200.0F / 341.0F, 252.0F / 341.0F,
    28.0F / 31.0F,   368.0F / 341.0F,
};

static const float                               INTERVAL_WIDTH_45 = 2.0F * M_SQRT1_682;
static constexpr unsigned                        NOF_INTERVALS_45  = 32;
static const std::array<float, NOF_INTERVALS_45> SLOPE_45          = {
    16.0F * M_SQRT1_682,  12.0F * M_SQRT1_682,  8.0F * M_SQRT1_682,  4.0F * M_SQRT1_682,   4.0F * M_SQRT1_682,
    8.0F * M_SQRT1_682,   12.0F * M_SQRT1_682,  16.0F * M_SQRT1_682, -16.0F * M_SQRT1_682, -12.0F * M_SQRT1_682,
    -8.0F * M_SQRT1_682,  -4.0F * M_SQRT1_682,  -4.0F * M_SQRT1_682, -8.0F * M_SQRT1_682,  -12.0F * M_SQRT1_682,
    -16.0F * M_SQRT1_682, 16.0F * M_SQRT1_682,  12.0F * M_SQRT1_682, 8.0F * M_SQRT1_682,   4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,   8.0F * M_SQRT1_682,   12.0F * M_SQRT1_682, 16.0F * M_SQRT1_682,  -16.0F * M_SQRT1_682,
    -12.0F * M_SQRT1_682, -8.0F * M_SQRT1_682,  -4.0F * M_SQRT1_682, -4.0F * M_SQRT1_682,  -8.0F * M_SQRT1_682,
    -12.0F * M_SQRT1_682, -16.0F * M_SQRT1_682,
};
static constexpr std::array<float, NOF_INTERVALS_45> INTERCEPT_45 = {
    216.0F / 341.0F, 156.0F / 341.0F, 100.0F / 341.0F, 48.0F / 341.0F,  48.0F / 341.0F,  92.0F / 341.0F,
    12.0F / 31.0F,   168.0F / 341.0F, -8.0F / 31.0F,   -60.0F / 341.0F, -36.0F / 341.0F, -16.0F / 341.0F,
    -16.0F / 341.0F, -28.0F / 341.0F, -36.0F / 341.0F, -40.0F / 341.0F, -40.0F / 341.0F, -36.0F / 341.0F,
    -28.0F / 341.0F, -16.0F / 341.0F, -16.0F / 341.0F, -36.0F / 341.0F, -60.0F / 341.0F, -8.0F / 31.0F,
    168.0F / 341.0F, 12.0F / 31.0F,   92.0F / 341.0F,  48.0F / 341.0F,  48.0F / 341.0F,  100.0F / 341.0F,
    156.0F / 341.0F, 216.0F / 341.0F,
};

static const float                               INTERVAL_WIDTH_67 = 2.0F * M_SQRT1_682;
static constexpr unsigned                        NOF_INTERVALS_67  = 32;
static const std::array<float, NOF_INTERVALS_67> SLOPE_67          = {
    8.0F * M_SQRT1_682,  4.0F * M_SQRT1_682,  4.0F * M_SQRT1_682,  8.0F * M_SQRT1_682,  -8.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682, -4.0F * M_SQRT1_682, -8.0F * M_SQRT1_682, 8.0F * M_SQRT1_682,  4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,  8.0F * M_SQRT1_682,  -8.0F * M_SQRT1_682, -4.0F * M_SQRT1_682, -4.0F * M_SQRT1_682,
    -8.0F * M_SQRT1_682, 8.0F * M_SQRT1_682,  4.0F * M_SQRT1_682,  4.0F * M_SQRT1_682,  8.0F * M_SQRT1_682,
    -8.0F * M_SQRT1_682, -4.0F * M_SQRT1_682, -4.0F * M_SQRT1_682, -8.0F * M_SQRT1_682, 8.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,  4.0F * M_SQRT1_682,  8.0F * M_SQRT1_682,  -8.0F * M_SQRT1_682, -4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682, -8.0F * M_SQRT1_682,
};

static constexpr std::array<float, NOF_INTERVALS_67> INTERCEPT_67 = {
    116.0F / 341.0F, 56.0F / 341.0F,  56.0F / 341.0F,  108.0F / 341.0F, -84.0F / 341.0F, -40.0F / 341.0F,
    -40.0F / 341.0F, -76.0F / 341.0F, 52.0F / 341.0F,  24.0F / 341.0F,  24.0F / 341.0F,  4.0F / 31.0F,
    -20.0F / 341.0F, -8.0F / 341.0F,  -8.0F / 341.0F,  -12.0F / 341.0F, -12.0F / 341.0F, -8.0F / 341.0F,
    -8.0F / 341.0F,  -20.0F / 341.0F, 4.0F / 31.0F,    24.0F / 341.0F,  24.0F / 341.0F,  52.0F / 341.0F,
    -76.0F / 341.0F, -40.0F / 341.0F, -40.0F / 341.0F, -84.0F / 341.0F, 108.0F / 341.0F, 56.0F / 341.0F,
    56.0F / 341.0F,  116.0F / 341.0F,
};

static const float                               INTERVAL_WIDTH_89 = 4.0F * M_SQRT1_682;
static constexpr unsigned                        NOF_INTERVALS_89  = 16;
static const std::array<float, NOF_INTERVALS_89> SLOPE_89          = {
    4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682,
    4.0F * M_SQRT1_682,
    -4.0F * M_SQRT1_682,
};
static constexpr std::array<float, NOF_INTERVALS_89> INTERCEPT_89 = {
    60.0F / 341.0F,
    -52.0F / 341.0F,
    4.0F / 31.0F,
    -36.0F / 341.0F,
    28.0F / 341.0F,
    -20.0F / 341.0F,
    12.0F / 341.0F,
    -4.0F / 341.0F,
    -4.0F / 341.0F,
    12.0F / 341.0F,
    -20.0F / 341.0F,
    28.0F / 341.0F,
    -36.0F / 341.0F,
    4.0F / 31.0F,
    -52.0F / 341.0F,
    60.0F / 341.0F,
};

#ifdef HAVE_AVX512

static void demod_QAM1024_avx512(log_likelihood_ratio* llr, const cf_t* symbol, const float* noise_var)
{
  // Indices for collocating the LLRs of each symbol contiguously. The first source register contains the LLRs for bits
  // 0 to 7 and the second source register contains the LLRs for bits 8 and 9, in symbol order (offset 0x20). Only the
  // first sixteen LLRs of the second reordered register are stored.
  alignas(64) static constexpr std::array<uint16_t, 32> idx0_ = {
      0x00, 0x02, 0x04, 0x06, 0x20, // 0 - 9
      0x01, 0x03, 0x05, 0x07, 0x21, // 10 - 19
      0x08, 0x0a, 0x0c, 0x0e, 0x22, // 20 - 29
      0x09, 0x0b, 0x0d, 0x0f, 0x23, // 30 - 39
      0x10, 0x12, 0x14, 0x16, 0x24, // 40 - 49
      0x11, 0x13, 0x15, 0x17, 0x25, // 50 - 59
      0x18, 0x1a,                   // 60 - 63
  };
  alignas(64) static constexpr std::array<uint16_t, 32> idx1_ = {
      0x1c,
      0x1e,
      0x26, // 0 - 5
      0x19,
      0x1b,
      0x1d,
      0x1f,
      0x27, // 6 - 15
  };

  __m512i rcp_noise_idx = _mm512_setr_epi32(0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7);

  // Load symbols.
  __m512 symbols = _mm512_loadu_ps(reinterpret_cast<const float*>(symbol));

  // Load noise. 8 values only.
  __m256 noise = _mm256_loadu_ps(noise_var);

  // Take the reciprocal of the noise variance.
  __m256 rcp_noise = mm512::safe_div(_mm256_set1_ps(1), noise);

  // Repeat noise values for real and imaginary parts and put the results in an AVX512 register.
  __m512 rcp_noise_ = _mm512_permutexvar_ps(rcp_noise_idx, _mm512_zextps256_ps512(rcp_noise));

  // Calculate l_value for bits 0 and 1.
  __m512 l_value_01 =
      mm512::interval_function(symbols, rcp_noise_, INTERVAL_WIDTH_01, NOF_INTERVALS_01, SLOPE_01, INTERCEPT_01);

  // Calculate l_value for bits 2 and 3.
  __m512 l_value_23 =
      mm512::interval_function(symbols, rcp_noise_, INTERVAL_WIDTH_23, NOF_INTERVALS_23, SLOPE_23, INTERCEPT_23);

  // Calculate l_value for bits 4 and 5.
  __m512 l_value_45 =
      mm512::interval_function(symbols, rcp_noise_, INTERVAL_WIDTH_45, NOF_INTERVALS_45, SLOPE_45, INTERCEPT_45);

  // Calculate l_value for bits 6 and 7.
  __m512 l_value_67 =
      mm512::interval_function(symbols, rcp_noise_, INTERVAL_WIDTH_67, NOF_INTERVALS_67, SLOPE_67, INTERCEPT_67);

  // Calculate l_value for bits 8 and 9.
  __m512 l_value_89 =
      mm512::interval_function(symbols, rcp_noise_, INTERVAL_WIDTH_89, NOF_INTERVALS_89, SLOPE_89, INTERCEPT_89);

  // Quantize LLRs for bits 0 to 7.
  __m512i llr_i8_0 = mm512::quantize_ps(l_value_01, l_value_23, l_value_45, l_value_67, RANGE_LIMIT_FLOAT);

  // Quantize LLRs for bits 8 and 9. A single register needs no packing, the conversion to 8 bit keeps the order.
  __m512  SCALE       = _mm512_set1_ps(static_cast<float>(log_likelihood_ratio::max().to_int()) / RANGE_LIMIT_FLOAT);
  __m512  RANGE_CEIL  = _mm512_set1_ps(log_likelihood_ratio::max().to_int());
  __m512  RANGE_FLOOR = _mm512_set1_ps(log_likelihood_ratio::min().to_int());
  __m512  l_value_89_ = mm512::scale_clip_and_round_ps(l_value_89, SCALE, RANGE_CEIL, RANGE_FLOOR);
  __m512i llr_i8_1    = _mm512_zextsi128_si512(_mm512_cvtsepi32_epi8(_mm512_cvtps_epi32(l_value_89_)));

  // Recolocate LLRs within two 512-bit registers.
  __m512i idx0       = _mm512_load_si512(idx0_.data());
  __m512i idx1       = _mm512_load_si512(idx1_.data());
  __m512i reordered0 = _mm512_permutex2var_epi16(llr_i8_0, idx0, llr_i8_1);
  __m512i reordered1 = _mm512_permutex2var_epi16(llr_i8_0, idx1, llr_i8_1);

  // Store 80 LLRs.
  _mm512_storeu_si512(reinterpret_cast<__m512i*>(llr), reordered0);
  _mm_storeu_si128(reinterpret_cast<__m128i*>(llr + 64), _mm512_castsi512_si128(reordered1));
}
#elif defined(__AVX2__)
// Computes the l_values of four symbols and collocates them contiguously in five AVX registers.
static void demod_QAM1024_avx2_four_symbols(__m256  symbols,
                                            __m256  rcp_noise,
                                            __m256& l_value_0_,
                                            __m256& l_value_1_,
                                            __m256& l_value_2_,
                                            __m256& l_value_3_,
                                            __m256& l_value_4_)
{
  // Calculate l_value for bits 0 and 1.
  __m256 l_value_01 =
      mm256::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_01, NOF_INTERVALS_01, SLOPE_01, INTERCEPT_01);

  // Calculate l_value for bits 2 and 3.
  __m256 l_value_23 =
      mm256::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_23, NOF_INTERVALS_23, SLOPE_23, INTERCEPT_23);

  // Calculate l_value for bits 4 and 5.
  __m256 l_value_45 =
      mm256::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_45, NOF_INTERVALS_45, SLOPE_45, INTERCEPT_45);

  // Calculate l_value for bits 6 and 7.
  __m256 l_value_67 =
      mm256::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_67, NOF_INTERVALS_67, SLOPE_67, INTERCEPT_67);

  // Calculate l_value for bits 8 and 9.
  __m256 l_value_89 =
      mm256::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_89, NOF_INTERVALS_89, SLOPE_89, INTERCEPT_89);

  // Re-collocate values.
  __m256 l_value_0 = _mm256_shuffle_ps(l_value_01, l_value_23, _MM_SHUFFLE(1, 0, 1, 0));
  __m256 l_value_1 = _mm256_shuffle_ps(l_value_45, l_value_67, _MM_SHUFFLE(1, 0, 1, 0));
  __m256 l_value_2 = _mm256_shuffle_ps(l_value_89, l_value_01, _MM_SHUFFLE(3, 2, 1, 0));
  __m256 l_value_3 = _mm256_shuffle_ps(l_value_23, l_value_45, _MM_SHUFFLE(3, 2, 3, 2));
  __m256 l_value_4 = _mm256_shuffle_ps(l_value_67, l_value_89, _MM_SHUFFLE(3, 2, 3, 2));

  // Re-collocate SSE registers.
  l_value_0_ = _mm256_permute2f128_ps(l_value_0, l_value_1, 0x20);
  l_value_1_ = _mm256_permute2f128_ps(l_value_2, l_value_3, 0x20);
  l_value_2_ = _mm256_permute2f128_ps(l_value_4, l_value_0, 0x30);
  l_value_3_ = _mm256_permute2f128_ps(l_value_1, l_value_2, 0x31);
  l_value_4_ = _mm256_permute2f128_ps(l_value_3, l_value_4, 0x31);
}

static void demod_QAM1024_avx2(log_likelihood_ratio* llr, const cf_t* symbol, const float* noise_var)
{
  // Load symbols.
  __m256 symbols_0 = _mm256_loadu_ps(reinterpret_cast<const float*>(symbol + 0));
  __m256 symbols_1 = _mm256_loadu_ps(reinterpret_cast<const float*>(symbol + 4));

  // Load noise.
  __m256 noise = _mm256_loadu_ps(noise_var);

  // Take the reciprocal of the noise variance.
  __m256 rcp_noise = mm256::safe_div(_mm256_set1_ps(1), noise);

  // Repeat noise values for real and imaginary parts.
  __m256 rcp_noise_lo = _mm256_unpacklo_ps(rcp_noise, rcp_noise);
  __m256 rcp_noise_hi = _mm256_unpackhi_ps(rcp_noise, rcp_noise);

  // Re-collocate SSE registers.
  __m256 rcp_noise_0 = _mm256_permute2f128_ps(rcp_noise_lo, rcp_noise_hi, 0x20);
  __m256 rcp_noise_1 = _mm256_permute2f128_ps(rcp_noise_lo, rcp_noise_hi, 0x31);

  // Calculate the l_values of the first four symbols.
  __m256 l_value_0;
  __m256 l_value_1;
  __m256 l_value_2;
  __m256 l_value_3;
  __m256 l_value_4;
  demod_QAM1024_avx2_four_symbols(symbols_0, rcp_noise_0, l_value_0, l_value_1, l_value_2, l_value_3, l_value_4);

  // Calculate the l_values of the last four symbols.
  __m256 l_value_5;
  __m256 l_value_6;
  __m256 l_value_7;
  __m256 l_value_8;
  __m256 l_value_9;
  demod_QAM1024_avx2_four_symbols(symbols_1, rcp_noise_1, l_value_5, l_value_6, l_value_7, l_value_8, l_value_9);

  // Store 80 LLRs.
  _mm256_storeu_si256(reinterpret_cast<__m256i*>(llr + 0),
                      mm256::quantize_ps(l_value_0, l_value_1, l_value_2, l_value_3, RANGE_LIMIT_FLOAT));
  _mm256_storeu_si256(reinterpret_cast<__m256i*>(llr + 32),
                      mm256::quantize_ps(l_value_4, l_value_5, l_value_6, l_value_7, RANGE_LIMIT_FLOAT));
  __m256i llr_i8_2 =
      mm256::quantize_ps(l_value_8, l_value_9, _mm256_setzero_ps(), _mm256_setzero_ps(), RANGE_LIMIT_FLOAT);
  _mm_storeu_si128(reinterpret_cast<__m128i*>(llr + 64), _mm256_castsi256_si128(llr_i8_2));
}
#endif // HAVE_AVX512

#ifdef __ARM_NEON
// Computes the l_values of two symbols and collocates them contiguously in five NEON registers.
static void demod_QAM1024_neon_two_symbols(float32x4_t  symbols,
                                           float32x4_t  rcp_noise,
                                           float32x4_t& l_value_0,
                                           float32x4_t& l_value_1,
                                           float32x4_t& l_value_2,
                                           float32x4_t& l_value_3,
                                           float32x4_t& l_value_4)
{
  // Calculate l_value for bits 0 and 1.
  float32x4_t l_value_01 =
      neon::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_01, NOF_INTERVALS_01, SLOPE_01, INTERCEPT_01);

  // Calculate l_value for bits 2 and 3.
  float32x4_t l_value_23 =
      neon::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_23, NOF_INTERVALS_23, SLOPE_23, INTERCEPT_23);

  // Calculate l_value for bits 4 and 5.
  float32x4_t l_value_45 =
      neon::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_45, NOF_INTERVALS_45, SLOPE_45, INTERCEPT_45);

  // Calculate l_value for bits 6 and 7.
  float32x4_t l_value_67 =
      neon::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_67, NOF_INTERVALS_67, SLOPE_67, INTERCEPT_67);

  // Calculate l_value for bits 8 and 9.
  float32x4_t l_value_89 =
      neon::interval_function(symbols, rcp_noise, INTERVAL_WIDTH_89, NOF_INTERVALS_89, SLOPE_89, INTERCEPT_89);

  // Re-collocate values.
  l_value_0 = vreinterpretq_f32_u64(vtrn1q_u64(vreinterpretq_u64_f32(l_value_01), vreinterpretq_u64_f32(l_value_23)));
  l_value_1 = vreinterpretq_f32_u64(vtrn1q_u64(vreinterpretq_u64_f32(l_value_45), vreinterpretq_u64_f32(l_value_67)));
  l_value_2 = vreinterpretq_f32_u64(
      vtrn1q_u64(vreinterpretq_u64_f32(l_value_89), vreinterpretq_u64_f32(vextq_f32(l_value_01, l_value_01, 2))));
  l_value_3 = vreinterpretq_f32_u64(vtrn2q_u64(vreinterpretq_u64_f32(l_value_23), vreinterpretq_u64_f32(l_value_45)));
  l_value_4 = vreinterpretq_f32_u64(vtrn2q_u64(vreinterpretq_u64_f32(l_value_67), vreinterpretq_u64_f32(l_value_89)));
}

static void demod_QAM1024_neon(log_likelihood_ratio* llr, const cf_t* symbol, const float* noise_var)
{
  // Load symbols.
  float32x4_t symbols_0 = vld1q_f32(reinterpret_cast<const float*>(symbol + 0));
  float32x4_t symbols_1 = vld1q_f32(reinterpret_cast<const float*>(symbol + 2));
  float32x4_t symbols_2 = vld1q_f32(reinterpret_cast<const float*>(symbol + 4));
  float32x4_t symbols_3 = vld1q_f32(reinterpret_cast<const float*>(symbol + 6));

  // Load noise.
  float32x4_t noise_0 = vld1q_f32(noise_var);
  float32x4_t noise_1 = vld1q_f32(noise_var + 4);

  // Take the reciprocal of the noise variance.
  float32x4_t rcp_noise_0 = neon::safe_div(vdupq_n_f32(1.0f), noise_0);
  float32x4_t rcp_noise_1 = neon::safe_div(vdupq_n_f32(1.0f), noise_1);

  // Repeat noise values for real and imaginary parts.
  float32x4x2_t noise_rep_0  = vzipq_f32(rcp_noise_0, rcp_noise_0);
  float32x4_t   rcp_noise_0_ = noise_rep_0.val[0];
  float32x4_t   rcp_noise_1_ = noise_rep_0.val[1];
  float32x4x2_t noise_rep_1  = vzipq_f32(rcp_noise_1, rcp_noise_1);
  float32x4_t   rcp_noise_2_ = noise_rep_1.val[0];
  float32x4_t   rcp_noise_3_ = noise_rep_1.val[1];

  // Calculate the l_values of each pair of symbols.
  float32x4_t l_value_0;
  float32x4_t l_value_1;
  float32x4_t l_value_2;
  float32x4_t l_value_3;
  float32x4_t l_value_4;
  demod_QAM1024_neon_two_symbols(symbols_0, rcp_noise_0_, l_value_0, l_value_1, l_value_2, l_value_3, l_value_4);
  float32x4_t l_value_5;
  float32x4_t l_value_6;
  float32x4_t l_value_7;
  float32x4_t l_value_8;
  float32x4_t l_value_9;
  demod_QAM1024_neon_two_symbols(symbols_1, rcp_noise_1_, l_value_5, l_value_6, l_value_7, l_value_8, l_value_9);
  float32x4_t l_value_10;
  float32x4_t l_value_11;
  float32x4_t l_value_12;
  float32x4_t l_value_13;
  float32x4_t l_value_14;
  demod_QAM1024_neon_two_symbols(symbols_2, rcp_noise_2_, l_value_10, l_value_11, l_value_12, l_value_13, l_value_14);
  float32x4_t l_value_15;
  float32x4_t l_value_16;
  float32x4_t l_value_17;
  float32x4_t l_value_18;
  float32x4_t l_value_19;
  demod_QAM1024_neon_two_symbols(symbols_3, rcp_noise_3_, l_value_15, l_value_16, l_value_17, l_value_18, l_value_19);

  // Store 80 LLRs.
  vst1q_s8(reinterpret_cast<int8_t*>(llr + 0),
           neon::quantize_f32(l_value_0, l_value_1, l_value_2, l_value_3, RANGE_LIMIT_FLOAT));
  vst1q_s8(reinterpret_cast<int8_t*>(llr + 16),
           neon::quantize_f32(l_value_4, l_value_5, l_value_6, l_value_7, RANGE_LIMIT_FLOAT));
  vst1q_s8(reinterpret_cast<int8_t*>(llr + 32),
           neon::quantize_f32(l_value_8, l_value_9, l_value_10, l_value_11, RANGE_LIMIT_FLOAT));
  vst1q_s8(reinterpret_cast<int8_t*>(llr + 48),
           neon::quantize_f32(l_value_12, l_value_13, l_value_14, l_value_15, RANGE_LIMIT_FLOAT));
  vst1q_s8(reinterpret_cast<int8_t*>(llr + 64),
           neon::quantize_f32(l_value_16, l_value_17, l_value_18, l_value_19, RANGE_LIMIT_FLOAT));
}
#endif // __ARM_NEON

static log_likelihood_ratio demod_1024QAM_symbol_01(float value, float rcp_noise_var)
{
  float l_value = interval_function(value, rcp_noise_var, INTERVAL_WIDTH_01, NOF_INTERVALS_01, SLOPE_01, INTERCEPT_01);
  return log_likelihood_ratio::quantize(l_value, RANGE_LIMIT_FLOAT);
}

static log_likelihood_ratio demod_1024QAM_symbol_23(float value, float rcp_noise_var)
{
  float l_value = interval_function(value, rcp_noise_var, INTERVAL_WIDTH_23, NOF_INTERVALS_23, SLOPE_23, INTERCEPT_23);
  return log_likelihood_ratio::quantize(l_value, RANGE_LIMIT_FLOAT);
}

static log_likelihood_ratio demod_1024QAM_symbol_45(float value, float rcp_noise_var)
{
  float l_value = interval_function(value, rcp_noise_var, INTERVAL_WIDTH_45, NOF_INTERVALS_45, SLOPE_45, INTERCEPT_45);
  return log_likelihood_ratio::quantize(l_value, RANGE_LIMIT_FLOAT);
}

static log_likelihood_ratio demod_1024QAM_symbol_67(float value, float rcp_noise_var)
{
  float l_value = interval_function(value, rcp_noise_var, INTERVAL_WIDTH_67, NOF_INTERVALS_67, SLOPE_67, INTERCEPT_67);
  return log_likelihood_ratio::quantize(l_value, RANGE_LIMIT_FLOAT);
}

static log_likelihood_ratio demod_1024QAM_symbol_89(float value, float rcp_noise_var)
{
  float l_value = interval_function(value, rcp_noise_var, INTERVAL_WIDTH_89, NOF_INTERVALS_89, SLOPE_89, INTERCEPT_89);
  return log_likelihood_ratio::quantize(l_value, RANGE_LIMIT_FLOAT);
}

void ocudu::demodulate_soft_QAM1024(span<log_likelihood_ratio> llrs,
                                    span<const cf_t>           symbols,
                                    span<const float>          noise_vars)
{
  const cf_t*           symbols_it   = symbols.begin();
  const float*          noise_it     = noise_vars.begin();
  log_likelihood_ratio* llr_it       = llrs.begin();
  std::size_t           symbol_index = 0;

#ifdef HAVE_AVX512
  // For AVX512, it generates 80 LLRs simultaneously. The input is read in batches of 8 symbols.
  for (std::size_t symbol_index_end = (symbols.size() / 8) * 8; symbol_index != symbol_index_end; symbol_index += 8) {
    demod_QAM1024_avx512(llr_it, symbols_it, noise_it);

    llr_it += 80;
    symbols_it += 8;
    noise_it += 8;
  }
#elif defined(__AVX2__)
  // For AVX2, it generates 80 LLRs simultaneously. The input is read in batches of 8 symbols.
  for (std::size_t symbol_index_end = (symbols.size() / 8) * 8; symbol_index != symbol_index_end; symbol_index += 8) {
    demod_QAM1024_avx2(llr_it, symbols_it, noise_it);

    llr_it += 80;
    symbols_it += 8;
    noise_it += 8;
  }
#endif // HAVE_AVX512

#ifdef __ARM_NEON
  // For NEON, it generates 80 LLRs simultaneously. The input is read in batches of 8 symbols.
  for (std::size_t symbol_index_end = (symbols.size() / 8) * 8; symbol_index != symbol_index_end; symbol_index += 8) {
    demod_QAM1024_neon(llr_it, symbols_it, noise_it);

    llr_it += 80;
    symbols_it += 8;
    noise_it += 8;
  }
#endif // __ARM_NEON

  for (std::size_t symbol_index_end = symbols.size(); symbol_index != symbol_index_end; ++symbol_index) {
    //  Set all LLR to zero if the symbol is near zero.
    if (is_near_zero(*symbols_it)) {
      for (unsigned i_bit = 0; i_bit != 10; ++i_bit) {
        *llr_it++ = 0;
      }
      ++symbols_it;
      ++noise_it;
      continue;
    }

    float rcp_noise = 0;
    // Note: "noise_var > 0" is false also when "noise_var" is NaN.
    if (*noise_it > 0) {
      rcp_noise = 1 / (*noise_it);
    }
    *llr_it++ = demod_1024QAM_symbol_01(std::real(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_01(std::imag(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_23(std::real(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_23(std::imag(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_45(std::real(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_45(std::imag(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_67(std::real(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_67(std::imag(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_89(std::real(*symbols_it), rcp_noise);
    *llr_it++ = demod_1024QAM_symbol_89(std::imag(*symbols_it), rcp_noise);
    ++symbols_it;
    ++noise_it;
  }
}
