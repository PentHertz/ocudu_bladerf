// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ldpc_rate_dematcher_avx512_impl.h"
#include "avx512_support.h"
#include <immintrin.h>

using namespace ocudu;

void ldpc_rate_dematcher_avx512_impl::combine_softbits(span<ocudu::log_likelihood_ratio>       out,
                                                       span<const ocudu::log_likelihood_ratio> in0,
                                                       span<const ocudu::log_likelihood_ratio> in1) const
{
  ocudu_assert(out.size() == in0.size(), "All sizes must be equal.");
  ocudu_assert(out.size() == in1.size(), "All sizes must be equal.");

  unsigned index = 0;

  mm512::avx512_span       avx_out(out, out.size() / AVX512_SIZE_BYTE);
  mm512::avx512_const_span avx_in0(in0, in0.size() / AVX512_SIZE_BYTE);
  mm512::avx512_const_span avx_in1(in1, in1.size() / AVX512_SIZE_BYTE);

  __m512i LLR_MAX_epi8 = _mm512_set1_epi8(static_cast<int8_t>(LLR_MAX));
  __m512i LLR_MIN_epi8 = _mm512_set1_epi8(static_cast<int8_t>(LLR_MIN));

  for (unsigned index_end = (out.size() / AVX512_SIZE_BYTE); index != index_end; ++index) {
    __m512i in0_epi8 = avx_in0.get_at(index);
    __m512i in1_epi8 = avx_in1.get_at(index);

    __m512i result = _mm512_adds_epi8(in0_epi8, in1_epi8);

    __mmask64 mask_epi8 = _mm512_cmpgt_epi8_mask(result, LLR_MAX_epi8);
    result              = _mm512_mask_blend_epi8(mask_epi8, result, LLR_MAX_epi8);

    mask_epi8 = _mm512_cmpgt_epi8_mask(LLR_MIN_epi8, result);
    result    = _mm512_mask_blend_epi8(mask_epi8, result, LLR_MIN_epi8);

    avx_out.set_at(index, result);
  }
  index *= AVX512_SIZE_BYTE;

  for (unsigned index_end = out.size(); index != index_end; ++index) {
    out[index] = in0[index] + in1[index];
  }
}

static void mm512_deinterleave(__m512i& out_even, __m512i& out_odd, __m512i in_lo, __m512i in_hi)
{
  // Convert AVX registers to a linear input.
  alignas(AVX512_SIZE_BYTE) std::array<int8_t, 2 * AVX512_SIZE_BYTE> in = {};
  _mm512_store_si512(reinterpret_cast<__m512i*>(in.data() + 0), in_lo);
  _mm512_store_si512(reinterpret_cast<__m512i*>(in.data() + AVX512_SIZE_BYTE), in_hi);

  // Actual deinterleaver algorithm. Let the compiler optimize.
  alignas(AVX512_SIZE_BYTE) std::array<int8_t, 2 * AVX512_SIZE_BYTE> out = {};
  for (unsigned i = 0; i != AVX512_SIZE_BYTE; ++i) {
    for (unsigned j = 0; j != 2; ++j) {
      out[AVX512_SIZE_BYTE * j + i] = in[i * 2 + j];
    }
  }

  // Convert linear output to AVX registers.
  out_even = _mm512_load_si512(reinterpret_cast<const __m512i*>(out.data() + 0));
  out_odd  = _mm512_load_si512(reinterpret_cast<const __m512i*>(out.data() + AVX512_SIZE_BYTE));
}

static void mm512_deinterleave(__m512i& out0, __m512i& out1, __m512i& out2, __m512i in0, __m512i in1, __m512i in2)
{
  // Permute indexes.
  alignas(AVX512_SIZE_BYTE) static constexpr int8_t idx0_[AVX512_SIZE_BYTE] = {
      0,  3,  6,  9,  12, 15, 18, 21, 24, 27, 30, 33, 36,  39,  42,  45,  48,  51,  54,  57,  60,  63,
      66, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 123, 126, 0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,
  };
  alignas(AVX512_SIZE_BYTE) static constexpr int8_t idx1_[AVX512_SIZE_BYTE] = {
      65, 68, 71, 74, 77, 80, 83, 86, 89, 92, 95, 98, 101, 104, 107, 110, 113, 116, 119, 122, 125, 0,
      3,  6,  9,  12, 15, 18, 21, 24, 27, 30, 33, 36, 39,  42,  45,  48,  51,  54,  57,  60,  63,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,
  };
  alignas(AVX512_SIZE_BYTE) static constexpr int8_t idx2_[AVX512_SIZE_BYTE] = {
      2,  5,  8,  11, 14, 17, 20, 23, 26, 29, 32, 35,  38,  41,  44,  47,  50,  53,  56,  59,  62, 65,
      68, 71, 74, 77, 80, 83, 86, 89, 92, 95, 98, 101, 104, 107, 110, 113, 116, 119, 122, 125, 0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,
  };
  alignas(AVX512_SIZE_BYTE) static constexpr int8_t idx3_[AVX512_SIZE_BYTE] = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,  12,  13,  14,  15,  16,  17,  18,  19,  20, 21,
      22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,  34,  35,  36,  37,  38,  39,  40,  41,  42, 65,
      68, 71, 74, 77, 80, 83, 86, 89, 92, 95, 98, 101, 104, 107, 110, 113, 116, 119, 122, 125,
  };
  alignas(AVX512_SIZE_BYTE) static constexpr int8_t idx4_[AVX512_SIZE_BYTE] = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,  12,  13,  14,  15,  16,  17,  18,  19,  20, 21,
      22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,  34,  35,  36,  37,  38,  39,  40,  41,  42, 66,
      69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 123, 126,
  };
  alignas(AVX512_SIZE_BYTE) static constexpr int8_t idx5_[AVX512_SIZE_BYTE] = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  20, 21,
      22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  64, 67,
      70, 73, 76, 79, 82, 85, 88, 91, 94, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124, 127,
  };

  // Deinterleave first output.
  __m512i idx0  = _mm512_load_si512(idx0_);
  __m512i idx3  = _mm512_load_si512(idx3_);
  __m512i temp0 = _mm512_permutex2var_epi8(in0, idx0, in1);
  out0          = _mm512_permutex2var_epi8(temp0, idx3, in2);

  // Deinterleave second output.
  __m512i idx1  = _mm512_load_si512(idx1_);
  __m512i idx4  = _mm512_load_si512(idx4_);
  __m512i temp1 = _mm512_permutex2var_epi8(in1, idx1, in0);
  out1          = _mm512_permutex2var_epi8(temp1, idx4, in2);

  // Deinterleave third output.
  __m512i idx2  = _mm512_load_si512(idx2_);
  __m512i idx5  = _mm512_load_si512(idx5_);
  __m512i temp2 = _mm512_permutex2var_epi8(in0, idx2, in1);
  out2          = _mm512_permutex2var_epi8(temp2, idx5, in2);
}

// Generates the permutation indices of one stage of the five-way deinterleaver for the given output.
//
// The byte k of output j_out is the input byte 5 * k + j_out. The first stage gathers the bytes from the first two
// input registers. Each of the following stages keeps the bytes gathered so far and adds the ones from the next input
// register.
static constexpr std::array<int8_t, AVX512_SIZE_BYTE> deinterleave5_permute_idx(unsigned j_out, unsigned stage)
{
  std::array<int8_t, AVX512_SIZE_BYTE> idx = {};
  for (unsigned k = 0; k != AVX512_SIZE_BYTE; ++k) {
    unsigned in_index  = 5 * k + j_out;
    unsigned in_reg    = in_index / AVX512_SIZE_BYTE;
    unsigned in_offset = in_index % AVX512_SIZE_BYTE;
    if (stage == 0) {
      if (in_reg <= 1) {
        idx[k] = static_cast<int8_t>(in_index);
      }
    } else if (in_reg <= stage) {
      idx[k] = static_cast<int8_t>(k);
    } else if (in_reg == stage + 1) {
      idx[k] = static_cast<int8_t>(AVX512_SIZE_BYTE + in_offset);
    }
  }
  return idx;
}

// Gathers the output OutIdx of the five-way deinterleaver.
template <unsigned OutIdx>
static __m512i mm512_deinterleave5_gather(__m512i in0, __m512i in1, __m512i in2, __m512i in3, __m512i in4)
{
  alignas(AVX512_SIZE_BYTE) static constexpr std::array<int8_t, AVX512_SIZE_BYTE> idx0_ =
      deinterleave5_permute_idx(OutIdx, 0);
  alignas(AVX512_SIZE_BYTE) static constexpr std::array<int8_t, AVX512_SIZE_BYTE> idx1_ =
      deinterleave5_permute_idx(OutIdx, 1);
  alignas(AVX512_SIZE_BYTE) static constexpr std::array<int8_t, AVX512_SIZE_BYTE> idx2_ =
      deinterleave5_permute_idx(OutIdx, 2);
  alignas(AVX512_SIZE_BYTE) static constexpr std::array<int8_t, AVX512_SIZE_BYTE> idx3_ =
      deinterleave5_permute_idx(OutIdx, 3);

  __m512i temp = _mm512_permutex2var_epi8(in0, _mm512_load_si512(idx0_.data()), in1);
  temp         = _mm512_permutex2var_epi8(temp, _mm512_load_si512(idx1_.data()), in2);
  temp         = _mm512_permutex2var_epi8(temp, _mm512_load_si512(idx2_.data()), in3);
  return _mm512_permutex2var_epi8(temp, _mm512_load_si512(idx3_.data()), in4);
}

static void mm512_deinterleave(__m512i& out0,
                               __m512i& out1,
                               __m512i& out2,
                               __m512i& out3,
                               __m512i& out4,
                               __m512i  in0,
                               __m512i  in1,
                               __m512i  in2,
                               __m512i  in3,
                               __m512i  in4)
{
  out0 = mm512_deinterleave5_gather<0>(in0, in1, in2, in3, in4);
  out1 = mm512_deinterleave5_gather<1>(in0, in1, in2, in3, in4);
  out2 = mm512_deinterleave5_gather<2>(in0, in1, in2, in3, in4);
  out3 = mm512_deinterleave5_gather<3>(in0, in1, in2, in3, in4);
  out4 = mm512_deinterleave5_gather<4>(in0, in1, in2, in3, in4);
}

void ldpc_rate_dematcher_avx512_impl::deinterleave_qpsk(span<log_likelihood_ratio>       out,
                                                        span<const log_likelihood_ratio> in) const
{
  unsigned E = out.size();
  unsigned K = E / 2;
  unsigned i = 0;

  // Reinterpret pointers.
  mm512::avx512_const_span avx512_in(in, E / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out0(out.subspan(0UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out1(out.subspan(1UL * K, K), K / AVX512_SIZE_BYTE);

  // Do SIMD processing.
  for (unsigned i_end = (K / AVX512_SIZE_BYTE), i_in = 0; i != i_end; ++i) {
    __m512i in0 = avx512_in.get_at(i_in++);
    __m512i in1 = avx512_in.get_at(i_in++);

    __m512i out0;
    __m512i out1;
    mm512_deinterleave(out0, out1, in0, in1);

    avx512_out0.set_at(i, out0);
    avx512_out1.set_at(i, out1);
  }
  i *= AVX512_SIZE_BYTE;

  // Remainder of soft bits.
  for (; i != K; ++i) {
    out[K * 0U + i] = in[2U * i + 0];
    out[K * 1U + i] = in[2U * i + 1];
  }
}

void ldpc_rate_dematcher_avx512_impl::deinterleave_qam16(span<log_likelihood_ratio>       out,
                                                         span<const log_likelihood_ratio> in) const
{
  unsigned E = out.size();
  unsigned K = E / 4;
  unsigned i = 0;

  // Reinterpret pointers.
  mm512::avx512_const_span avx512_in(in, E / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out0(out.subspan(0UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out1(out.subspan(1UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out2(out.subspan(2UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out3(out.subspan(3UL * K, K), K / AVX512_SIZE_BYTE);

  // Do SIMD processing.
  for (unsigned i_end = (K / AVX512_SIZE_BYTE), i_in = 0; i != i_end; ++i) {
    __m512i in0 = avx512_in.get_at(i_in++);
    __m512i in1 = avx512_in.get_at(i_in++);
    __m512i in2 = avx512_in.get_at(i_in++);
    __m512i in3 = avx512_in.get_at(i_in++);

    __m512i deinterleaved_01_even;
    __m512i deinterleaved_01_odd;
    mm512_deinterleave(deinterleaved_01_even, deinterleaved_01_odd, in0, in1);

    __m512i deinterleaved_23_even;
    __m512i deinterleaved_23_odd;
    mm512_deinterleave(deinterleaved_23_even, deinterleaved_23_odd, in2, in3);

    __m512i out0;
    __m512i out2;
    mm512_deinterleave(out0, out2, deinterleaved_01_even, deinterleaved_23_even);

    __m512i out1;
    __m512i out3;
    mm512_deinterleave(out1, out3, deinterleaved_01_odd, deinterleaved_23_odd);

    avx512_out0.set_at(i, out0);
    avx512_out1.set_at(i, out1);
    avx512_out2.set_at(i, out2);
    avx512_out3.set_at(i, out3);
  }
  i *= AVX512_SIZE_BYTE;

  // Remainder of soft bits.
  for (; i != K; ++i) {
    out[K * 0U + i] = in[4U * i + 0];
    out[K * 1U + i] = in[4U * i + 1];
    out[K * 2U + i] = in[4U * i + 2];
    out[K * 3U + i] = in[4U * i + 3];
  }
}

void ldpc_rate_dematcher_avx512_impl::deinterleave_qam64(span<log_likelihood_ratio>       out,
                                                         span<const log_likelihood_ratio> in) const
{
  unsigned E = out.size();
  unsigned K = E / 6;
  unsigned i = 0;

  // Reinterpret pointers.
  mm512::avx512_const_span avx512_in(in, E / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out0(out.subspan(0UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out1(out.subspan(1UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out2(out.subspan(2UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out3(out.subspan(3UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out4(out.subspan(4UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out5(out.subspan(5UL * K, K), K / AVX512_SIZE_BYTE);

  // Do SIMD processing.
  for (unsigned i_end = (K / AVX512_SIZE_BYTE), i_in = 0; i != i_end; ++i) {
    __m512i in0 = avx512_in.get_at(i_in++);
    __m512i in1 = avx512_in.get_at(i_in++);
    __m512i in2 = avx512_in.get_at(i_in++);
    __m512i in3 = avx512_in.get_at(i_in++);
    __m512i in4 = avx512_in.get_at(i_in++);
    __m512i in5 = avx512_in.get_at(i_in++);

    __m512i deinterleaved_0;
    __m512i deinterleaved_1;
    __m512i deinterleaved_2;
    mm512_deinterleave(deinterleaved_0, deinterleaved_1, deinterleaved_2, in0, in1, in2);

    __m512i deinterleaved_3;
    __m512i deinterleaved_4;
    __m512i deinterleaved_5;
    mm512_deinterleave(deinterleaved_3, deinterleaved_4, deinterleaved_5, in3, in4, in5);

    __m512i out0;
    __m512i out3;
    mm512_deinterleave(out0, out3, deinterleaved_0, deinterleaved_3);

    __m512i out1;
    __m512i out4;
    mm512_deinterleave(out1, out4, deinterleaved_1, deinterleaved_4);

    __m512i out2;
    __m512i out5;
    mm512_deinterleave(out2, out5, deinterleaved_2, deinterleaved_5);

    avx512_out0.set_at(i, out0);
    avx512_out1.set_at(i, out1);
    avx512_out2.set_at(i, out2);
    avx512_out3.set_at(i, out3);
    avx512_out4.set_at(i, out4);
    avx512_out5.set_at(i, out5);
  }
  i *= AVX512_SIZE_BYTE;

  // Remainder of soft bits.
  for (; i != K; ++i) {
    out[K * 0U + i] = in[6U * i + 0];
    out[K * 1U + i] = in[6U * i + 1];
    out[K * 2U + i] = in[6U * i + 2];
    out[K * 3U + i] = in[6U * i + 3];
    out[K * 4U + i] = in[6U * i + 4];
    out[K * 5U + i] = in[6U * i + 5];
  }
}
void ldpc_rate_dematcher_avx512_impl::deinterleave_qam256(span<log_likelihood_ratio>       out,
                                                          span<const log_likelihood_ratio> in) const
{
  unsigned E = out.size();
  unsigned K = E / 8;
  unsigned i = 0;

  // Reinterpret pointers.
  mm512::avx512_const_span avx512_in(in, E / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out0(out.subspan(0UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out1(out.subspan(1UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out2(out.subspan(2UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out3(out.subspan(3UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out4(out.subspan(4UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out5(out.subspan(5UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out6(out.subspan(6UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out7(out.subspan(7UL * K, K), K / AVX512_SIZE_BYTE);

  // Do SIMD processing.
  for (unsigned i_end = (K / AVX512_SIZE_BYTE), i_in = 0; i != i_end; ++i) {
    __m512i in0 = avx512_in.get_at(i_in++);
    __m512i in1 = avx512_in.get_at(i_in++);
    __m512i in2 = avx512_in.get_at(i_in++);
    __m512i in3 = avx512_in.get_at(i_in++);
    __m512i in4 = avx512_in.get_at(i_in++);
    __m512i in5 = avx512_in.get_at(i_in++);
    __m512i in6 = avx512_in.get_at(i_in++);
    __m512i in7 = avx512_in.get_at(i_in++);

    __m512i deinterleaved_01_even;
    __m512i deinterleaved_01_odd;
    mm512_deinterleave(deinterleaved_01_even, deinterleaved_01_odd, in0, in1);

    __m512i deinterleaved_23_even;
    __m512i deinterleaved_23_odd;
    mm512_deinterleave(deinterleaved_23_even, deinterleaved_23_odd, in2, in3);

    __m512i deinterleaved_45_even;
    __m512i deinterleaved_45_odd;
    mm512_deinterleave(deinterleaved_45_even, deinterleaved_45_odd, in4, in5);

    __m512i deinterleaved_67_even;
    __m512i deinterleaved_67_odd;
    mm512_deinterleave(deinterleaved_67_even, deinterleaved_67_odd, in6, in7);

    __m512i deinterleaved_0123_even_even;
    __m512i deinterleaved_0123_even_odd;
    mm512_deinterleave(
        deinterleaved_0123_even_even, deinterleaved_0123_even_odd, deinterleaved_01_even, deinterleaved_23_even);

    __m512i deinterleaved_4567_even_even;
    __m512i deinterleaved_4567_even_odd;
    mm512_deinterleave(
        deinterleaved_4567_even_even, deinterleaved_4567_even_odd, deinterleaved_45_even, deinterleaved_67_even);

    __m512i deinterleaved_0123_odd_even;
    __m512i deinterleaved_0123_odd_odd;
    mm512_deinterleave(
        deinterleaved_0123_odd_even, deinterleaved_0123_odd_odd, deinterleaved_01_odd, deinterleaved_23_odd);

    __m512i deinterleaved_4567_odd_even;
    __m512i deinterleaved_4567_odd_odd;
    mm512_deinterleave(
        deinterleaved_4567_odd_even, deinterleaved_4567_odd_odd, deinterleaved_45_odd, deinterleaved_67_odd);

    __m512i out0;
    __m512i out4;
    mm512_deinterleave(out0, out4, deinterleaved_0123_even_even, deinterleaved_4567_even_even);

    __m512i out1;
    __m512i out5;
    mm512_deinterleave(out1, out5, deinterleaved_0123_odd_even, deinterleaved_4567_odd_even);

    __m512i out2;
    __m512i out6;
    mm512_deinterleave(out2, out6, deinterleaved_0123_even_odd, deinterleaved_4567_even_odd);

    __m512i out3;
    __m512i out7;
    mm512_deinterleave(out3, out7, deinterleaved_0123_odd_odd, deinterleaved_4567_odd_odd);

    avx512_out0.set_at(i, out0);
    avx512_out1.set_at(i, out1);
    avx512_out2.set_at(i, out2);
    avx512_out3.set_at(i, out3);
    avx512_out4.set_at(i, out4);
    avx512_out5.set_at(i, out5);
    avx512_out6.set_at(i, out6);
    avx512_out7.set_at(i, out7);
  }
  i *= AVX512_SIZE_BYTE;

  // Remainder of soft bits.
  for (; i != K; ++i) {
    out[K * 0U + i] = in[8U * i + 0];
    out[K * 1U + i] = in[8U * i + 1];
    out[K * 2U + i] = in[8U * i + 2];
    out[K * 3U + i] = in[8U * i + 3];
    out[K * 4U + i] = in[8U * i + 4];
    out[K * 5U + i] = in[8U * i + 5];
    out[K * 6U + i] = in[8U * i + 6];
    out[K * 7U + i] = in[8U * i + 7];
  }
}

void ldpc_rate_dematcher_avx512_impl::deinterleave_qam1024(span<log_likelihood_ratio>       out,
                                                           span<const log_likelihood_ratio> in) const
{
  unsigned E = out.size();
  unsigned K = E / 10;
  unsigned i = 0;

  // Reinterpret pointers.
  mm512::avx512_const_span avx512_in(in, E / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out0(out.subspan(0UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out1(out.subspan(1UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out2(out.subspan(2UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out3(out.subspan(3UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out4(out.subspan(4UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out5(out.subspan(5UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out6(out.subspan(6UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out7(out.subspan(7UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out8(out.subspan(8UL * K, K), K / AVX512_SIZE_BYTE);
  mm512::avx512_span       avx512_out9(out.subspan(9UL * K, K), K / AVX512_SIZE_BYTE);

  // Do SIMD processing.
  for (unsigned i_end = (K / AVX512_SIZE_BYTE), i_in = 0; i != i_end; ++i) {
    __m512i in0 = avx512_in.get_at(i_in++);
    __m512i in1 = avx512_in.get_at(i_in++);
    __m512i in2 = avx512_in.get_at(i_in++);
    __m512i in3 = avx512_in.get_at(i_in++);
    __m512i in4 = avx512_in.get_at(i_in++);
    __m512i in5 = avx512_in.get_at(i_in++);
    __m512i in6 = avx512_in.get_at(i_in++);
    __m512i in7 = avx512_in.get_at(i_in++);
    __m512i in8 = avx512_in.get_at(i_in++);
    __m512i in9 = avx512_in.get_at(i_in++);

    __m512i deinterleaved_0;
    __m512i deinterleaved_1;
    __m512i deinterleaved_2;
    __m512i deinterleaved_3;
    __m512i deinterleaved_4;
    mm512_deinterleave(
        deinterleaved_0, deinterleaved_1, deinterleaved_2, deinterleaved_3, deinterleaved_4, in0, in1, in2, in3, in4);

    __m512i deinterleaved_5;
    __m512i deinterleaved_6;
    __m512i deinterleaved_7;
    __m512i deinterleaved_8;
    __m512i deinterleaved_9;
    mm512_deinterleave(
        deinterleaved_5, deinterleaved_6, deinterleaved_7, deinterleaved_8, deinterleaved_9, in5, in6, in7, in8, in9);

    __m512i out0;
    __m512i out5;
    mm512_deinterleave(out0, out5, deinterleaved_0, deinterleaved_5);

    __m512i out1;
    __m512i out6;
    mm512_deinterleave(out1, out6, deinterleaved_1, deinterleaved_6);

    __m512i out2;
    __m512i out7;
    mm512_deinterleave(out2, out7, deinterleaved_2, deinterleaved_7);

    __m512i out3;
    __m512i out8;
    mm512_deinterleave(out3, out8, deinterleaved_3, deinterleaved_8);

    __m512i out4;
    __m512i out9;
    mm512_deinterleave(out4, out9, deinterleaved_4, deinterleaved_9);

    avx512_out0.set_at(i, out0);
    avx512_out1.set_at(i, out1);
    avx512_out2.set_at(i, out2);
    avx512_out3.set_at(i, out3);
    avx512_out4.set_at(i, out4);
    avx512_out5.set_at(i, out5);
    avx512_out6.set_at(i, out6);
    avx512_out7.set_at(i, out7);
    avx512_out8.set_at(i, out8);
    avx512_out9.set_at(i, out9);
  }
  i *= AVX512_SIZE_BYTE;

  // Remainder of soft bits.
  for (; i != K; ++i) {
    out[K * 0U + i] = in[10U * i + 0];
    out[K * 1U + i] = in[10U * i + 1];
    out[K * 2U + i] = in[10U * i + 2];
    out[K * 3U + i] = in[10U * i + 3];
    out[K * 4U + i] = in[10U * i + 4];
    out[K * 5U + i] = in[10U * i + 5];
    out[K * 6U + i] = in[10U * i + 6];
    out[K * 7U + i] = in[10U * i + 7];
    out[K * 8U + i] = in[10U * i + 8];
    out[K * 9U + i] = in[10U * i + 9];
  }
}
