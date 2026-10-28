// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "channel_precoder_avx2.h"
#include "ocudu/adt/format.h"
#include <immintrin.h>

using namespace ocudu;

/// Size of an AVX2 register in complex numbers with 32-bit floating point precision.
static constexpr unsigned AVX2_CF_SIZE = 4;

/// Number of resource elements processed per iteration by the layer mapping and precoding loops.
static constexpr unsigned AVX2_LAYER_MAP_NOF_RE = 8;

/// Scaling applied to the ci8_t symbols when they are placed in the most significant byte of a 32-bit integer.
static constexpr float CI8_MSB_SCALING = 1.0F / static_cast<float>(1U << 24U);

namespace {

/// Representation of a set of complex numbers using a pair of AVX2 registers, for the real and imaginary parts.
struct simd_cf_t {
  __m256 re;
  __m256 im;

  /// Sets the registers using a complex constant.
  void set1(const cf_t a)
  {
    re = _mm256_set1_ps(a.real());
    im = _mm256_set1_ps(a.imag());
  }
};

/// Type to hold a set of complex numbers using an AVX2 register, with interleaved real and imaginary parts.
using simd_cf_interleaved = __m256;

} // namespace

/// Multiplies two sets of complex numbers stored in split real and imaginary registers.
static simd_cf_t operator*(const simd_cf_t& in, const simd_cf_t& weight)
{
  return {_mm256_fmsub_ps(in.re, weight.re, _mm256_mul_ps(in.im, weight.im)),
          _mm256_fmadd_ps(in.re, weight.im, _mm256_mul_ps(in.im, weight.re))};
}

/// Multiplies two sets of complex numbers stored in split registers and accumulates the result.
static void simd_cmac(simd_cf_t& acc, const simd_cf_t& in, const simd_cf_t& weight)
{
  acc.re = _mm256_fnmadd_ps(in.im, weight.im, _mm256_fmadd_ps(in.re, weight.re, acc.re));
  acc.im = _mm256_fmadd_ps(in.im, weight.re, _mm256_fmadd_ps(in.re, weight.im, acc.im));
}

/// Rounds single precision floating point numbers to the nearest even brain floating point in the upper 16 bits.
static __m256i simd_f_round_to_bf16(__m256 in)
{
  const __m256i bias = _mm256_set1_epi32(0x7fff);
  const __m256i one  = _mm256_set1_epi32(0x1);

  __m256i a_i32 = _mm256_castps_si256(in);
  return _mm256_add_epi32(a_i32, _mm256_add_epi32(bias, _mm256_and_si256(_mm256_srli_epi32(a_i32, 16), one)));
}

/// Converts a register with interleaved complex numbers into four consecutive c.
static __m128i simd_cf_to_cbf16(simd_cf_interleaved in)
{
  // Round to nearest even and shift right 16 bits.
  __m256i a_i32 = _mm256_srai_epi32(simd_f_round_to_bf16(in), 16);

  // Pack both parts in 32-bit registers.
  return _mm_packs_epi32(_mm256_extractf128_si256(a_i32, 0), _mm256_extractf128_si256(a_i32, 1));
}

/// Converts two registers with interleaved complex numbers into eight consecutive cbf16_t.
static __m256i simd_cf_to_cbf16(simd_cf_interleaved in0, simd_cf_interleaved in1)
{
  // The logical shift leaves values within the unsigned 16-bit range, so the unsigned saturation is lossless.
  __m256i packed = _mm256_packus_epi32(_mm256_srli_epi32(simd_f_round_to_bf16(in0), 16),
                                       _mm256_srli_epi32(simd_f_round_to_bf16(in1), 16));

  // Restore the order of the 64-bit words after the in-lane packing.
  return _mm256_permute4x64_epi64(packed, 0xd8);
}

/// Converts complex numbers stored in split real and imaginary registers into eight consecutive cbf16_t.
static __m256i simd_cf_to_cbf16(const simd_cf_t& in)
{
  // The real part goes to the lower 16 bits and the imaginary part to the upper 16 bits of each 32-bit word.
  return _mm256_blend_epi16(_mm256_srli_epi32(simd_f_round_to_bf16(in.re), 16), simd_f_round_to_bf16(in.im), 0xaa);
}

void channel_precoder_avx2::apply_precoding_port(span<cbf16_t>             port_re,
                                                 const re_buffer_reader<>& input_re,
                                                 span<const cf_t>          port_weights) const
{
  unsigned nof_re     = input_re.get_nof_re();
  unsigned nof_layers = input_re.get_nof_slices();

  // Create a list of all the input layer RE views.
  std::array<span<const cf_t>, precoding_constants::MAX_NOF_LAYERS> layer_re_view_list;

  // Array holding SIMD registers initialized with the precoding weights.
  std::array<simd_cf_t, precoding_constants::MAX_NOF_LAYERS> port_weights_simd;

  for (unsigned i_layer = 0; i_layer != nof_layers; ++i_layer) {
    // Fill the RE view list.
    layer_re_view_list[i_layer] = input_re.get_slice(i_layer);

    // Set the SIMD registers with the port weights.
    port_weights_simd[i_layer].set1(port_weights[i_layer]);
  }

  // Applies the port weights to AVX2_CF_SIZE consecutive RE. The products by the real and imaginary parts of the
  // weights are accumulated separately and combined at the end.
  auto precode = [&layer_re_view_list, &port_weights_simd, nof_layers](unsigned i_re) {
    simd_cf_interleaved re_in  = _mm256_loadu_ps(reinterpret_cast<const float*>(&layer_re_view_list[0][i_re]));
    simd_cf_interleaved acc_re = _mm256_mul_ps(re_in, port_weights_simd[0].re);
    simd_cf_interleaved acc_im = _mm256_mul_ps(_mm256_shuffle_ps(re_in, re_in, 0xb1), port_weights_simd[0].im);

    for (unsigned i_layer = 1; i_layer != nof_layers; ++i_layer) {
      re_in  = _mm256_loadu_ps(reinterpret_cast<const float*>(&layer_re_view_list[i_layer][i_re]));
      acc_re = _mm256_fmadd_ps(re_in, port_weights_simd[i_layer].re, acc_re);
      acc_im = _mm256_fmadd_ps(_mm256_shuffle_ps(re_in, re_in, 0xb1), port_weights_simd[i_layer].im, acc_im);
    }

    return _mm256_addsub_ps(acc_re, acc_im);
  };

  unsigned i_re = 0;

  for (unsigned max_re = (nof_re / (2 * AVX2_CF_SIZE)) * (2 * AVX2_CF_SIZE); i_re != max_re; i_re += 2 * AVX2_CF_SIZE) {
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(&port_re[i_re]),
                        simd_cf_to_cbf16(precode(i_re), precode(i_re + AVX2_CF_SIZE)));
  }

  if (nof_re - i_re >= AVX2_CF_SIZE) {
    _mm_storeu_si128(reinterpret_cast<__m128i*>(&port_re[i_re]), simd_cf_to_cbf16(precode(i_re)));
    i_re += AVX2_CF_SIZE;
  }

  for (; i_re != nof_re; ++i_re) {
    cf_t sum = layer_re_view_list[0][i_re] * port_weights[0];

    for (unsigned i_layer = 1; i_layer != nof_layers; ++i_layer) {
      sum += layer_re_view_list[i_layer][i_re] * port_weights[i_layer];
    }
    port_re[i_re] = sum;
  }
}

/// Loads two unaligned 128-bit words into the lower and higher lanes of an AVX2 register.
static __m256i simd_loadu2_si128(const int8_t* hi, const int8_t* lo)
{
  return _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128(reinterpret_cast<const __m128i*>(lo))),
                                 _mm_loadu_si128(reinterpret_cast<const __m128i*>(hi)),
                                 1);
}

/// \brief Converts the ci8_t symbol in the lower 16 bits of each 32-bit word into split complex floats.
///
/// Each byte is moved to the most significant byte of the 32-bit word and converted to float. The result is scaled
/// by \ref CI8_MSB_SCALING, which is compensated in the precoding weights.
static simd_cf_t simd_ci8_lo_to_cf(__m256i in)
{
  const __m256i mask = _mm256_set1_epi32(0xff000000);
  return {_mm256_cvtepi32_ps(_mm256_slli_epi32(in, 24)),
          _mm256_cvtepi32_ps(_mm256_and_si256(_mm256_slli_epi32(in, 16), mask))};
}

/// Converts the ci8_t symbol in the upper 16 bits of each 32-bit word into split complex floats.
static simd_cf_t simd_ci8_hi_to_cf(__m256i in)
{
  const __m256i mask = _mm256_set1_epi32(0xff000000);
  return {_mm256_cvtepi32_ps(_mm256_and_si256(_mm256_slli_epi32(in, 8), mask)),
          _mm256_cvtepi32_ps(_mm256_and_si256(in, mask))};
}

/// Applies layer mapping for one layer and converts eight symbols to split complex floats.
static void simd_layer1_map_and_ci8_to_cf(simd_cf_t* out, const ci8_t* in)
{
  out[0] = simd_ci8_lo_to_cf(_mm256_cvtepu16_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i*>(in))));
}

/// Applies layer mapping for two layers and converts eight RE to split complex floats.
static void simd_layer2_map_and_ci8_to_cf(simd_cf_t* out, const ci8_t* in)
{
  // Each 32-bit word already contains the symbols of both layers for one RE.
  __m256i tmp = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(in));

  out[0] = simd_ci8_lo_to_cf(tmp);
  out[1] = simd_ci8_hi_to_cf(tmp);
}

/// Applies layer mapping for three layers and converts eight RE to split complex floats.
static void simd_layer3_map_and_ci8_to_cf(simd_cf_t* out, const ci8_t* in)
{
  const int8_t* in8 = reinterpret_cast<const int8_t*>(in);

  // Load REs 0, 1, 4 and 5 starting at the beginning of each lane, and REs 2, 3, 6 and 7 starting four bytes into each
  // lane. The second load ends on the last input byte, so no data is read past the input.
  __m256i x = simd_loadu2_si128(in8 + 24, in8);
  __m256i y = simd_loadu2_si128(in8 + 32, in8 + 8);

  // Arrange each lane as the symbols of layers 0 and 1 of two consecutive RE followed by the symbols of layer 2. The
  // indices of y are offset by four bytes, and the zeroing indices keep the most significant bit set after the offset.
  const __m256i idx_x =
      _mm256_broadcastsi128_si256(_mm_setr_epi8(0, 1, 2, 3, 6, 7, 8, 9, 4, 5, -128, -128, 10, 11, -128, -128));
  const __m256i idx_y = _mm256_add_epi8(idx_x, _mm256_set1_epi8(4));
  x                   = _mm256_shuffle_epi8(x, idx_x);
  y                   = _mm256_shuffle_epi8(y, idx_y);

  // Gather layers 0 and 1, and layer 2, for REs 0 to 7.
  __m256  xf  = _mm256_castsi256_ps(x);
  __m256  yf  = _mm256_castsi256_ps(y);
  __m256i l01 = _mm256_castps_si256(_mm256_shuffle_ps(xf, yf, 0x44));
  __m256i l2  = _mm256_castps_si256(_mm256_shuffle_ps(xf, yf, 0xee));

  out[0] = simd_ci8_lo_to_cf(l01);
  out[1] = simd_ci8_hi_to_cf(l01);
  out[2] = simd_ci8_lo_to_cf(l2);
}

/// Applies layer mapping for four layers and converts eight RE to split complex floats.
static void simd_layer4_map_and_ci8_to_cf(simd_cf_t* out, const ci8_t* in)
{
  const int8_t* in8 = reinterpret_cast<const int8_t*>(in);

  // Load REs 0, 1, 4 and 5, and REs 2, 3, 6 and 7, so that the in-lane shuffles return the RE in order.
  __m256 a = _mm256_castsi256_ps(simd_loadu2_si128(in8 + 32, in8));
  __m256 b = _mm256_castsi256_ps(simd_loadu2_si128(in8 + 48, in8 + 16));

  // Gather layers 0 and 1, and layers 2 and 3, for REs 0 to 7.
  __m256i l01 = _mm256_castps_si256(_mm256_shuffle_ps(a, b, 0x88));
  __m256i l23 = _mm256_castps_si256(_mm256_shuffle_ps(a, b, 0xdd));

  out[0] = simd_ci8_lo_to_cf(l01);
  out[1] = simd_ci8_hi_to_cf(l01);
  out[2] = simd_ci8_lo_to_cf(l23);
  out[3] = simd_ci8_hi_to_cf(l23);
}

/// Applies layer mapping and precoding to all the RE that fill complete AVX2 registers.
template <unsigned NofLayers, typename LayerMapFunction>
static unsigned simd_layer_map_and_precode(span<span<cbf16_t>> outputs,
                                           span<const ci8_t>   input,
                                           const simd_cf_t (*weights)[precoding_constants::MAX_NOF_LAYERS],
                                           LayerMapFunction&& layer_map)
{
  unsigned nof_re   = outputs.front().size();
  unsigned i_re_end = (nof_re / AVX2_LAYER_MAP_NOF_RE) * AVX2_LAYER_MAP_NOF_RE;

  for (unsigned i_re = 0; i_re != i_re_end; i_re += AVX2_LAYER_MAP_NOF_RE) {
    // Layer map and convert to complex float.
    simd_cf_t in[NofLayers];
    layer_map(in, &input[NofLayers * i_re]);

    for (unsigned i_port = 0, nof_ports = outputs.size(); i_port != nof_ports; ++i_port) {
      // Apply precoding.
      simd_cf_t result = in[0] * weights[i_port][0];
      for (unsigned i_layer = 1; i_layer != NofLayers; ++i_layer) {
        simd_cmac(result, in[i_layer], weights[i_port][i_layer]);
      }

      // Store.
      _mm256_storeu_si256(reinterpret_cast<__m256i*>(&outputs[i_port][i_re]), simd_cf_to_cbf16(result));
    }
  }

  return i_re_end;
}

void channel_precoder_avx2::apply_layer_map_and_precoding(re_buffer_writer<cbf16_t>&     output,
                                                          span<const ci8_t>              input,
                                                          const precoding_weight_matrix& precoding) const
{
  unsigned nof_re     = output.get_nof_re();
  unsigned nof_layers = precoding.get_nof_layers();
  unsigned nof_ports  = precoding.get_nof_ports();

  // Registers to load the precoding weights, scaled to compensate the ci8_t to float conversion.
  simd_cf_t weights[precoding_constants::MAX_NOF_PORTS][precoding_constants::MAX_NOF_LAYERS];

  // Views to store the precoded symbols.
  std::array<span<cbf16_t>, precoding_constants::MAX_NOF_PORTS> outputs;

  for (unsigned i_port = 0; i_port != nof_ports; ++i_port) {
    span<const cf_t> port_coeff = precoding.get_port_coefficients(i_port);
    outputs[i_port]             = output.get_slice(i_port);
    for (unsigned i_layer = 0; i_layer != nof_layers; ++i_layer) {
      weights[i_port][i_layer].set1(port_coeff[i_layer] * CI8_MSB_SCALING);
    }
  }

  span<span<cbf16_t>> port_outputs = span<span<cbf16_t>>(outputs).first(nof_ports);

  unsigned i_re = 0;
  switch (nof_layers) {
    case 1:
      i_re = simd_layer_map_and_precode<1>(port_outputs, input, weights, simd_layer1_map_and_ci8_to_cf);
      break;
    case 2:
      i_re = simd_layer_map_and_precode<2>(port_outputs, input, weights, simd_layer2_map_and_ci8_to_cf);
      break;
    case 3:
      i_re = simd_layer_map_and_precode<3>(port_outputs, input, weights, simd_layer3_map_and_ci8_to_cf);
      break;
    case 4:
      i_re = simd_layer_map_and_precode<4>(port_outputs, input, weights, simd_layer4_map_and_ci8_to_cf);
      break;
    default:
      break;
  }

  // Process remaining symbols.
  for (unsigned i_port = 0; i_port != nof_ports; ++i_port) {
    span<const cf_t> port_weights = precoding.get_port_coefficients(i_port);
    span<cbf16_t>    port_re      = outputs[i_port];

    for (unsigned i_re_tail = i_re; i_re_tail != nof_re; ++i_re_tail) {
      cf_t sum = to_cf(input[nof_layers * i_re_tail]) * port_weights[0];
      for (unsigned i_layer = 1; i_layer != nof_layers; ++i_layer) {
        sum += to_cf(input[nof_layers * i_re_tail + i_layer]) * port_weights[i_layer];
      }
      port_re[i_re_tail] = sum;
    }
  }
}
