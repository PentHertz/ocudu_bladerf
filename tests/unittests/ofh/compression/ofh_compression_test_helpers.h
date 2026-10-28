// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "ocudu/adt/bf16.h"
#include "ocudu/adt/complex.h"
#include "ocudu/adt/span.h"
#include "ocudu/ofh/compression/compression_params.h"
#include "ocudu/ran/resource_block.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

/// \file
/// \brief Reference implementation of the O-RAN IQ data and beamforming weights compression.
///
/// It is written directly from O-RAN.WG4.CUS, Annex A.1 (block floating point) and Annex D.1 (bit ordering), and it is
/// independent of the library implementation.

namespace ocudu {
namespace ofh {
namespace testing {

/// Number of quantized values (I and Q) in a resource block.
constexpr unsigned nof_values_per_prb = NOF_SUBCARRIERS_PER_RB * 2;

/// Returns the quantization gain, i.e., the full scale, of the given bit width.
inline float get_quantizer_gain_reference(unsigned bit_width)
{
  return static_cast<float>(1U << (bit_width - 1U)) - 1.0F;
}

/// \brief Quantizes an already scaled IQ value, given in units of the quantization step.
///
/// Rounds to the nearest integer with ties to even and saturates to the symmetric full scale of the given gain.
/// \remark For 16 bits, the negative values saturate at -32768.
inline int16_t quantize_scaled_reference(float scaled_value, float gain)
{
  float lower_bound = (gain == get_quantizer_gain_reference(MAX_IQ_WIDTH)) ? (-gain - 1.0F) : -gain;
  return static_cast<int16_t>(std::clamp(std::nearbyint(scaled_value), lower_bound, gain));
}

/// \brief Quantizes a beamforming weight into a fixed point value of the given bit width.
///
/// The weight is clipped at +/-1 and rounded to the nearest integer with ties to even.
inline int16_t quantize_weight_reference(float value, unsigned bit_width)
{
  return static_cast<int16_t>(std::nearbyint(std::clamp(value, -1.0F, 1.0F) * get_quantizer_gain_reference(bit_width)));
}

/// Appends the given number of bits of a value to the buffer, most significant bit first.
inline void append_bits_reference(std::vector<uint8_t>& buffer, unsigned& bit_pos, uint16_t value, unsigned nof_bits)
{
  for (unsigned i = 0; i != nof_bits; ++i) {
    if ((value >> (nof_bits - 1U - i)) & 1U) {
      buffer[bit_pos / 8U] |= static_cast<uint8_t>(0x80U >> (bit_pos % 8U));
    }
    ++bit_pos;
  }
}

/// Determines the shared exponent of a compression block following O-RAN.WG4.CUS, Annex A.1.2.
inline uint8_t determine_exponent_reference(span<const int16_t> quantized, unsigned iq_width)
{
  int max_v = *std::max_element(quantized.begin(), quantized.end());
  int min_v = *std::min_element(quantized.begin(), quantized.end());

  // The most significant bit of a negative value can be one higher.
  int max_value = std::max(max_v, std::abs(min_v) - 1);
  if (max_value <= 0) {
    return 0;
  }

  int raw_exp = static_cast<int>(std::floor(std::log2(static_cast<double>(max_value)))) + 1;

  return static_cast<uint8_t>(std::max(raw_exp - static_cast<int>(iq_width) + 1, 0));
}

/// \brief Compresses one block of quantized values and appends it to the buffer.
///
/// For BFP, the block is preceded by the compression parameter, i.e., the shared exponent plus the given offset, and
/// the values are quantized to 16 bits. Without compression, the values are already quantized to the given width.
inline void compress_block_reference(std::vector<uint8_t>&        buffer,
                                     unsigned&                    bit_pos,
                                     span<const int16_t>          quantized,
                                     const ru_compression_params& params,
                                     uint8_t                      exponent_offset = 0)
{
  uint8_t exponent = 0;
  if (params.type == compression_type::BFP) {
    exponent = determine_exponent_reference(quantized, params.data_width);
    append_bits_reference(buffer, bit_pos, exponent + exponent_offset, 8);
  }

  uint16_t mask = static_cast<uint16_t>((1U << params.data_width) - 1U);
  for (int16_t value : quantized) {
    append_bits_reference(buffer, bit_pos, static_cast<uint16_t>(value >> exponent) & mask, params.data_width);
  }
}

/// Returns the size in bytes of a compressed resource block.
inline unsigned get_compressed_prb_size_reference(const ru_compression_params& params)
{
  unsigned nof_bits = nof_values_per_prb * params.data_width;
  if (params.type == compression_type::BFP) {
    nof_bits += 8;
  }
  return (nof_bits + 7) / 8;
}

/// Scaling that moves the OCUDU IQ full scale to an O-RAN FS_Offset (O-RAN.WG4.CUS, clause 8.1.3.1).
struct fs_offset_scaling_reference {
  /// Offset added to the BFP exponent of every PRB in downlink.
  uint8_t exponent_offset = 0;
  /// Amplitude factor applied to the downlink IQ data prior to quantization.
  float dl_amplitude = 1.0F;
  /// Amplitude factor applied to the decompressed uplink IQ data.
  float ul_amplitude = 1.0F;
};

/// \brief Computes the FS_Offset scaling.
///
/// OCUDU maps an amplitude of 1.0 to the 16-bit full scale before compression. For BFP with an \f$m\f$-bit mantissa,
/// this is the O-RAN full scale with FS_Offset \f$2(m-1)\f$. The downlink applies whole bits through the exponent and
/// the remaining 3.01 dB step as an amplitude that does not exceed one.
inline fs_offset_scaling_reference get_fs_offset_scaling_reference(const ru_compression_params& params,
                                                                   std::optional<unsigned>      fs_offset)
{
  if (!fs_offset.has_value()) {
    return {};
  }

  bool is_bfp    = (params.type == compression_type::BFP);
  int  native    = is_bfp ? 2 * (static_cast<int>(params.data_width) - 1) : 0;
  int  nof_steps = native - static_cast<int>(*fs_offset);

  fs_offset_scaling_reference scaling;
  scaling.ul_amplitude = std::exp2(-0.5F * nof_steps);
  if (is_bfp && (nof_steps > 0)) {
    int exponent_offset     = (nof_steps + 1) / 2;
    scaling.exponent_offset = static_cast<uint8_t>(exponent_offset);
    scaling.dl_amplitude    = std::exp2(0.5F * (nof_steps - 2 * exponent_offset));
    return scaling;
  }

  scaling.dl_amplitude = std::exp2(0.5F * nof_steps);
  return scaling;
}

/// \brief Compresses IQ data resource block by resource block, following O-RAN.WG4.CUS, Annex A.1.2 and Annex D.1.
///
/// \param[in] iq_data    IQ data, an integer number of resource blocks.
/// \param[in] params     Compression parameters.
/// \param[in] iq_scaling Scaling applied to the IQ data prior to quantization.
/// \param[in] fs_offset  O-RAN FS_Offset. If not set, the OCUDU IQ full scale is used.
/// \return The compressed resource blocks.
inline std::vector<uint8_t> compress_iq_reference(span<const cbf16_t>          iq_data,
                                                  const ru_compression_params& params,
                                                  float                        iq_scaling,
                                                  std::optional<unsigned>      fs_offset)
{
  fs_offset_scaling_reference fs_scaling = get_fs_offset_scaling_reference(params, fs_offset);

  // BFP quantizes to 16 bits and then shifts the values by the shared exponent.
  unsigned quantization_width = (params.type == compression_type::BFP) ? MAX_IQ_WIDTH : params.data_width;
  float    gain               = get_quantizer_gain_reference(quantization_width);

  // The scaling factors are combined in the same order as in the library, so that the rounding ties are the same.
  float scale = gain * (iq_scaling * fs_scaling.dl_amplitude);

  unsigned             nof_prbs = iq_data.size() / NOF_SUBCARRIERS_PER_RB;
  std::vector<uint8_t> compressed(nof_prbs * get_compressed_prb_size_reference(params), 0);
  unsigned             bit_pos = 0;

  for (unsigned i_prb = 0; i_prb != nof_prbs; ++i_prb) {
    std::vector<int16_t> quantized;
    for (const cbf16_t& sample : iq_data.subspan(i_prb * NOF_SUBCARRIERS_PER_RB, NOF_SUBCARRIERS_PER_RB)) {
      quantized.push_back(quantize_scaled_reference(to_float(sample.real) * scale, gain));
      quantized.push_back(quantize_scaled_reference(to_float(sample.imag) * scale, gain));
    }
    compress_block_reference(compressed, bit_pos, quantized, params, fs_scaling.exponent_offset);
  }

  return compressed;
}

/// Reads the given number of bits from the buffer, most significant bit first, and sign extends them.
inline int read_signed_bits_reference(span<const uint8_t> buffer, unsigned& bit_pos, unsigned nof_bits)
{
  int value = 0;
  for (unsigned i = 0; i != nof_bits; ++i) {
    value = (value << 1) | ((buffer[bit_pos / 8U] >> (7U - bit_pos % 8U)) & 1U);
    ++bit_pos;
  }
  if (value >= (1 << (nof_bits - 1U))) {
    value -= (1 << nof_bits);
  }
  return value;
}

/// \brief Decompresses IQ data resource block by resource block, following O-RAN.WG4.CUS, Annex A.1.3.
///
/// The BFP exponent is the four least significant bits of the compression parameter, the others are reserved.
///
/// \param[in] compressed Compressed resource blocks.
/// \param[in] params     Compression parameters.
/// \param[in] fs_offset  O-RAN FS_Offset. If not set, the OCUDU IQ full scale is used.
/// \return The decompressed IQ data, where 1.0 is the OCUDU IQ full scale.
inline std::vector<cf_t> decompress_iq_reference(span<const uint8_t>          compressed,
                                                 const ru_compression_params& params,
                                                 std::optional<unsigned>      fs_offset)
{
  bool   is_bfp       = (params.type == compression_type::BFP);
  double gain         = get_quantizer_gain_reference(is_bfp ? MAX_IQ_WIDTH : params.data_width);
  double ul_amplitude = get_fs_offset_scaling_reference(params, fs_offset).ul_amplitude;

  unsigned          prb_size = get_compressed_prb_size_reference(params);
  unsigned          nof_prbs = compressed.size() / prb_size;
  std::vector<cf_t> iq_data;

  for (unsigned i_prb = 0; i_prb != nof_prbs; ++i_prb) {
    span<const uint8_t> prb      = compressed.subspan(i_prb * prb_size, prb_size);
    unsigned            bit_pos  = 0;
    double              exponent = 0;
    if (is_bfp) {
      exponent = prb[0] & 0x0fU;
      bit_pos  = 8;
    }

    double scale = std::exp2(exponent) * ul_amplitude / gain;
    for (unsigned i_re = 0; i_re != NOF_SUBCARRIERS_PER_RB; ++i_re) {
      int re = read_signed_bits_reference(prb, bit_pos, params.data_width);
      int im = read_signed_bits_reference(prb, bit_pos, params.data_width);
      iq_data.emplace_back(static_cast<float>(re * scale), static_cast<float>(im * scale));
    }
  }

  return iq_data;
}

} // namespace testing
} // namespace ofh
} // namespace ocudu
