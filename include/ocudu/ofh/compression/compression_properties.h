// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "ocudu/ofh/compression/compression_params.h"
#include "ocudu/ran/resource_block.h"
#include "ocudu/support/math/math_utils.h"
#include "ocudu/support/units.h"
#include <cmath>

namespace ocudu {
namespace ofh {

/// Returns true if the compression parameter is present based on the given compression type.
constexpr bool is_compression_param_present(compression_type type)
{
  return (type == compression_type::BFP) || (type == compression_type::mu_law) ||
         (type == compression_type::bfp_selective) || (type == compression_type::mod_selective);
}

/// Returns size of a PRB compressed according to the given compression parameters.
inline units::bytes get_compressed_prb_size(const ru_compression_params& params)
{
  static constexpr units::bytes compr_param_size{1};

  unsigned prb_size = NOF_SUBCARRIERS_PER_RB * 2 * params.data_width;
  if (is_compression_param_present(params.type)) {
    prb_size += compr_param_size.to_bits().value();
  }
  return units::bits(prb_size).round_up_to_bytes();
}

/// \brief Returns the size of a set of beamforming weights packed with the given compression parameters.
///
/// The size comprises the optional bfwCompParam field and the bfwI and bfwQ pair of each weight, see O-RAN.WG4.CUS,
/// 7.7.1.
inline units::bytes get_packed_beamforming_weights_size(unsigned nof_weights, const ru_compression_params& params)
{
  static constexpr units::bytes compr_param_size{1};

  units::bytes size = units::bits(nof_weights * 2 * params.data_width).round_up_to_bytes();
  if (is_compression_param_present(params.type)) {
    size += compr_param_size;
  }

  return size;
}

/// \brief Returns the O-RAN FS_Offset that corresponds to the OCUDU IQ full scale for the given compression parameters.
///
/// OCUDU maps an I or Q amplitude of 1.0 to the 16-bit full scale (\f$2^{15}\f$) before compression. With BFP and an
/// \f$m\f$-bit mantissa, the O-RAN full scale is \f$FS_0 = (2^{m-1} 2^{15})^2\f$ (O-RAN.WG4.CUS, clause 8.1.3.1), i.e.,
/// a factor of \f$2^{2(m-1)}\f$ above the OCUDU full scale power. Since \f$FS = FS_0 2^{-FS\_Offset}\f$, this returns
/// an FS_Offset of \f$2(m-1)\f$.
///
/// The OCUDU full scale is therefore \f$2(m-1)\f$ FS_Offset steps of 3.01 dB below the O-RAN full scale. Without
/// compression, both full scales coincide and this returns 0.
constexpr unsigned get_native_fs_offset(const ru_compression_params& params)
{
  return ((params.type == compression_type::BFP) && (params.data_width > 0)) ? 2 * (params.data_width - 1) : 0;
}

/// Scaling of the downlink IQ data that moves the OCUDU IQ full scale to a configured O-RAN FS_Offset.
struct fs_offset_scaling {
  /// Offset added to the shared exponent of every BFP compressed PRB.
  unsigned bfp_exponent_offset = 0;
  /// Amplitude factor applied to the IQ data prior to quantization.
  float amplitude = 1.0F;
};

/// \brief Returns the downlink scaling that maps the OCUDU IQ full scale to the O-RAN full scale with the given
/// FS_Offset.
///
/// For BFP compression, the scaling is applied through the BFP exponent whenever possible, as it does not affect the
/// mantissa resolution. The remaining half-bit, or a full scale above the OCUDU one, is applied as an amplitude factor
/// that does not exceed one.
///
/// Examples:
/// | Compression | FS_Offset | Exponent offset | Amplitude                  |
/// |-------------|-----------|-----------------|----------------------------|
/// | BFP9        | 16        | 0               | 1                          |
/// | BFP9        | 0         | 8               | 1                          |
/// | BFP9        | 1         | 8               | \f$2^{-0.5}\f$ (-3.01 dB)  |
/// | BFP9        | 20        | 0               | \f$2^{-2}\f$ (-12.04 dB)   |
/// | None 16-bit | 2         | 0               | \f$2^{-1}\f$ (-6.02 dB)    |
inline fs_offset_scaling get_dl_fs_offset_scaling(const ru_compression_params& params, unsigned fs_offset)
{
  // Full scale difference in FS_Offset steps, i.e., half bits.
  int nof_steps = static_cast<int>(get_native_fs_offset(params)) - static_cast<int>(fs_offset);

  if ((params.type == compression_type::BFP) && (nof_steps > 0)) {
    int exponent_offset = divide_ceil(nof_steps, 2);
    return {static_cast<unsigned>(exponent_offset), std::exp2(0.5F * (nof_steps - 2 * exponent_offset))};
  }

  return {0, std::exp2(0.5F * nof_steps)};
}

/// \brief Returns the amplitude factor that maps decompressed uplink IQ data from the O-RAN full scale with the given
/// FS_Offset to the OCUDU IQ full scale.
inline float get_ul_fs_offset_scaling(const ru_compression_params& params, unsigned fs_offset)
{
  int nof_steps = static_cast<int>(get_native_fs_offset(params)) - static_cast<int>(fs_offset);
  return std::exp2(-0.5F * nof_steps);
}

} // namespace ofh
} // namespace ocudu
