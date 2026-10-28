// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "ocudu/ofh/compression/compression_properties.h"
#include "ocudu/support/ocudu_assert.h"
#include <array>
#include <cstdint>
#include <optional>

namespace ocudu {
namespace ofh {

/// \brief IQ data scaling of a compression type for every IQ bit width.
///
/// Combines the IQ scaling applied prior to compression with the scaling that maps the OCUDU IQ full scale to the
/// configured O-RAN FS_Offset, see \ref get_dl_fs_offset_scaling and \ref get_ul_fs_offset_scaling. The values are
/// computed once, so that the compression and decompression do not have any additional cost.
class iq_scaling_table
{
public:
  /// \brief Constructs the scaling table.
  ///
  /// \param[in] type       Compression type.
  /// \param[in] iq_scaling Scaling factor applied to IQ data prior to compression.
  /// \param[in] fs_offset  O-RAN FS_Offset of the IQ full scale. If not set, the OCUDU IQ full scale is used.
  iq_scaling_table(compression_type type, float iq_scaling, std::optional<unsigned> fs_offset)
  {
    for (unsigned data_width = 0; data_width <= MAX_IQ_WIDTH; ++data_width) {
      ru_compression_params params{type, data_width};
      fs_offset_scaling     dl_scaling =
          fs_offset.has_value() ? get_dl_fs_offset_scaling(params, *fs_offset) : fs_offset_scaling{};

      compression_iq_scaling[data_width]      = iq_scaling * dl_scaling.amplitude;
      compression_exponent_offset[data_width] = dl_scaling.bfp_exponent_offset;
      decompression_iq_scaling[data_width] =
          fs_offset.has_value() ? get_ul_fs_offset_scaling(params, *fs_offset) : 1.0F;
    }
  }

  /// Returns the scaling factor applied to IQ data prior to compression with the given IQ bit width.
  float get_compression_iq_scaling(unsigned data_width) const
  {
    ocudu_assert(data_width <= MAX_IQ_WIDTH, "Invalid IQ bit width {}", data_width);
    return compression_iq_scaling[data_width];
  }

  /// Returns the offset added to the BFP exponent of every PRB compressed with the given IQ bit width.
  uint8_t get_compression_exponent_offset(unsigned data_width) const
  {
    ocudu_assert(data_width <= MAX_IQ_WIDTH, "Invalid IQ bit width {}", data_width);
    return compression_exponent_offset[data_width];
  }

  /// Returns the scaling factor applied to IQ data decompressed from the given IQ bit width.
  float get_decompression_iq_scaling(unsigned data_width) const
  {
    ocudu_assert(data_width <= MAX_IQ_WIDTH, "Invalid IQ bit width {}", data_width);
    return decompression_iq_scaling[data_width];
  }

private:
  /// Scaling factor applied to IQ data prior to compression, indexed by IQ bit width.
  std::array<float, MAX_IQ_WIDTH + 1> compression_iq_scaling;
  /// Offset added to the BFP exponent of every compressed PRB, indexed by IQ bit width.
  std::array<uint8_t, MAX_IQ_WIDTH + 1> compression_exponent_offset;
  /// Scaling factor applied to decompressed IQ data, indexed by IQ bit width.
  std::array<float, MAX_IQ_WIDTH + 1> decompression_iq_scaling;
};

} // namespace ofh
} // namespace ocudu
