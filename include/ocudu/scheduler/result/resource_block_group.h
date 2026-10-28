// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/adt/bounded_bitset.h"
#include "ocudu/ran/resource_allocation/rb_interval.h"
#include "ocudu/ran/resource_block.h"
#include "ocudu/support/enum_utils.h"
#include "ocudu/support/math/math_utils.h"
#include "ocudu/support/ocudu_assert.h"

namespace ocudu {

/// \brief RBG size types for PDSCH.
/// \remark See TS 38.331, "rbg-Size".
enum class rbg_size : uint8_t { config1, config2 };

/// Nominal RBG-size (P value).
/// \remark See TS 38.214, Sections 5.1.2.2 and 6.1.2.2.
enum class nominal_rbg_size : uint8_t { P2 = 2, P4 = 4, P8 = 8, P16 = 16 };

/// Bitset of RBGs with size up to \c MAX_NOF_RBGS.
using rbg_bitmap = bounded_bitset<MAX_NOF_RBGS, true>;

/// Calculates Nominal RBG size (P) based on the BWP number of PRBs and configuration type.
/// \remark See TS 38.214, Table 5.1.2.2.1-1 and 6.1.2.2.1-1.
inline nominal_rbg_size get_nominal_rbg_size(unsigned bwp_nof_prb, rbg_size rbg_sz)
{
  ocudu_assert(bwp_nof_prb > 0 and bwp_nof_prb <= MAX_NOF_PRBS, "Invalid BWP size");
  const bool is_config_1 = rbg_sz == rbg_size::config1;
  if (bwp_nof_prb <= 36) {
    return is_config_1 ? nominal_rbg_size::P2 : nominal_rbg_size::P4;
  }
  if (bwp_nof_prb <= 72) {
    return is_config_1 ? nominal_rbg_size::P4 : nominal_rbg_size::P8;
  }
  if (bwp_nof_prb <= 144) {
    return is_config_1 ? nominal_rbg_size::P8 : nominal_rbg_size::P16;
  }
  return nominal_rbg_size::P16;
}

/// Calculates the total number of RBGs for a bandwidth part.
/// \remark See TS 38.214, Sections 5.1.2.2.1 and 6.1.2.2.1.
inline unsigned get_nof_rbgs(crb_interval bwp_rb_dims, nominal_rbg_size P)
{
  return divide_ceil(bwp_rb_dims.length() + (bwp_rb_dims.start() % to_underlying(P)), to_underlying(P));
}
inline unsigned get_nof_rbgs(crb_interval bwp_rb_dims, rbg_size rbg_sz)
{
  return get_nof_rbgs(bwp_rb_dims, get_nominal_rbg_size(bwp_rb_dims.length(), rbg_sz));
}

/// Calculates size of indexed RBG in number of RBs.
/// \remark See TS 38.214, Sections 5.1.2.2.1 and 6.1.2.2.1.
inline unsigned get_rbg_size(crb_interval bwp_rbs, nominal_rbg_size P, unsigned rbg_idx)
{
  unsigned nof_rbgs = get_nof_rbgs(bwp_rbs, P);
  ocudu_sanity_check(rbg_idx < nof_rbgs, "RBG index out-of-bounds ({} >= {})", rbg_idx, nof_rbgs);
  if (rbg_idx == 0) {
    return to_underlying(P) - (bwp_rbs.start() % to_underlying(P));
  }
  if (rbg_idx == nof_rbgs - 1) {
    unsigned ret = bwp_rbs.stop() % to_underlying(P);
    return ret > 0 ? ret : to_underlying(P);
  }
  return to_underlying(P);
}

} // namespace ocudu
