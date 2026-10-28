// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ran/cu_types.h"
#include <unordered_set>

namespace ocudu::ocucp {

/// \brief RAN UE NGAP IDs in use by all NGAP instances of the CU-CP.
///
/// A RAN UE NGAP ID is unique within the NG-RAN node (TS 38.401 section 6.2.1). All NGAP instances share one object
/// of this class.
class ngap_ran_ue_id_allocator
{
public:
  /// Returns the next RAN UE NGAP ID that is not in use, or ran_ue_id_t::invalid if all IDs are in use.
  ran_ue_id_t allocate()
  {
    if (ids_in_use.size() == MAX_NOF_RAN_UES) {
      return ran_ue_id_t::invalid;
    }

    while (ids_in_use.count(next_ran_ue_id) != 0) {
      increase_next_ran_ue_id();
    }
    ran_ue_id_t ret = next_ran_ue_id;
    increase_next_ran_ue_id();
    return ret;
  }

  /// Marks a RAN UE NGAP ID as in use.
  void add(ran_ue_id_t ran_ue_id) { ids_in_use.insert(ran_ue_id); }

  /// Marks a RAN UE NGAP ID as free.
  void remove(ran_ue_id_t ran_ue_id) { ids_in_use.erase(ran_ue_id); }

private:
  void increase_next_ran_ue_id()
  {
    next_ran_ue_id =
        next_ran_ue_id == ran_ue_id_t::max ? ran_ue_id_t::min : uint_to_ran_ue_id(to_underlying(next_ran_ue_id) + 1);
  }

  /// Next RAN UE NGAP ID to try.
  ran_ue_id_t next_ran_ue_id = ran_ue_id_t::min;
  /// RAN UE NGAP IDs that a UE context uses.
  std::unordered_set<ran_ue_id_t> ids_in_use;
};

} // namespace ocudu::ocucp
