// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/adt/circular_vector.h"
#include "ocudu/adt/static_vector.h"
#include "ocudu/ran/rnti.h"
#include "ocudu/ran/slot_pdu_capacity_constants.h"
#include "ocudu/ran/slot_point.h"
#include "ocudu/scheduler/resource_grid_util.h"
#include <algorithm>

namespace ocudu {

/// \brief Storage of the periodic CG PUSCH opportunities to be scheduled in the resource grid.
///
/// Each position of the wheel represents a slot in a ring-like structure (i.e. slot % wheel size) and holds the list of
/// RNTIs with a CG grant due in that slot. Used by both the CG Type 1 and Type 2 schedulers.
///
/// The wheel is sized to the cell's CG period rather than to the largest one the standard allows, so every UE occupies
/// a single entry. Every UE of the cell uses the cell's CG period - the RRM builds each UE's CG configuration from the
/// cell defaults, and the UE config validator rejects a UE whose period differs - so the wheel holds that period once
/// and derives every UE's grant positions from it, rather than taking one per UE.
class cg_slot_wheel
{
public:
  /// List of UEs with a CG opportunity in a given slot.
  using rnti_list = static_vector<rnti_t, MAX_CG_PUSCH_PDUS_PER_SLOT>;

  /// \brief Outcome of add_ue(): the offsets at which the UE's grant could not be written.
  ///
  /// Either count being non-zero denotes a bookkeeping error in the caller, which is expected to report it. The wheel
  /// holds no logger of its own, so this is the only channel through which it signals one.
  struct add_ue_result {
    /// Offsets at which the UE was already present, and which were therefore left untouched.
    unsigned nof_dup = 0;
    /// Offsets that had no room left for the UE's grant, at which it was therefore dropped.
    unsigned nof_full = 0;
  };

  /// Maximum CG periodicity in slots: sym5120x14 = 5120 slots (normal CP, 14 symbols/slot).
  static constexpr unsigned max_cg_slot_periodicity = 5120U;

  /// \brief Minimum size of the wheel, in slots.
  ///
  /// It has to exceed the largest k2 (32, as per TS 38.331), so that a CG Type 2 UE is left with at least one usable
  /// occasion: the ones between the activation PDCCH and the first transmission k2 slots later are skipped.
  static constexpr unsigned min_wheel_size = 40U;

  /// \brief Size of the wheel holding a CG period of \c cg_period_slots slots.
  ///
  /// The wheel is indexed by slot modulo its size, so that size has to be a multiple of the CG period - otherwise the
  /// entries a UE is written to would not be the slots its grant actually recurs on - and it has to divide the number
  /// of slots in a hyper system frame, or the mapping would shift when the slot counter wraps. Doubling the period
  /// until it reaches the minimum preserves both: every CG periodicity is of the form 2^a or 5*2^a, so any
  /// power-of-two multiple of one still divides NOF_SLOTS_PER_HYPER_SYSTEM_FRAME.
  static constexpr unsigned ring_size_for(unsigned cg_period_slots)
  {
    unsigned ring_size = cg_period_slots;
    while (ring_size < min_wheel_size) {
      ring_size *= 2;
    }
    return ring_size;
  }

  /// \brief Builds a wheel holding one CG period.
  /// \param cg_period_slots CG periodicity of the cell, in slots. It is the period of every UE of the cell, so the
  /// wheel derives each UE's grant positions from it alone.
  explicit cg_slot_wheel(unsigned cg_period_slots) : period_slots(cg_period_slots)
  {
    ocudu_assert(cg_period_slots > 0 and cg_period_slots <= max_cg_slot_periodicity,
                 "Invalid CG periodicity of {} slots",
                 cg_period_slots);

    wheel.resize(ring_size_for(cg_period_slots));
  }

  /// \brief Adds a UE's CG grant positions to the wheel, one every CG period.
  ///
  /// The wheel size is a multiple of the CG period (see ring_size_for()), so a UE occupies more than one entry whenever
  /// the period is below \ref min_wheel_size.
  /// \return The offsets at which the UE's grant could not be written; see \ref add_ue_result.
  add_ue_result add_ue(rnti_t rnti, unsigned time_domain_offset)
  {
    add_ue_result result;
    for (unsigned offset = time_domain_offset; offset < wheel.size(); offset += period_slots) {
      rnti_list& slot_wheel = wheel[offset];
      if (std::find(slot_wheel.begin(), slot_wheel.end(), rnti) != slot_wheel.end()) {
        ++result.nof_dup;
        continue;
      }
      // The RRM admits a UE only once it has checked that the slot offset has room, so a full list here means the two
      // went out of sync. Pushing regardless would overrun the static_vector, so drop the entry instead and report it:
      // the UE loses this occasion rather than the cell losing its scheduler state.
      if (slot_wheel.full()) {
        ++result.nof_full;
        continue;
      }
      slot_wheel.push_back(rnti);
    }
    return result;
  }

  /// \brief Removes a UE's CG grant positions from the wheel.
  ///
  /// \c time_domain_offset must be the same value that was passed to add_ue(), otherwise stale entries are left behind.
  /// \return Number of offsets at which the UE was not found. This is expected to be non-zero for a UE that was never
  /// added to the wheel, e.g. a CG Type 2 UE still pending activation.
  unsigned rem_ue(rnti_t rnti, unsigned time_domain_offset)
  {
    unsigned nof_missing = 0;
    for (unsigned offset = time_domain_offset; offset < wheel.size(); offset += period_slots) {
      rnti_list& slot_wheel = wheel[offset];
      auto*      it         = std::find(slot_wheel.begin(), slot_wheel.end(), rnti);
      if (it == slot_wheel.end()) {
        ++nof_missing;
        continue;
      }
      // Swap with last element and pop for O(1) removal.
      std::swap(*it, slot_wheel.back());
      slot_wheel.pop_back();
    }
    return nof_missing;
  }

  /// \brief Removes a UE from every position of the wheel.
  ///
  /// Scans the whole wheel, so it is meant as a recovery path for when the offset the UE was added with can no longer
  /// be recovered; prefer rem_ue() otherwise.
  /// \return Number of offsets from which the UE was removed.
  unsigned rem_ue_everywhere(rnti_t rnti)
  {
    unsigned nof_removed = 0;
    for (rnti_list& slot_wheel : wheel) {
      auto* it = std::find(slot_wheel.begin(), slot_wheel.end(), rnti);
      if (it == slot_wheel.end()) {
        continue;
      }
      std::swap(*it, slot_wheel.back());
      slot_wheel.pop_back();
      ++nof_removed;
    }
    return nof_removed;
  }

  /// Returns the list of UEs with a CG opportunity in the given slot.
  const rnti_list& operator[](slot_point slot) const { return wheel[slot.count()]; }

  /// Returns whether the given UE has a CG opportunity in the given slot.
  bool contains(slot_point slot, rnti_t rnti) const
  {
    const rnti_list& slot_wheel = wheel[slot.count()];
    return std::find(slot_wheel.begin(), slot_wheel.end(), rnti) != slot_wheel.end();
  }

  /// Removes all the CG opportunities from the wheel.
  void clear()
  {
    for (rnti_list& slot_wheel : wheel) {
      slot_wheel.clear();
    }
  }

private:
  /// CG periodicity of the cell, in slots. Shared by every UE of the cell.
  unsigned                   period_slots;
  circular_vector<rnti_list> wheel;
};

} // namespace ocudu
