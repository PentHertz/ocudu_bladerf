// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Unit tests for cg_slot_wheel. Verifies where a UE's CG opportunities land, that add/remove are exact
/// inverses, and that a UE can be swept out of the wheel when the offset it was added with is no longer known.

#include "lib/scheduler/configured_grant/cg_slot_wheel.h"
#include <gtest/gtest.h>

using namespace ocudu;

namespace {

constexpr rnti_t rnti1 = to_rnti(0x4601);
constexpr rnti_t rnti2 = to_rnti(0x4602);

/// \brief Counts the offsets of the whole wheel at which \c rnti has a CG opportunity.
/// \param cg_period_slots Period the wheel was built for, from which its size follows.
unsigned count_entries(const cg_slot_wheel& wheel, rnti_t rnti, unsigned cg_period_slots)
{
  unsigned count = 0;
  for (unsigned offset = 0; offset != cg_slot_wheel::ring_size_for(cg_period_slots); ++offset) {
    if (wheel.contains(slot_point{to_numerology_value(subcarrier_spacing::kHz15), offset}, rnti)) {
      ++count;
    }
  }
  return count;
}

TEST(cg_slot_wheel_test, ue_is_added_at_every_multiple_of_the_period)
{
  constexpr unsigned offset = 3;
  constexpr unsigned period = 10;

  cg_slot_wheel wheel{period};
  ASSERT_EQ(0, wheel.add_ue(rnti1, offset).nof_dup);

  ASSERT_EQ(cg_slot_wheel::ring_size_for(period) / period, count_entries(wheel, rnti1, period));
  for (unsigned n = 0; n != cg_slot_wheel::ring_size_for(period); ++n) {
    const slot_point sl{to_numerology_value(subcarrier_spacing::kHz15), n};
    ASSERT_EQ(n % period == offset, wheel.contains(sl, rnti1)) << "unexpected wheel content at offset " << n;
  }
}

TEST(cg_slot_wheel_test, rem_ue_undoes_add_ue)
{
  constexpr unsigned offset = 7;
  constexpr unsigned period = 20;

  cg_slot_wheel wheel{period};
  ASSERT_EQ(0, wheel.add_ue(rnti1, offset).nof_dup);
  ASSERT_EQ(0, wheel.rem_ue(rnti1, offset));

  ASSERT_EQ(0, count_entries(wheel, rnti1, period));
}

TEST(cg_slot_wheel_test, adding_the_same_ue_twice_does_not_duplicate_its_grants)
{
  constexpr unsigned offset      = 5;
  constexpr unsigned period      = 40;
  constexpr unsigned nof_entries = cg_slot_wheel::ring_size_for(period) / period;

  cg_slot_wheel wheel{period};
  ASSERT_EQ(0, wheel.add_ue(rnti1, offset).nof_dup);

  // The second add reports every offset as a duplicate and leaves the wheel untouched, so a single removal suffices.
  ASSERT_EQ(nof_entries, wheel.add_ue(rnti1, offset).nof_dup);
  ASSERT_EQ(nof_entries, count_entries(wheel, rnti1, period));

  ASSERT_EQ(0, wheel.rem_ue(rnti1, offset));
  ASSERT_EQ(0, count_entries(wheel, rnti1, period));
}

TEST(cg_slot_wheel_test, removing_a_ue_that_was_never_added_reports_every_offset_as_missing)
{
  constexpr unsigned offset = 1;
  constexpr unsigned period = 80;

  cg_slot_wheel wheel{period};
  ASSERT_EQ(cg_slot_wheel::ring_size_for(period) / period, wheel.rem_ue(rnti1, offset));
}

TEST(cg_slot_wheel_test, removing_one_ue_preserves_the_grants_of_another)
{
  constexpr unsigned offset = 2;
  constexpr unsigned period = 10;

  // Both UEs share the same offset, so they sit in the same per-slot lists and a removal must not disturb the other.
  cg_slot_wheel wheel{period};
  ASSERT_EQ(0, wheel.add_ue(rnti1, offset).nof_dup);
  ASSERT_EQ(0, wheel.add_ue(rnti2, offset).nof_dup);

  ASSERT_EQ(0, wheel.rem_ue(rnti1, offset));

  ASSERT_EQ(0, count_entries(wheel, rnti1, period));
  ASSERT_EQ(cg_slot_wheel::ring_size_for(period) / period, count_entries(wheel, rnti2, period));
}

TEST(cg_slot_wheel_test, a_ue_added_to_a_full_offset_is_dropped_and_reported)
{
  constexpr unsigned offset      = 2;
  constexpr unsigned period      = 10;
  constexpr unsigned nof_entries = cg_slot_wheel::ring_size_for(period) / period;

  // Every UE added at this offset takes one place in each of its entries, so the offset is full once its per-slot
  // capacity is reached.
  cg_slot_wheel wheel{period};
  for (unsigned n = 0; n != MAX_CG_PUSCH_PDUS_PER_SLOT; ++n) {
    const cg_slot_wheel::add_ue_result res = wheel.add_ue(to_rnti(0x4601 + n), offset);
    ASSERT_EQ(0, res.nof_dup);
    ASSERT_EQ(0, res.nof_full) << "the offset ran out of room after only " << n << " UEs";
  }

  // The RRM admits a UE only once it has checked that the offset has room, so this can only happen if the two went out
  // of sync. The wheel drops the grant rather than overrunning its storage, and reports every offset it dropped it at.
  const cg_slot_wheel::add_ue_result res = wheel.add_ue(to_rnti(0x4601 + MAX_CG_PUSCH_PDUS_PER_SLOT), offset);
  EXPECT_EQ(0, res.nof_dup);
  EXPECT_EQ(nof_entries, res.nof_full);

  // The UEs already in the wheel are untouched.
  for (unsigned n = 0; n != MAX_CG_PUSCH_PDUS_PER_SLOT; ++n) {
    ASSERT_EQ(nof_entries, count_entries(wheel, to_rnti(0x4601 + n), period));
  }
  ASSERT_EQ(0, count_entries(wheel, to_rnti(0x4601 + MAX_CG_PUSCH_PDUS_PER_SLOT), period));
}

TEST(cg_slot_wheel_test, rem_ue_with_the_wrong_offset_leaves_stale_grants_that_a_sweep_reclaims)
{
  constexpr unsigned offset     = 4;
  constexpr unsigned bad_offset = 5;
  constexpr unsigned period     = 10;

  // The offsets must fall within the first period and be distinct, so that the entries rem_ue() visits are disjoint
  // from the ones add_ue() wrote and none of them is reclaimed by accident.
  static_assert(offset < period and bad_offset < period, "offsets must fall within the first period");
  static_assert(offset != bad_offset, "the wrong offset must differ from the right one");

  constexpr unsigned nof_added = cg_slot_wheel::ring_size_for(period) / period;

  cg_slot_wheel wheel{period};
  ASSERT_EQ(0, wheel.add_ue(rnti1, offset).nof_dup);
  ASSERT_EQ(nof_added, count_entries(wheel, rnti1, period));

  // Removing with an offset the UE was not added with visits entries that hold no grant of its own, so every one of
  // them is reported missing and the UE's actual entries all leak.
  ASSERT_EQ(nof_added, wheel.rem_ue(rnti1, bad_offset));
  ASSERT_EQ(nof_added, count_entries(wheel, rnti1, period));

  // A sweep reclaims exactly those, without needing to know the offset.
  ASSERT_EQ(nof_added, wheel.rem_ue_everywhere(rnti1));
  ASSERT_EQ(0, count_entries(wheel, rnti1, period));
}

TEST(cg_slot_wheel_test, sweep_removes_all_the_grants_of_one_ue_only)
{
  constexpr unsigned period      = 10;
  constexpr unsigned nof_entries = cg_slot_wheel::ring_size_for(period) / period;

  // Both UEs use the cell's CG period, which the wheel was built for; only their offsets differ.
  cg_slot_wheel wheel{period};
  ASSERT_EQ(0, wheel.add_ue(rnti1, 3).nof_dup);
  ASSERT_EQ(0, wheel.add_ue(rnti2, 8).nof_dup);

  ASSERT_EQ(nof_entries, wheel.rem_ue_everywhere(rnti1));

  ASSERT_EQ(0, count_entries(wheel, rnti1, period));
  ASSERT_EQ(nof_entries, count_entries(wheel, rnti2, period));

  // Sweeping a UE that is not in the wheel is a no-op.
  ASSERT_EQ(0, wheel.rem_ue_everywhere(rnti1));
}

TEST(cg_slot_wheel_test, clear_empties_the_whole_wheel)
{
  constexpr unsigned period = 10;

  cg_slot_wheel wheel{period};
  ASSERT_EQ(0, wheel.add_ue(rnti1, 0).nof_dup);
  ASSERT_EQ(0, wheel.add_ue(rnti2, 5).nof_dup);

  wheel.clear();

  ASSERT_EQ(0, count_entries(wheel, rnti1, period));
  ASSERT_EQ(0, count_entries(wheel, rnti2, period));
}

} // namespace
