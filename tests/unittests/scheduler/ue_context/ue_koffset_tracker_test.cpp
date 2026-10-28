// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Unit test for the derivation and tracking of the Differential Koffset of a UE.

#include "lib/scheduler/ue_context/ue_koffset_tracker.h"
#include "ocudu/ran/harq_id.h"
#include <gtest/gtest.h>

using namespace ocudu;
using namespace std::chrono_literals;

TEST(differential_koffset_derivation_test, leaves_the_reported_timing_advance_and_the_margin_in_the_offset)
{
  // 40ms - (20ms T_TA + 5ms margin) = 15ms.
  ASSERT_EQ(derive_differential_koffset(40ms, 20000us, 5ms), 15ms);
}

TEST(differential_koffset_derivation_test, rounds_the_reported_timing_advance_up_to_a_whole_slot)
{
  // T_TA rounds up, which keeps more of the Koffset.
  ASSERT_EQ(derive_differential_koffset(40ms, 20001us, 5ms), 14ms);
  ASSERT_EQ(derive_differential_koffset(40ms, 19999us, 5ms), 15ms);
}

TEST(differential_koffset_derivation_test, gives_up_nothing_when_the_reported_timing_advance_fills_the_offset)
{
  ASSERT_EQ(derive_differential_koffset(40ms, 40000us, 0ms), 0ms);
  ASSERT_EQ(derive_differential_koffset(40ms, 35000us, 5ms), 0ms);
  // Never negative.
  ASSERT_EQ(derive_differential_koffset(40ms, 60000us, 5ms), 0ms);
}

TEST(differential_koffset_derivation_test, is_bounded_by_what_the_mac_ce_field_carries)
{
  // Capped by the 6-bit field, as per TS 38.321, Section 6.1.3.57.
  ASSERT_EQ(derive_differential_koffset(1023ms, 1000us, 0ms), MAX_DIFFERENTIAL_KOFFSET);
  ASSERT_EQ(MAX_DIFFERENTIAL_KOFFSET, 63ms);
}

TEST(differential_koffset_derivation_test, effective_offset_always_covers_the_reported_timing_advance)
{
  // The UE Koffset has to cover T_TA, as per TS 38.300, Section 16.14.2.1.
  constexpr std::chrono::milliseconds cell_koffset{40};
  constexpr std::chrono::milliseconds margin{5};

  for (unsigned reported_us = 0; reported_us != 60000; reported_us += 250) {
    const std::chrono::microseconds reported{reported_us};
    const std::chrono::milliseconds derived  = derive_differential_koffset(cell_koffset, reported, margin);
    const std::chrono::milliseconds reserved = std::chrono::ceil<std::chrono::milliseconds>(reported);

    if (reserved <= cell_koffset) {
      ASSERT_GE(cell_koffset - derived, reserved)
          << "reported T_TA of " << reported_us << "us left the UE without the offset it needs";
    } else {
      // The cell-specific Koffset alone is already too short.
      ASSERT_EQ(derived, std::chrono::milliseconds{0});
    }
  }
}

TEST(differential_koffset_derivation_test, effective_offset_survives_a_report_stale_by_the_reporting_threshold)
{
  // T_TA may drift by offsetThresholdTA between reports, as per TS 38.321, Section 5.4.8. The margin absorbs it.
  constexpr std::chrono::milliseconds cell_koffset{40};
  constexpr std::chrono::milliseconds offset_threshold_ta{4};
  constexpr std::chrono::milliseconds margin = offset_threshold_ta;

  for (unsigned reported_ms = 0, e = cell_koffset.count() - offset_threshold_ta.count() + 1; reported_ms != e;
       ++reported_ms) {
    const std::chrono::milliseconds reported{reported_ms};
    const std::chrono::milliseconds derived = derive_differential_koffset(cell_koffset, reported, margin);

    // Worst case before the next report.
    const std::chrono::milliseconds worst_case_ta = reported + offset_threshold_ta;
    ASSERT_GE(cell_koffset - derived, worst_case_ta)
        << "a report of " << reported_ms << "ms going stale left the UE short";
  }
}

TEST(ue_koffset_tracker_test, ue_applies_no_differential_koffset_until_one_is_signalled)
{
  ue_koffset_tracker tracker;

  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_FALSE(tracker.has_pending_ce());
}

TEST(ue_koffset_tracker_test, signalled_value_is_not_applied_when_the_mac_ce_is_sent)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);

  // Not applied before its apply slot, as per TS 38.213, Section 9.
  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_TRUE(tracker.has_pending_ce());
}

TEST(ue_koffset_tracker_test, acknowledged_value_is_applied_from_its_slot)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();

  tracker.slot_indication(apply_slot - 1, {});
  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_TRUE(tracker.is_known_at(apply_slot - 1));

  tracker.slot_indication(apply_slot, {});
  ASSERT_EQ(tracker.applied_koffset(), 15ms);
  ASSERT_FALSE(tracker.has_pending_ce());
}

TEST(ue_koffset_tracker_test, ue_is_held_back_while_the_mac_ce_has_no_reported_feedback)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);

  // A UE that decodes the MAC CE may apply its value at once, and the two values place a grant in different slots.
  ASSERT_FALSE(tracker.is_known_at(apply_slot - 30));
  ASSERT_FALSE(tracker.is_known_at(apply_slot));

  tracker.handle_ce_acked();
  ASSERT_TRUE(tracker.is_known_at(apply_slot - 30));
}

TEST(ue_koffset_tracker_test, acknowledgement_arriving_after_the_apply_slot_still_applies_the_value)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);

  // The HARQ-ACK of the PDSCH reaches the scheduler a round trip later, past the apply slot of the value it carries.
  tracker.slot_indication(apply_slot, {});
  tracker.slot_indication(apply_slot + 5, {});
  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_TRUE(tracker.has_pending_ce()) << "A value was given up before its HARQ resolved";

  tracker.handle_ce_acked();
  tracker.slot_indication(apply_slot + 6, {});
  ASSERT_EQ(tracker.applied_koffset(), 15ms);
  ASSERT_FALSE(tracker.has_pending_ce());
}

TEST(ue_koffset_tracker_test, ue_waits_for_the_uplink_of_its_previous_koffset_before_being_scheduled_again)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};
  // Uplink the UE was granted at its previous Koffset, which reaches past the apply slot.
  const slot_point last_ul_slot = apply_slot + 25;

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();
  tracker.slot_indication(apply_slot, last_ul_slot);
  ASSERT_EQ(tracker.applied_koffset(), 15ms);

  ASSERT_FALSE(tracker.is_known_at(apply_slot));
  ASSERT_FALSE(tracker.is_known_at(last_ul_slot));
  ASSERT_TRUE(tracker.is_known_at(last_ul_slot + 1));
}

TEST(ue_koffset_tracker_test, ue_with_no_uplink_granted_is_scheduled_right_after_its_koffset_grows)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();
  // Nothing was granted that the earlier Koffset places past the apply slot.
  tracker.slot_indication(apply_slot, {});

  ASSERT_EQ(tracker.applied_koffset(), 15ms);
  ASSERT_TRUE(tracker.is_known_at(apply_slot));
}

TEST(ue_koffset_tracker_test, nacked_value_waits_for_the_retransmission_that_moves_its_slot)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_nacked();
  tracker.slot_indication(apply_slot + 10, {});

  // The UE decoded nothing, and the retransmission still to come carries the same value.
  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_TRUE(tracker.has_pending_ce());
  ASSERT_TRUE(tracker.is_known_at(apply_slot + 10));
}

TEST(ue_koffset_tracker_test, value_given_up_after_the_last_retransmission_leaves_the_previous_one_in_place)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_nacked();
  // No transmission of the MAC CE reached the UE, and none will.
  tracker.handle_ce_dropped();
  tracker.slot_indication(apply_slot + 10, {});

  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_FALSE(tracker.has_pending_ce());
}

TEST(ue_koffset_tracker_test, retransmission_moves_the_slot_the_value_is_applied_from)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};
  const slot_point   retx_apply_slot = apply_slot + 20;

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_nacked();
  // The UE acts on the first transmission it decodes, as per TS 38.321, Section 5.3.2.2.
  tracker.handle_ce_retx(retx_apply_slot);

  tracker.handle_ce_acked();
  tracker.slot_indication(apply_slot, {});
  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  tracker.slot_indication(retx_apply_slot, {});
  ASSERT_EQ(tracker.applied_koffset(), 15ms);
  ASSERT_FALSE(tracker.has_pending_ce());
}

TEST(ue_koffset_tracker_test, a_reported_feedback_releases_the_ue_so_the_retransmission_can_go_out)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_nacked();

  // The retransmission needs a PDCCH of its own, so the UE cannot stay held back once a report arrives.
  ASSERT_TRUE(tracker.is_known_at(apply_slot));

  // Sending it holds the UE back again, since the UE may have decoded the earlier transmission.
  tracker.handle_ce_retx(apply_slot + 20);
  ASSERT_FALSE(tracker.is_known_at(apply_slot));
}

TEST(ue_koffset_tracker_test, a_value_the_ue_has_not_acknowledged_is_given_up_when_its_harq_is_gone)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  ASSERT_TRUE(tracker.has_unacknowledged_ce()) << "No feedback has been reported yet";

  tracker.handle_ce_nacked();
  ASSERT_TRUE(tracker.has_unacknowledged_ce());

  // A HARQ process discarded on timeout reports nothing, so the value is given up without a feedback.
  tracker.handle_ce_dropped();
  ASSERT_FALSE(tracker.has_unacknowledged_ce());
  ASSERT_FALSE(tracker.has_pending_ce());
  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_TRUE(tracker.is_known_at(apply_slot)) << "The UE is scheduled again once nothing is in flight";
}

TEST(ue_koffset_tracker_test, an_acknowledged_value_outlives_the_harq_process_that_carried_it)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  // The HARQ process is freed as soon as the UE acknowledges it, well before the apply slot.
  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();

  ASSERT_FALSE(tracker.has_unacknowledged_ce());
}

TEST(ue_koffset_tracker_test, a_second_value_is_applied_from_its_own_slot)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};
  const slot_point   next_apply_slot = apply_slot + 40;
  const slot_point   last_ul_slot    = next_apply_slot + 25;

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();
  tracker.slot_indication(apply_slot, {});
  ASSERT_EQ(tracker.applied_koffset(), 15ms);

  // Each MAC CE carries a value that replaces the previous one rather than adding to it. A larger one shortens the
  // UE uplink delay further, so the uplink granted at the smaller one is still ordered first.
  tracker.handle_ce_sent(20ms, next_apply_slot);
  tracker.handle_ce_acked();
  tracker.slot_indication(next_apply_slot, last_ul_slot);

  ASSERT_EQ(tracker.applied_koffset(), 20ms);
  ASSERT_FALSE(tracker.is_known_at(last_ul_slot));
  ASSERT_TRUE(tracker.is_known_at(last_ul_slot + 1));
}

TEST(ue_koffset_tracker_test, a_smaller_value_holds_back_no_uplink)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};
  const slot_point   next_apply_slot = apply_slot + 40;
  const slot_point   last_ul_slot    = next_apply_slot + 25;

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();
  tracker.slot_indication(apply_slot, {});

  // A smaller value leaves the UE a longer Koffset, so its uplink moves later and keeps its order on its own.
  tracker.handle_ce_sent(9ms, next_apply_slot);
  tracker.handle_ce_acked();
  tracker.slot_indication(next_apply_slot, last_ul_slot);

  ASSERT_EQ(tracker.applied_koffset(), 9ms);
  ASSERT_TRUE(tracker.is_known_at(next_apply_slot));
  ASSERT_TRUE(tracker.is_known_at(last_ul_slot));
}

TEST(ue_koffset_tracker_test, applied_value_survives_a_slot_indication_carrying_no_value_in_flight)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();
  tracker.slot_indication(apply_slot, {});
  tracker.slot_indication(apply_slot + 10, {});

  ASSERT_EQ(tracker.applied_koffset(), 15ms);
}

TEST(ue_koffset_tracker_test, reset_returns_the_ue_to_the_cell_specific_koffset)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();
  tracker.slot_indication(apply_slot, {});
  ASSERT_EQ(tracker.applied_koffset(), 15ms);

  // As on MAC reset and satellite switch, as per TS 38.321, Sections 5.12 and 5.2a.
  tracker.reset();

  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_FALSE(tracker.has_pending_ce());
}

TEST(ue_koffset_tracker_test, reset_discards_a_value_still_in_flight)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();
  tracker.reset();
  tracker.slot_indication(apply_slot, {});

  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_FALSE(tracker.has_pending_ce());
  ASSERT_TRUE(tracker.is_known_at(apply_slot)) << "The UE is held back for a value that was discarded";
}

TEST(ue_koffset_tracker_test, reset_releases_a_ue_held_back_by_its_previous_uplink)
{
  ue_koffset_tracker tracker;
  const slot_point   apply_slot{subcarrier_spacing::kHz30, 40};
  // Uplink the UE was granted at its previous Koffset, which reaches past the apply slot.
  const slot_point last_ul_slot = apply_slot + 25;

  tracker.handle_ce_sent(15ms, apply_slot);
  tracker.handle_ce_acked();
  tracker.slot_indication(apply_slot, last_ul_slot);
  ASSERT_FALSE(tracker.is_known_at(apply_slot));

  // The uplink the UE waits out belongs to a Koffset it no longer applies.
  tracker.reset();

  ASSERT_EQ(tracker.applied_koffset(), 0ms);
  ASSERT_TRUE(tracker.is_known_at(apply_slot));
}
