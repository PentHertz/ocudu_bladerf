// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Unit test for the scheduling of a UE that applies a UE-specific Koffset.

#include "tests/test_doubles/scheduler/cell_config_builder_profiles.h"
#include "tests/test_doubles/scheduler/scheduler_config_helper.h"
#include "tests/unittests/scheduler/test_utils/result_test_helpers.h"
#include "tests/unittests/scheduler/test_utils/scheduler_test_simulator.h"
#include "ocudu/ran/du_types.h"
#include <algorithm>
#include <gtest/gtest.h>
#include <optional>
#include <variant>

using namespace ocudu;

/// Cell-specific Koffset.
constexpr std::chrono::milliseconds cell_koffset{20};
/// Margin added to the reported T_TA.
constexpr std::chrono::milliseconds margin{2};
/// T_TA the UE reports. With the margin, it reserves 12ms of the 20ms Koffset.
constexpr std::chrono::microseconds reported_ul_ta{10000};
/// Expected Differential Koffset in 15kHz slots.
constexpr unsigned expected_differential_slots = 8;
/// Slots within which the Differential Koffset MAC CE has to be scheduled once the UE reports its T_TA.
constexpr unsigned max_slots_to_koffset_ce = 40;
/// Slots run before the UE reports its T_TA, long enough for its uplink and HARQ-ACKs to settle on the cell-specific
/// Koffset.
constexpr unsigned nof_slots_at_cell_koffset = 40;
/// Slots run while the Differential Koffset MAC CE is in flight and not acknowledged yet.
constexpr unsigned nof_slots_during_transition = 10;
/// Slots run once the MAC CE is scheduled. They cover its HARQ-ACK, the three subframes up to the apply slot and the
/// uplink the UE waits out at its previous Koffset.
constexpr unsigned nof_slots_at_ue_koffset = 80;
/// Slots run without a T_TA report.
constexpr unsigned nof_slots_without_ta_report = 100;
/// \brief Slots run while the UE is held back from a Differential Koffset no feedback ever confirms.
///
/// Long enough for the HARQ carrying the MAC CE to time out, which is what gives the value up. The timeout is
/// \c scheduler_expert_config::dl_harq_retx_timeout.
constexpr unsigned nof_slots_to_unconfirmed_give_up = 300;
/// T_TA above \c reported_ul_ta, which derives a smaller Koffset.
constexpr std::chrono::microseconds grown_ul_ta{15000};
/// T_TA below \c reported_ul_ta, which derives a larger Koffset.
constexpr std::chrono::microseconds shrunk_ul_ta{5000};

/// Returns the simulator configuration of an NTN cell with the cell-specific Koffset of this test.
static scheduler_test_sim_config make_sim_config()
{
  scheduler_test_sim_config cfg;
  cfg.max_scs        = subcarrier_spacing::kHz15;
  cfg.auto_uci       = true;
  cfg.auto_crc       = true;
  cfg.ntn_cs_koffset = cell_koffset;
  return cfg;
}

namespace {

/// Fixture of an NTN cell deriving UE-specific Koffsets, with one UE that accepts them and keeps its buffers full.
class scheduler_ue_koffset_tester : public scheduler_test_simulator, public ::testing::Test
{
protected:
  explicit scheduler_ue_koffset_tester(std::chrono::milliseconds koffset_margin = margin) :
    scheduler_test_simulator(make_sim_config())
  {
    params = cell_config_builder_profiles::create(duplex_mode::FDD);

    auto cell_cfg_req = sched_config_helper::make_default_sched_cell_configuration_request(params);
    cell_cfg_req.ran.ntn_params.emplace();
    cell_cfg_req.ran.ntn_params->ntn_cfg.cell_specific_koffset = cell_koffset;
    cell_cfg_req.ran.ntn_params->ue_koffset_cfg                = ue_koffset_config{koffset_margin};
    add_cell(cell_cfg_req);

    auto ue_req = sched_config_helper::create_default_sched_ue_creation_request(cell_cfg(to_du_cell_index(0)).params,
                                                                                {ue_drb_lcid});
    ue_req.cfg.ue_specific_koffset_supported = true;
    crnti                                    = ue_req.crnti;
    add_ue(ue_req);

    refill_buffers();
  }

  /// Keeps the DL and UL buffers full.
  void refill_buffers()
  {
    constexpr unsigned buffer_bytes = std::numeric_limits<unsigned>::max() / 2;
    if (fill_dl_buffer) {
      push_dl_buffer_state(dl_buffer_state_indication_message{ue_idx, ue_drb_lcid, buffer_bytes});
    }
    push_bsr(ul_bsr_indication_message{to_du_cell_index(0),
                                       ue_idx,
                                       crnti,
                                       bsr_format::LONG_BSR,
                                       ul_bsr_lcg_report_list{{uint_to_lcg_id(2), buffer_bytes}}});
  }

  /// Returns whether the last scheduling result carries a Differential Koffset MAC CE.
  bool is_koffset_ce_scheduled() const
  {
    const sched_result_copy* result = last_sched_result(to_du_cell_index(0));
    if (result == nullptr) {
      return false;
    }
    for (const dl_msg_alloc& ue_grant : result->dl.ue_grants) {
      for (const dl_msg_tb_info& tb_info : ue_grant.tb_list) {
        for (const dl_msg_lc_info& lc_info : tb_info.lc_chs_to_sched) {
          if (lc_info.lcid == lcid_dl_sch_t::from_elcid(elcid_dl_sch_t::DIFFERENTIAL_KOFFSET)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  /// Runs up to \c max_slots slots, with full buffers, and returns whether a Differential Koffset MAC CE was scheduled.
  bool run_until_koffset_ce_scheduled(unsigned max_slots)
  {
    for (unsigned count = 0; count != max_slots; ++count) {
      run_slot();
      refill_buffers();
      if (is_koffset_ce_scheduled()) {
        return true;
      }
    }
    return false;
  }

  /// \brief Runs \c nof_slots slots and returns the smallest PUSCH delay, k2 plus Koffset.
  ///
  /// Empty when the UE was granted no PUSCH at all, which a caller must tell apart from a delay: the UE goes
  /// unscheduled for stretches longer than a few slots, waiting out the uplink of its previous Koffset or for a HARQ
  /// to free up.
  std::optional<unsigned> run_and_measure_min_pusch_delay(unsigned nof_slots)
  {
    std::optional<unsigned> min_delay;
    for (unsigned count = 0; count != nof_slots; ++count) {
      run_slot();
      refill_buffers();
      const sched_result_copy* result = last_sched_result(to_du_cell_index(0));
      if (result == nullptr) {
        continue;
      }
      for (const ul_sched_info& ul_info : result->ul.puschs) {
        min_delay =
            min_delay.has_value() ? std::min<unsigned>(min_delay.value(), ul_info.context.k2) : ul_info.context.k2;
      }
    }
    return min_delay;
  }

  /// \brief Returns the Koffset, in slots, measured from an isolated PDSCH and its HARQ-ACK.
  ///
  /// Drains pending downlink feedback for \c nof_slots slots before sending one PDSCH. A HARQ-ACK lands k1 plus the
  /// Koffset slots after its PDSCH, as per TS 38.213, Section 9, on either PUCCH or PUSCH.
  /// Returns empty when the PDSCH or its HARQ-ACK is absent from the following \c nof_slots slots.
  std::optional<unsigned> run_and_measure_harq_ack_koffset(unsigned nof_slots)
  {
    // Isolate the PDSCH so that feedback from other transmissions cannot match a different Koffset.
    push_dl_buffer_state(dl_buffer_state_indication_message{ue_idx, ue_drb_lcid, 0});
    for (unsigned count = 0; count != nof_slots; ++count) {
      run_slot();
    }

    push_dl_buffer_state(dl_buffer_state_indication_message{ue_idx, ue_drb_lcid, 1});
    std::optional<unsigned> ack_base;
    for (unsigned count = 0; count != nof_slots; ++count) {
      run_slot();
      const sched_result_copy* result = last_sched_result(to_du_cell_index(0));
      if (result == nullptr) {
        continue;
      }
      if (const dl_msg_alloc* grant = find_ue_pdsch(crnti, *result)) {
        EXPECT_FALSE(ack_base.has_value()) << "More than one PDSCH was scheduled during the HARQ-ACK measurement";
        ack_base = count + grant->context.k1;
      }
      if (ack_base.has_value() and (find_ue_pucch_with_harq_ack(crnti, *result) != nullptr or
                                    find_ue_pusch_with_harq_ack(crnti, *result) != nullptr)) {
        return count - ack_base.value();
      }
    }
    return std::nullopt;
  }

  /// Sends a timing advance report of the UE carrying \c ul_ta.
  void report_timing_advance(std::chrono::microseconds ul_ta = reported_ul_ta)
  {
    ul_ta_report_indication_message ta_report{};
    ta_report.cell_index = to_du_cell_index(0);
    ta_report.ue_index   = ue_idx;
    ta_report.rnti       = crnti;
    ta_report.slot_rx    = last_result_slot();
    ta_report.ul_ta      = ul_ta;
    sched->handle_ul_ta_report_indication(ta_report);
  }

  /// Runs the slots a Differential Koffset MAC CE takes to reach the UE and take effect.
  void run_until_koffset_applied()
  {
    ASSERT_TRUE(run_until_koffset_ce_scheduled(max_slots_to_koffset_ce));
    for (unsigned count = 0; count != nof_slots_at_ue_koffset; ++count) {
      run_slot();
      refill_buffers();
    }
  }

  /// Whether the downlink buffer of the UE is refilled. Downlink data occupies every HARQ process, which a test
  /// dropping the feedback that frees them cannot afford.
  bool fill_dl_buffer = true;

  const du_ue_index_t ue_idx      = to_du_ue_index(0);
  const lcid_t        ue_drb_lcid = LCID_MIN_DRB;
  rnti_t              crnti       = to_rnti(0x4601);

  cell_config_builder_params params;
};

} // namespace

TEST_F(scheduler_ue_koffset_tester, no_differential_koffset_is_signalled_without_a_timing_advance_report)
{
  for (unsigned count = 0; count != nof_slots_without_ta_report; ++count) {
    run_slot();
    refill_buffers();
    ASSERT_FALSE(is_koffset_ce_scheduled()) << "Nothing is known of the UE timing advance yet";
  }
}

TEST_F(scheduler_ue_koffset_tester, differential_koffset_is_signalled_after_a_timing_advance_report)
{
  // Slots within which the MAC CE has to be scheduled, counted from the first slot of the UE.
  static constexpr unsigned max_slots_to_first_koffset_ce = 200;

  run_slot();
  report_timing_advance();

  ASSERT_TRUE(run_until_koffset_ce_scheduled(max_slots_to_first_koffset_ce))
      << "No Differential Koffset MAC CE was scheduled for the UE";
}

TEST_F(scheduler_ue_koffset_tester, uplink_moves_earlier_once_the_ue_applies_its_own_koffset)
{
  // Let the uplink settle on the cell-specific Koffset. The first PUSCH lands k2 plus the Koffset slots in, so the
  // window only has to outlast that.
  const std::optional<unsigned> cell_koffset_delay = run_and_measure_min_pusch_delay(nof_slots_at_cell_koffset);
  ASSERT_TRUE(cell_koffset_delay.has_value()) << "No PUSCH was scheduled at the cell-specific Koffset";

  report_timing_advance();

  // The UE applies the new Koffset only after its HARQ-ACK and apply slot, as per TS 38.213, Section 9.
  ASSERT_TRUE(run_until_koffset_ce_scheduled(max_slots_to_koffset_ce));

  // Not applied yet, as the MAC CE is not even acknowledged.
  const std::optional<unsigned> during_transition = run_and_measure_min_pusch_delay(nof_slots_during_transition);
  ASSERT_TRUE(during_transition.has_value()) << "No PUSCH was scheduled while the MAC CE was in flight";
  ASSERT_EQ(during_transition.value(), cell_koffset_delay.value())
      << "Uplink moved before the UE applies the new Koffset";

  // Once applied, the uplink moves earlier by the Differential Koffset.
  const std::optional<unsigned> ue_koffset_delay = run_and_measure_min_pusch_delay(nof_slots_at_ue_koffset);
  ASSERT_TRUE(ue_koffset_delay.has_value()) << "No PUSCH was scheduled after the UE applied its own Koffset";
  ASSERT_EQ(ue_koffset_delay.value(), cell_koffset_delay.value() - expected_differential_slots);
}

TEST_F(scheduler_ue_koffset_tester, harq_ack_moves_earlier_once_the_ue_applies_its_own_koffset)
{
  const unsigned cell_koffset_slots = static_cast<unsigned>(cell_koffset.count());

  // Let the downlink settle on the cell-specific Koffset.
  const std::optional<unsigned> before = run_and_measure_harq_ack_koffset(nof_slots_at_cell_koffset);
  ASSERT_TRUE(before.has_value()) << "No HARQ-ACK was scheduled for the UE";
  ASSERT_EQ(before.value(), cell_koffset_slots) << "HARQ-ACK was not placed at the cell-specific Koffset";

  report_timing_advance();

  // The UE applies the new Koffset only after its HARQ-ACK and apply slot, as per TS 38.213, Section 9.
  ASSERT_TRUE(run_until_koffset_ce_scheduled(max_slots_to_koffset_ce));

  // Once applied, the HARQ-ACK moves earlier by the Differential Koffset, the same way the uplink does.
  const std::optional<unsigned> after = run_and_measure_harq_ack_koffset(nof_slots_at_ue_koffset);
  ASSERT_TRUE(after.has_value()) << "No HARQ-ACK was scheduled after the UE applied its own Koffset";
  ASSERT_EQ(after.value(), cell_koffset_slots - expected_differential_slots);
}

TEST_F(scheduler_ue_koffset_tester, a_repeated_timing_advance_report_signals_nothing)
{
  run_slot();
  report_timing_advance();
  ASSERT_NO_FATAL_FAILURE(run_until_koffset_applied());

  // The derived value has not moved, so the UE has nothing to be told.
  report_timing_advance();
  ASSERT_FALSE(run_until_koffset_ce_scheduled(max_slots_to_koffset_ce)) << "An unchanged Koffset was signalled again";
}

TEST_F(scheduler_ue_koffset_tester, a_moved_timing_advance_is_not_signalled_while_a_value_is_in_flight)
{
  run_slot();
  report_timing_advance();
  ASSERT_TRUE(run_until_koffset_ce_scheduled(max_slots_to_koffset_ce));

  // The MAC CE has not reached the UE yet, so the scheduler cannot tell which value the UE is on.
  report_timing_advance(grown_ul_ta);
  ASSERT_FALSE(run_until_koffset_ce_scheduled(nof_slots_during_transition))
      << "A second Differential Koffset was signalled while the first was in flight";
}

TEST_F(scheduler_ue_koffset_tester, a_koffset_whose_acknowledgement_is_not_detected_is_not_applied)
{
  // Every HARQ-ACK occasion reports DTX. Nothing was detected there, which leaves open whether the UE decoded the
  // MAC CE, so the scheduler stays on the value it last confirmed and the HARQ retransmits.
  register_uci_handler([](uci_indication& uci) {
    for (auto& pdu : uci.ucis) {
      if (auto* f1 = std::get_if<uci_indication::uci_pdu::uci_pucch_f0_or_f1_pdu>(&pdu.pdu)) {
        std::fill(f1->harqs.begin(), f1->harqs.end(), mac_harq_ack_report_status::dtx);
      } else if (auto* f2 = std::get_if<uci_indication::uci_pdu::uci_pucch_f2_or_f3_or_f4_pdu>(&pdu.pdu)) {
        std::fill(f2->harqs.begin(), f2->harqs.end(), mac_harq_ack_report_status::dtx);
      }
    }
  });

  run_slot();
  const std::optional<unsigned> before = run_and_measure_min_pusch_delay(nof_slots_at_cell_koffset);
  ASSERT_TRUE(before.has_value()) << "No PUSCH was scheduled for the UE";

  report_timing_advance();
  ASSERT_TRUE(run_until_koffset_ce_scheduled(max_slots_to_koffset_ce));

  const std::optional<unsigned> after = run_and_measure_min_pusch_delay(nof_slots_at_ue_koffset);
  ASSERT_TRUE(after.has_value()) << "No PUSCH was scheduled after the MAC CE went out";
  ASSERT_EQ(after.value(), before.value()) << "A Koffset with no detected acknowledgement was applied";
}

TEST_F(scheduler_ue_koffset_tester, a_koffset_whose_acknowledgement_never_reaches_the_scheduler_is_not_applied)
{
  // Drop every HARQ-ACK on the way, so no report on the MAC CE is ever handled. The UE may have decoded it and
  // applied the value, but the scheduler cannot schedule on one it has no confirmation of.
  register_uci_handler([](uci_indication& uci) { uci.ucis.clear(); });

  // Downlink data occupies every HARQ process, and none is freed while its feedback is dropped, so no PDSCH would be
  // left to carry the MAC CE. The MAC CE is downlink data of its own.
  fill_dl_buffer = false;
  push_dl_buffer_state(dl_buffer_state_indication_message{ue_idx, ue_drb_lcid, 0});

  run_slot();
  const std::optional<unsigned> before = run_and_measure_min_pusch_delay(nof_slots_at_cell_koffset);
  ASSERT_TRUE(before.has_value()) << "No PUSCH was scheduled for the UE";

  report_timing_advance();
  ASSERT_TRUE(run_until_koffset_ce_scheduled(max_slots_to_koffset_ce));

  const std::optional<unsigned> after = run_and_measure_min_pusch_delay(nof_slots_at_ue_koffset);
  ASSERT_TRUE(after.has_value()) << "No PUSCH was scheduled after the MAC CE went out";
  ASSERT_EQ(after.value(), before.value()) << "A Koffset with no acknowledgement was applied";

  // A window the UE was scheduled somewhere in says nothing on its own, as the grants of the slots before the MAC CE
  // went out already fill it. The UE has to be scheduled in a window that starts well past the apply slot too. It is
  // held back until the HARQ carrying the MAC CE times out and the value is given up, and is then scheduled again at
  // the Koffset it was on all along.
  const std::optional<unsigned> tail = run_and_measure_min_pusch_delay(nof_slots_to_unconfirmed_give_up);
  ASSERT_TRUE(tail.has_value()) << "The UE was never scheduled again after the Koffset in flight was given up";
  ASSERT_EQ(tail.value(), before.value());
}

TEST_F(scheduler_ue_koffset_tester, a_moved_timing_advance_signals_a_new_koffset)
{
  run_slot();
  report_timing_advance();
  ASSERT_NO_FATAL_FAILURE(run_until_koffset_applied());

  // A T_TA that grew takes more of the cell-specific Koffset, so the UE cannot wait for its uplink to be moved back.
  report_timing_advance(grown_ul_ta);
  ASSERT_NO_FATAL_FAILURE(run_until_koffset_applied());

  // The other way, where the UE only stands to gain a shorter uplink delay.
  report_timing_advance(shrunk_ul_ta);
  ASSERT_TRUE(run_until_koffset_ce_scheduled(max_slots_to_koffset_ce));
}
