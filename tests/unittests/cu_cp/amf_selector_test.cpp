// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "lib/cu_cp/cu_cp_controller/amf_selector.h"
#include "tests/test_doubles/utils/test_rng.h"
#include <algorithm>
#include <gtest/gtest.h>

using namespace ocudu;
using namespace ocucp;

namespace {

const plmn_identity plmn_a = plmn_identity::test_value();
const plmn_identity plmn_b = plmn_identity::parse("99902").value();

guami_t make_guami(uint16_t amf_set_id, uint8_t amf_pointer, plmn_identity plmn = plmn_a)
{
  return guami_t{.plmn = plmn, .amf_set_id = amf_set_id, .amf_pointer = amf_pointer, .amf_region_id = 2};
}

amf_candidate make_candidate(unsigned amf_idx, const guami_t& guami, uint16_t capacity = 255, bool connected = true)
{
  amf_candidate candidate;
  candidate.amf_index             = uint_to_cu_cp_amf_index(amf_idx);
  candidate.amf_name              = fmt::format("amf{}", amf_idx);
  candidate.connected             = connected;
  candidate.plmns                 = {guami.plmn};
  candidate.served_guamis         = {ngap_served_guami_item{guami, std::nullopt, std::nullopt}};
  candidate.relative_amf_capacity = capacity;
  return candidate;
}

s_nssai_t make_s_nssai(uint8_t sst, std::optional<uint32_t> sd = std::nullopt)
{
  s_nssai_t s_nssai;
  s_nssai.sst = slice_service_type{sst};
  if (sd.has_value()) {
    s_nssai.sd = slice_differentiator::create(*sd).value();
  }
  return s_nssai;
}

/// Sets the slices that the AMF supports in PLMN A.
void set_supported_slices(amf_candidate& candidate, const std::vector<s_nssai_t>& slices)
{
  ngap_plmn_support_item item;
  item.plmn_id = plmn_a;
  for (const s_nssai_t& s_nssai : slices) {
    item.slice_support_list.push_back(slice_support_item_t{s_nssai});
  }
  candidate.plmn_support_list = {item};
}

/// The AMF selection draws random numbers. A test of a rule that selects one AMF gives this AMF capacity 0 and the
/// other AMFs a higher capacity. Only the rule can then select this AMF. A failed test prints the seed of the test run.
class amf_selector_test : public ::testing::Test
{
protected:
  std::optional<cu_cp_amf_index_t> select(const amf_selection_request& request)
  {
    return selector.select_amf(candidates, request);
  }

  /// Counts how often each AMF is selected for a UE that identifies no AMF.
  std::vector<unsigned> count_selections(unsigned nof_selections)
  {
    std::vector<unsigned> counts(candidates.size(), 0);
    for (unsigned i = 0; i != nof_selections; ++i) {
      std::optional<cu_cp_amf_index_t> amf_index = select(amf_selection_request{.plmn = plmn_a});
      EXPECT_TRUE(amf_index.has_value());
      ++counts[cu_cp_amf_index_to_uint(amf_index.value())];
    }
    return counts;
  }

  std::vector<amf_candidate> candidates;
  amf_selector               selector{test_rng::uniform_int<uint32_t>()};
};

} // namespace

TEST_F(amf_selector_test, when_ue_has_registered_guami_then_amf_of_this_guami_is_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 1), 0)};

  ASSERT_EQ(select({.plmn = plmn_a, .registered_guami = make_guami(1, 1)}), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_ue_has_mapped_guami_then_only_a_mapped_served_guami_matches)
{
  candidates = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 0), 0)};
  candidates[1].served_guamis[0].guami_type = guami_type_t::mapped;

  ASSERT_EQ(
      select({.plmn = plmn_a, .registered_guami = make_guami(1, 0), .registered_guami_type = guami_type_t::mapped}),
      uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_ue_has_5g_s_tmsi_then_amf_of_its_set_and_pointer_is_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 1), 0)};

  ASSERT_EQ(select({.plmn = plmn_a, .five_g_s_tmsi = five_g_s_tmsi_t{1, 1, 0x1234}}), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_amf_of_the_ue_is_not_connected_then_an_amf_of_the_same_set_is_selected)
{
  candidates = {make_candidate(0, make_guami(2, 0), 255),
                make_candidate(1, make_guami(1, 0), 255, false),
                make_candidate(2, make_guami(1, 1), 0)};

  ASSERT_EQ(select({.plmn = plmn_a, .registered_guami = make_guami(1, 0)}), uint_to_cu_cp_amf_index(2));
  ASSERT_EQ(select({.plmn = plmn_a, .five_g_s_tmsi = five_g_s_tmsi_t{1, 0, 0x1234}}), uint_to_cu_cp_amf_index(2));
}

TEST_F(amf_selector_test, when_several_amfs_serve_the_registered_guami_then_selection_is_weighted_by_capacity)
{
  candidates = {make_candidate(0, make_guami(1, 0), 0), make_candidate(1, make_guami(1, 0), 255)};

  ASSERT_EQ(select({.plmn = plmn_a, .registered_guami = make_guami(1, 0)}), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_amf_of_the_registered_guami_does_not_support_the_plmn_then_another_amf_is_selected)
{
  candidates          = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1), 1)};
  candidates[0].plmns = {plmn_b};

  ASSERT_EQ(select({.plmn = plmn_a, .registered_guami = make_guami(1, 0)}), uint_to_cu_cp_amf_index(1));
  ASSERT_EQ(select({.plmn = plmn_a, .five_g_s_tmsi = five_g_s_tmsi_t{1, 0, 0x1234}}), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_ue_identifies_an_amf_set_then_only_amfs_of_the_same_guami_type_are_selected)
{
  // AMF 0 serves the AMF Set with native GUAMIs, AMF 1 with mapped GUAMIs.
  candidates = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 1), 0)};
  candidates[1].served_guamis[0].guami_type = guami_type_t::mapped;

  ASSERT_EQ(
      select({.plmn = plmn_a, .registered_guami = make_guami(1, 5), .registered_guami_type = guami_type_t::mapped}),
      uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_guami_of_the_ue_is_unavailable_then_backup_amf_of_status_indication_is_selected)
{
  candidates = {
      make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1)), make_candidate(2, make_guami(1, 2), 0)};
  candidates[0].served_guamis[0].backup_amf_name = "amf1";
  candidates[0].unavailable_guamis               = {ngap_unavailable_guami_item{make_guami(1, 0), "amf2"}};

  ASSERT_EQ(select({.plmn = plmn_a, .registered_guami = make_guami(1, 0)}), uint_to_cu_cp_amf_index(2));
  ASSERT_EQ(select({.plmn = plmn_a, .five_g_s_tmsi = five_g_s_tmsi_t{1, 0, 0x1234}}), uint_to_cu_cp_amf_index(2));
  ASSERT_EQ(selector.select_amf_for_guami(candidates, make_guami(1, 0)), uint_to_cu_cp_amf_index(2));
}

TEST_F(amf_selector_test, when_amf_of_the_ue_is_not_connected_then_backup_amf_of_served_guami_is_selected)
{
  candidates                                     = {make_candidate(0, make_guami(1, 0), 255, false),
                                                    make_candidate(1, make_guami(1, 1)),
                                                    make_candidate(2, make_guami(1, 2), 0)};
  candidates[0].served_guamis[0].backup_amf_name = "amf2";

  ASSERT_EQ(select({.plmn = plmn_a, .five_g_s_tmsi = five_g_s_tmsi_t{1, 0, 0x1234}}), uint_to_cu_cp_amf_index(2));
}

TEST_F(amf_selector_test, when_guami_of_the_ue_is_unavailable_without_backup_then_another_amf_of_the_set_is_selected)
{
  candidates                       = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1), 0)};
  candidates[0].unavailable_guamis = {ngap_unavailable_guami_item{make_guami(1, 0), std::nullopt}};

  ASSERT_EQ(select({.plmn = plmn_a, .registered_guami = make_guami(1, 0)}), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_guami_is_unavailable_without_backup_then_another_amf_of_its_set_is_found)
{
  // AMF 0 comes first and serves the AMF Set too, but marks the GUAMI as unavailable.
  candidates                       = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1), 0)};
  candidates[0].unavailable_guamis = {ngap_unavailable_guami_item{make_guami(1, 0), std::nullopt}};

  ASSERT_EQ(selector.select_amf_for_guami(candidates, make_guami(1, 0)), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_all_guamis_of_an_amf_are_unavailable_then_it_gets_no_new_ues)
{
  candidates                       = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 1), 0)};
  candidates[0].unavailable_guamis = {ngap_unavailable_guami_item{make_guami(1, 0), std::nullopt}};

  ASSERT_EQ(select({.plmn = plmn_a}), uint_to_cu_cp_amf_index(1));
  ASSERT_EQ(selector.select_amf_for_amf_set(candidates, plmn_a, 1), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_the_guamis_of_all_amfs_are_unavailable_then_an_amf_is_still_selected)
{
  candidates                       = {make_candidate(0, make_guami(1, 0))};
  candidates[0].unavailable_guamis = {ngap_unavailable_guami_item{make_guami(1, 0), std::nullopt}};

  ASSERT_EQ(select({.plmn = plmn_a}), uint_to_cu_cp_amf_index(0));
  ASSERT_EQ(selector.select_amf_for_amf_set(candidates, plmn_a, 1), uint_to_cu_cp_amf_index(0));
}

TEST_F(amf_selector_test, when_overload_action_rejects_the_establishment_cause_then_another_amf_is_selected)
{
  candidates             = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1))};
  candidates[0].overload = ngap_amf_overload{ngap_overload_action_t::reject_signalling, std::nullopt};

  ASSERT_EQ(
      select(
          {.plmn = plmn_a, .registered_guami = make_guami(1, 0), .establishment_cause = establishment_cause_t::mo_sig}),
      uint_to_cu_cp_amf_index(1));
  ASSERT_EQ(select({.plmn                = plmn_a,
                    .registered_guami    = make_guami(1, 0),
                    .establishment_cause = establishment_cause_t::mt_access}),
            uint_to_cu_cp_amf_index(0));
}

TEST_F(amf_selector_test, when_amf_is_overloaded_then_its_overload_action_decides_the_rejected_establishment_causes)
{
  using cause  = establishment_cause_t;
  using action = ngap_overload_action_t;
  struct test_case {
    action             overload_action;
    std::vector<cause> rejected;
  };
  // Mapping of TS 38.413 section 8.7.7.2.
  const std::vector<test_case> test_cases = {
      {action::reject_non_emergency_mo_data,
       {cause::mo_data, cause::mo_sms, cause::mo_video_call, cause::mo_voice_call}},
      {action::reject_signalling,
       {cause::mo_data, cause::mo_sms, cause::mo_video_call, cause::mo_voice_call, cause::mo_sig}},
      {action::only_emergency_and_mt,
       {cause::high_prio_access,
        cause::mo_sig,
        cause::mo_data,
        cause::mo_voice_call,
        cause::mo_video_call,
        cause::mo_sms,
        cause::mps_prio_access,
        cause::mcs_prio_access}},
      {action::only_high_priority_and_mt,
       {cause::emergency, cause::mo_sig, cause::mo_data, cause::mo_voice_call, cause::mo_video_call, cause::mo_sms}},
  };
  const std::vector<cause> all_causes = {cause::emergency,
                                         cause::high_prio_access,
                                         cause::mt_access,
                                         cause::mo_sig,
                                         cause::mo_data,
                                         cause::mo_voice_call,
                                         cause::mo_video_call,
                                         cause::mo_sms,
                                         cause::mps_prio_access,
                                         cause::mcs_prio_access};

  candidates = {make_candidate(0, make_guami(1, 0))};
  for (const test_case& test : test_cases) {
    candidates[0].overload = ngap_amf_overload{test.overload_action, std::nullopt};
    for (cause establishment_cause : all_causes) {
      bool is_rejected =
          std::find(test.rejected.begin(), test.rejected.end(), establishment_cause) != test.rejected.end();
      ASSERT_EQ(select({.plmn = plmn_a, .establishment_cause = establishment_cause}).has_value(), not is_rejected)
          << "action=" << static_cast<unsigned>(test.overload_action)
          << " cause=" << static_cast<unsigned>(establishment_cause);
    }
  }
}

TEST_F(amf_selector_test, when_amf_has_traffic_load_reduction_then_this_share_of_the_rejected_ues_is_dropped)
{
  candidates             = {make_candidate(0, make_guami(1, 0))};
  candidates[0].overload = ngap_amf_overload{ngap_overload_action_t::reject_signalling, 25};

  unsigned nof_rejected = 0;
  for (unsigned i = 0; i != 4000; ++i) {
    if (not select({.plmn = plmn_a, .establishment_cause = establishment_cause_t::mo_sig}).has_value()) {
      ++nof_rejected;
    }
    ASSERT_TRUE(select({.plmn = plmn_a, .establishment_cause = establishment_cause_t::mt_access}).has_value());
  }

  EXPECT_NEAR(nof_rejected, 1000, 150);
}

TEST_F(amf_selector_test, when_amf_is_overloaded_then_another_amf_gets_the_new_ues)
{
  // Without the overload, the selection picks AMF 0.
  candidates             = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 1), 0)};
  candidates[0].overload = ngap_amf_overload{};

  ASSERT_EQ(select({.plmn = plmn_a}), uint_to_cu_cp_amf_index(1));
  ASSERT_EQ(selector.select_amf_for_amf_set(candidates, plmn_a, 1), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_all_amfs_are_overloaded_without_action_then_an_amf_is_still_selected)
{
  candidates             = {make_candidate(0, make_guami(1, 0))};
  candidates[0].overload = ngap_amf_overload{};

  ASSERT_EQ(select({.plmn = plmn_a, .establishment_cause = establishment_cause_t::mo_data}),
            uint_to_cu_cp_amf_index(0));
}

TEST_F(amf_selector_test, when_every_connected_amf_rejects_the_establishment_cause_then_ue_setup_is_not_accepted)
{
  candidates             = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1), 255, false)};
  candidates[0].overload = ngap_amf_overload{ngap_overload_action_t::reject_signalling, std::nullopt};

  ASSERT_FALSE(selector.admit_ue_setup(candidates, establishment_cause_t::mo_sig).has_value());

  std::optional<std::vector<cu_cp_amf_index_t>> rejecting_amfs =
      selector.admit_ue_setup(candidates, establishment_cause_t::mt_access);
  ASSERT_TRUE(rejecting_amfs.has_value());
  ASSERT_TRUE(rejecting_amfs->empty());
}

TEST_F(amf_selector_test, when_amf_has_traffic_load_reduction_then_this_share_of_the_ue_setups_is_rejected)
{
  candidates             = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1))};
  candidates[0].overload = ngap_amf_overload{ngap_overload_action_t::reject_signalling, 25};
  candidates[1].overload = ngap_amf_overload{ngap_overload_action_t::reject_signalling, std::nullopt};

  unsigned nof_rejected = 0;
  for (unsigned i = 0; i != 4000; ++i) {
    std::optional<std::vector<cu_cp_amf_index_t>> rejecting_amfs =
        selector.admit_ue_setup(candidates, establishment_cause_t::mo_sig);
    if (not rejecting_amfs.has_value()) {
      ++nof_rejected;
    } else {
      // AMF 1 rejects every UE, AMF 0 accepts this UE.
      ASSERT_EQ(*rejecting_amfs, std::vector<cu_cp_amf_index_t>{uint_to_cu_cp_amf_index(1)});
    }
  }

  EXPECT_NEAR(nof_rejected, 1000, 150);
}

TEST_F(amf_selector_test, when_ue_setup_request_decided_the_overload_then_the_selection_uses_this_decision)
{
  // AMF 0 rejects every UE with the establishment cause mo-Signalling.
  candidates             = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1))};
  candidates[0].overload = ngap_amf_overload{ngap_overload_action_t::reject_signalling, std::nullopt};

  // The RRC Setup Request found that AMF 0 accepts the UE, so the selection does not check the overload again.
  ASSERT_EQ(select({.plmn                    = plmn_a,
                    .registered_guami        = make_guami(1, 0),
                    .establishment_cause     = establishment_cause_t::mo_sig,
                    .overload_rejecting_amfs = std::vector<cu_cp_amf_index_t>{}}),
            uint_to_cu_cp_amf_index(0));
  // The RRC Setup Request found that AMF 1 rejects the UE.
  ASSERT_EQ(select({.plmn                    = plmn_a,
                    .registered_guami        = make_guami(1, 1),
                    .establishment_cause     = establishment_cause_t::mt_access,
                    .overload_rejecting_amfs = std::vector<cu_cp_amf_index_t>{uint_to_cu_cp_amf_index(1)}}),
            uint_to_cu_cp_amf_index(0));
}

TEST_F(amf_selector_test, when_amf_rejects_the_resume_cause_then_the_resume_is_not_accepted)
{
  amf_candidate candidate = make_candidate(0, make_guami(1, 0));
  candidate.overload      = ngap_amf_overload{ngap_overload_action_t::reject_signalling, std::nullopt};

  ASSERT_FALSE(selector.is_ue_resume_accepted(candidate, resume_cause_t::mo_sig, {}));
  ASSERT_FALSE(selector.is_ue_resume_accepted(candidate, resume_cause_t::mo_data, {}));
  ASSERT_TRUE(selector.is_ue_resume_accepted(candidate, resume_cause_t::mt_access, {}));
  // An RNA update sends no signalling to the AMF.
  ASSERT_TRUE(selector.is_ue_resume_accepted(candidate, resume_cause_t::rna_upd, {}));

  candidate.overload.reset();
  ASSERT_TRUE(selector.is_ue_resume_accepted(candidate, resume_cause_t::mo_sig, {}));
}

TEST_F(amf_selector_test, when_ue_requests_a_slice_then_an_amf_that_supports_it_is_selected)
{
  // Without the requested slice, the selection picks AMF 0.
  candidates = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 1), 0)};
  set_supported_slices(candidates[0], {make_s_nssai(1)});
  set_supported_slices(candidates[1], {make_s_nssai(1), make_s_nssai(2, 0x10)});

  ASSERT_EQ(select({.plmn = plmn_a, .requested_nssai = {make_s_nssai(2, 0x10)}}), uint_to_cu_cp_amf_index(1));
  // An AMF that supports all the requested slices comes before an AMF that supports some of them.
  ASSERT_EQ(select({.plmn = plmn_a, .requested_nssai = {make_s_nssai(1), make_s_nssai(2, 0x10)}}),
            uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_the_amf_of_the_requested_slice_is_unavailable_then_another_amf_is_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 1), 0)};
  set_supported_slices(candidates[0], {make_s_nssai(2)});
  set_supported_slices(candidates[1], {make_s_nssai(1)});
  candidates[0].unavailable_guamis = {ngap_unavailable_guami_item{make_guami(1, 0), std::nullopt}};

  ASSERT_EQ(select({.plmn = plmn_a, .requested_nssai = {make_s_nssai(2)}}), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_no_amf_supports_all_requested_slices_then_an_amf_that_supports_some_is_selected)
{
  // Without the requested slices, the selection picks AMF 1.
  candidates = {make_candidate(0, make_guami(1, 0), 0), make_candidate(1, make_guami(1, 1), 255)};
  set_supported_slices(candidates[0], {make_s_nssai(1)});
  set_supported_slices(candidates[1], {make_s_nssai(3)});

  ASSERT_EQ(select({.plmn = plmn_a, .requested_nssai = {make_s_nssai(1), make_s_nssai(2)}}),
            uint_to_cu_cp_amf_index(0));
}

TEST_F(amf_selector_test, when_no_amf_supports_the_requested_slice_then_any_amf_is_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1))};
  set_supported_slices(candidates[0], {make_s_nssai(1)});
  set_supported_slices(candidates[1], {make_s_nssai(1)});

  ASSERT_TRUE(select({.plmn = plmn_a, .requested_nssai = {make_s_nssai(2)}}).has_value());
}

TEST_F(amf_selector_test, when_ue_has_5g_s_tmsi_then_the_requested_slices_do_not_select_the_amf)
{
  candidates = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1))};
  set_supported_slices(candidates[0], {make_s_nssai(1)});
  set_supported_slices(candidates[1], {make_s_nssai(2)});

  ASSERT_EQ(
      select({.plmn = plmn_a, .five_g_s_tmsi = five_g_s_tmsi_t{1, 0, 0x1234}, .requested_nssai = {make_s_nssai(2)}}),
      uint_to_cu_cp_amf_index(0));
}

TEST_F(amf_selector_test, when_all_requested_slices_are_overloaded_then_another_amf_is_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1))};
  ngap_amf_overload overload;
  overload.overload_start_nssai_list = {ngap_overload_start_nssai_item{{make_s_nssai(2)}, std::nullopt, std::nullopt}};
  candidates[0].overload             = overload;

  // The slice overload affects a UE whose requested S-NSSAIs are all overloaded.
  ASSERT_EQ(select({.plmn = plmn_a, .registered_guami = make_guami(1, 0), .requested_nssai = {make_s_nssai(2)}}),
            uint_to_cu_cp_amf_index(1));
  ASSERT_EQ(select({.plmn             = plmn_a,
                    .registered_guami = make_guami(1, 0),
                    .requested_nssai  = {make_s_nssai(1), make_s_nssai(2)}}),
            uint_to_cu_cp_amf_index(0));
  ASSERT_EQ(select({.plmn = plmn_a, .registered_guami = make_guami(1, 0)}), uint_to_cu_cp_amf_index(0));
}

TEST_F(amf_selector_test, when_slice_overload_has_an_action_then_only_the_rejected_establishment_causes_are_affected)
{
  candidates = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1))};
  ngap_amf_overload overload;
  overload.overload_start_nssai_list = {
      ngap_overload_start_nssai_item{{make_s_nssai(2)}, ngap_overload_action_t::reject_signalling, std::nullopt}};
  candidates[0].overload = overload;

  ASSERT_EQ(select({.plmn                = plmn_a,
                    .registered_guami    = make_guami(1, 0),
                    .requested_nssai     = {make_s_nssai(2)},
                    .establishment_cause = establishment_cause_t::mo_sig}),
            uint_to_cu_cp_amf_index(1));
  ASSERT_EQ(select({.plmn                = plmn_a,
                    .registered_guami    = make_guami(1, 0),
                    .requested_nssai     = {make_s_nssai(2)},
                    .establishment_cause = establishment_cause_t::mt_access}),
            uint_to_cu_cp_amf_index(0));
}

TEST_F(amf_selector_test, when_slice_overload_has_traffic_load_reduction_then_this_share_of_the_ues_is_dropped)
{
  candidates = {make_candidate(0, make_guami(1, 0))};
  ngap_amf_overload overload;
  overload.overload_start_nssai_list = {ngap_overload_start_nssai_item{{make_s_nssai(2)}, std::nullopt, 25}};
  candidates[0].overload             = overload;

  unsigned nof_rejected = 0;
  for (unsigned i = 0; i != 4000; ++i) {
    if (not select({.plmn = plmn_a, .requested_nssai = {make_s_nssai(2)}}).has_value()) {
      ++nof_rejected;
    }
  }

  EXPECT_NEAR(nof_rejected, 1000, 150);
}

TEST_F(amf_selector_test, when_all_allowed_slices_of_an_inactive_ue_are_overloaded_then_the_resume_is_not_accepted)
{
  amf_candidate     candidate = make_candidate(0, make_guami(1, 0));
  ngap_amf_overload overload;
  overload.overload_start_nssai_list = {ngap_overload_start_nssai_item{{make_s_nssai(2)}, std::nullopt, std::nullopt}};
  candidate.overload                 = overload;

  const std::vector<s_nssai_t> overloaded_slices = {make_s_nssai(2)};
  const std::vector<s_nssai_t> mixed_slices      = {make_s_nssai(1), make_s_nssai(2)};
  ASSERT_FALSE(selector.is_ue_resume_accepted(candidate, resume_cause_t::mo_data, overloaded_slices));
  ASSERT_TRUE(selector.is_ue_resume_accepted(candidate, resume_cause_t::mo_data, mixed_slices));
  ASSERT_TRUE(selector.is_ue_resume_accepted(candidate, resume_cause_t::mo_data, {}));
}

TEST_F(amf_selector_test, when_ue_identifies_no_amf_then_selection_is_weighted_by_relative_amf_capacity)
{
  candidates = {make_candidate(0, make_guami(1, 0), 255), make_candidate(1, make_guami(1, 1), 85)};

  std::vector<unsigned> counts = count_selections(4000);

  // The expected ratio is 255:85, i.e. 3000 and 1000 selections.
  EXPECT_NEAR(counts[0], 3000, 150);
  EXPECT_NEAR(counts[1], 1000, 150);
}

TEST_F(amf_selector_test, when_amf_has_capacity_zero_then_it_is_not_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0), 0), make_candidate(1, make_guami(1, 1), 1)};

  std::vector<unsigned> counts = count_selections(100);

  ASSERT_EQ(counts[0], 0);
  ASSERT_EQ(counts[1], 100);
}

TEST_F(amf_selector_test, when_all_amfs_have_capacity_zero_then_all_amfs_are_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0), 0), make_candidate(1, make_guami(1, 1), 0)};

  std::vector<unsigned> counts = count_selections(100);

  ASSERT_GT(counts[0], 0);
  ASSERT_GT(counts[1], 0);
}

TEST_F(amf_selector_test, when_no_connected_amf_supports_the_plmn_then_no_amf_is_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0, plmn_b)), make_candidate(1, make_guami(1, 1), 255, false)};

  ASSERT_FALSE(select({.plmn = plmn_a}).has_value());
}

TEST_F(amf_selector_test, when_guami_is_served_then_its_amf_is_found)
{
  candidates = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(1, 1))};

  ASSERT_EQ(selector.select_amf_for_guami(candidates, make_guami(1, 1)), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_amf_of_guami_is_not_connected_then_an_amf_of_the_same_set_is_found)
{
  candidates = {make_candidate(0, make_guami(1, 0), 255, false), make_candidate(1, make_guami(1, 1))};

  ASSERT_EQ(selector.select_amf_for_guami(candidates, make_guami(1, 0)), uint_to_cu_cp_amf_index(1));
}

TEST_F(amf_selector_test, when_no_amf_of_the_guami_set_is_connected_then_no_amf_is_found)
{
  candidates = {make_candidate(0, make_guami(1, 0), 255, false), make_candidate(1, make_guami(2, 0))};

  ASSERT_FALSE(selector.select_amf_for_guami(candidates, make_guami(1, 0)).has_value());
}

TEST_F(amf_selector_test, when_amf_set_is_requested_then_a_connected_amf_of_this_set_is_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0), 255),
                make_candidate(1, make_guami(2, 0), 255, false),
                make_candidate(2, make_guami(2, 1), 0)};

  ASSERT_EQ(selector.select_amf_for_amf_set(candidates, plmn_a, 2), uint_to_cu_cp_amf_index(2));
}

TEST_F(amf_selector_test, when_no_amf_of_the_requested_set_is_connected_then_no_amf_is_selected)
{
  candidates = {make_candidate(0, make_guami(1, 0)), make_candidate(1, make_guami(2, 0), 255, false)};

  ASSERT_FALSE(selector.select_amf_for_amf_set(candidates, plmn_a, 2).has_value());
}
