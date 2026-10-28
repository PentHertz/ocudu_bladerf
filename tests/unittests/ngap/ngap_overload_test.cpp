// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ngap_test_helpers.h"
#include <gtest/gtest.h>

using namespace ocudu;
using namespace ocucp;

class ngap_overload_test : public ngap_test
{
protected:
  ngap_overload_test() { report_fatal_error_if_not(run_ng_setup(), "NG Setup failed"); }
};

TEST_F(ngap_overload_test, when_overload_start_is_received_then_overload_state_is_stored)
{
  ngap->handle_message(generate_overload_start(asn1::ngap::overload_action_opts::reject_rrc_cr_sig, 30));

  const std::optional<ngap_amf_overload>& overload = ngap->get_ngap_context().overload;
  ASSERT_TRUE(overload.has_value());
  ASSERT_EQ(overload->action, ngap_overload_action_t::reject_signalling);
  ASSERT_EQ(overload->traffic_load_reduction, 30);
}

TEST_F(ngap_overload_test, when_overload_start_has_no_ie_then_amf_is_overloaded_without_action)
{
  ngap->handle_message(generate_overload_start());

  const std::optional<ngap_amf_overload>& overload = ngap->get_ngap_context().overload;
  ASSERT_TRUE(overload.has_value());
  ASSERT_FALSE(overload->action.has_value());
  ASSERT_FALSE(overload->traffic_load_reduction.has_value());
}

TEST_F(ngap_overload_test, when_overload_start_is_repeated_then_overload_state_is_replaced)
{
  ngap->handle_message(generate_overload_start(asn1::ngap::overload_action_opts::reject_rrc_cr_sig, 30));
  ngap->handle_message(generate_overload_start(
      asn1::ngap::overload_action_opts::permit_emergency_sessions_and_mobile_terminated_services_only));

  const std::optional<ngap_amf_overload>& overload = ngap->get_ngap_context().overload;
  ASSERT_TRUE(overload.has_value());
  ASSERT_EQ(overload->action, ngap_overload_action_t::only_emergency_and_mt);
  ASSERT_FALSE(overload->traffic_load_reduction.has_value());
}

TEST_F(ngap_overload_test, when_overload_stop_is_received_then_amf_is_not_overloaded)
{
  ngap->handle_message(generate_overload_start(asn1::ngap::overload_action_opts::reject_rrc_cr_sig));
  ngap->handle_message(generate_overload_stop());

  ASSERT_FALSE(ngap->get_ngap_context().overload.has_value());
}

TEST_F(ngap_overload_test, when_overload_start_has_nssai_list_then_slice_overload_is_stored)
{
  ngap_message                            msg = generate_overload_start();
  asn1::ngap::overload_start_nssai_item_s item;
  asn1::ngap::slice_overload_item_s       slice;
  slice.s_nssai.sst.from_number(2);
  slice.s_nssai.sd_present = true;
  slice.s_nssai.sd.from_number(0x10);
  item.slice_overload_list.push_back(slice);
  item.slice_overload_resp_present                     = true;
  item.slice_overload_resp.set_overload_action().value = asn1::ngap::overload_action_opts::reject_non_emergency_mo_dt;
  item.slice_traffic_load_reduction_ind_present        = true;
  item.slice_traffic_load_reduction_ind                = 40;
  auto& overload_start                                 = msg.pdu.init_msg().value.overload_start();
  overload_start->overload_start_nssai_list_present    = true;
  overload_start->overload_start_nssai_list.push_back(item);
  ngap->handle_message(msg);

  const std::optional<ngap_amf_overload>& overload = ngap->get_ngap_context().overload;
  ASSERT_TRUE(overload.has_value());
  ASSERT_EQ(overload->overload_start_nssai_list.size(), 1);
  const ngap_overload_start_nssai_item& stored = overload->overload_start_nssai_list[0];
  ASSERT_EQ(stored.slice_overload_list.size(), 1);
  ASSERT_EQ(stored.slice_overload_list[0].sst.value(), 2);
  ASSERT_EQ(stored.slice_overload_list[0].sd.value(), 0x10);
  ASSERT_EQ(stored.slice_overload_action, ngap_overload_action_t::reject_non_emergency_mo_data);
  ASSERT_EQ(stored.slice_traffic_load_reduction, 40);
}
