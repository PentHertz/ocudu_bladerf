// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ngap_test_helpers.h"
#include "ocudu/asn1/ngap/ngap_pdu_contents.h"
#include <gtest/gtest.h>

using namespace ocudu;
using namespace ocucp;

class ngap_amf_configuration_update_test : public ngap_test
{
protected:
  ngap_amf_configuration_update_test() { report_fatal_error_if_not(run_ng_setup(), "NG Setup failed"); }

  /// Checks that the last message to the AMF is an AMF Configuration Update Acknowledge.
  bool last_message_is_acknowledge() const
  {
    const ngap_message& msg = n2_gw.last_ngap_msgs.back();
    return msg.pdu.type().value == asn1::ngap::ngap_pdu_c::types_opts::successful_outcome and
           msg.pdu.successful_outcome().value.type().value ==
               asn1::ngap::ngap_elem_procs_o::successful_outcome_c::types_opts::amf_cfg_upd_ack;
  }
};

TEST_F(ngap_amf_configuration_update_test, when_update_is_received_then_amf_information_is_replaced_and_acknowledged)
{
  ngap_message msg         = generate_amf_configuration_update();
  auto&        update      = msg.pdu.init_msg().value.amf_cfg_upd();
  update->amf_name_present = true;
  update->amf_name.from_string("amf-1");
  update->relative_amf_capacity_present = true;
  update->relative_amf_capacity         = 10;
  update->served_guami_list_present     = true;
  asn1::ngap::served_guami_item_s served_guami_item;
  served_guami_item.guami.plmn_id = plmn_identity::test_value().to_bytes();
  served_guami_item.guami.amf_region_id.from_number(2);
  served_guami_item.guami.amf_set_id.from_number(3);
  served_guami_item.guami.amf_pointer.from_number(4);
  update->served_guami_list.push_back(served_guami_item);
  update->plmn_support_list_present = true;
  asn1::ngap::plmn_support_item_s plmn_support_item;
  plmn_support_item.plmn_id = plmn_identity::parse("99902").value().to_bytes();
  update->plmn_support_list.push_back(plmn_support_item);
  ngap->handle_message(msg);

  ASSERT_TRUE(last_message_is_acknowledge());
  const ngap_context_t& context = ngap->get_ngap_context();
  ASSERT_EQ(context.amf_name, "amf-1");
  ASSERT_EQ(context.relative_amf_capacity, 10);
  ASSERT_EQ(context.served_guami_list.size(), 1);
  ASSERT_EQ(context.served_guami_list[0].guami.amf_set_id, 3);
  ASSERT_EQ(context.served_guami_list[0].guami.amf_pointer, 4);
  ASSERT_EQ(context.plmn_support_list.size(), 1);
  ASSERT_EQ(context.plmn_support_list[0].plmn_id, plmn_identity::parse("99902").value());
}

TEST_F(ngap_amf_configuration_update_test, when_update_has_no_ie_then_amf_information_is_kept)
{
  const uint16_t capacity = ngap->get_ngap_context().relative_amf_capacity;

  ngap->handle_message(generate_amf_configuration_update());

  ASSERT_TRUE(last_message_is_acknowledge());
  ASSERT_EQ(ngap->get_ngap_context().relative_amf_capacity, capacity);
  ASSERT_EQ(ngap->get_ngap_context().served_guami_list.size(), 1);
  ASSERT_EQ(ngap->get_ngap_context().plmn_support_list.size(), 1);
}

TEST_F(ngap_amf_configuration_update_test, when_update_adds_tnl_associations_then_they_are_reported_as_failed)
{
  ngap_message msg                          = generate_amf_configuration_update();
  auto&        update                       = msg.pdu.init_msg().value.amf_cfg_upd();
  update->amf_tnl_assoc_to_add_list_present = true;
  asn1::ngap::amf_tnl_assoc_to_add_item_s item;
  item.amf_tnl_assoc_address.set_endpoint_ip_address().from_number(0x7f000001, 32);
  update->amf_tnl_assoc_to_add_list.push_back(item);
  ngap->handle_message(msg);

  ASSERT_TRUE(last_message_is_acknowledge());
  const auto& ack = n2_gw.last_ngap_msgs.back().pdu.successful_outcome().value.amf_cfg_upd_ack();
  ASSERT_TRUE(ack->amf_tnl_assoc_failed_to_setup_list_present);
  ASSERT_EQ(ack->amf_tnl_assoc_failed_to_setup_list.size(), 1);
}

TEST_F(ngap_amf_configuration_update_test, when_update_has_invalid_plmn_then_it_is_rejected_and_ignored)
{
  ngap_message msg                  = generate_amf_configuration_update();
  auto&        update               = msg.pdu.init_msg().value.amf_cfg_upd();
  update->plmn_support_list_present = true;
  asn1::ngap::plmn_support_item_s plmn_support_item;
  plmn_support_item.plmn_id.from_string("ffffff");
  update->plmn_support_list.push_back(plmn_support_item);
  ngap->handle_message(msg);

  const ngap_message& reply = n2_gw.last_ngap_msgs.back();
  ASSERT_EQ(reply.pdu.type().value, asn1::ngap::ngap_pdu_c::types_opts::unsuccessful_outcome);
  ASSERT_EQ(reply.pdu.unsuccessful_outcome().value.type().value,
            asn1::ngap::ngap_elem_procs_o::unsuccessful_outcome_c::types_opts::amf_cfg_upd_fail);
  ASSERT_EQ(ngap->get_ngap_context().plmn_support_list[0].plmn_id, plmn_identity::test_value());
}
