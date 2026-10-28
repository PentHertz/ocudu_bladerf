// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ngap_test_helpers.h"
#include "ocudu/asn1/ngap/ngap_pdu_contents.h"
#include <gtest/gtest.h>

using namespace ocudu;
using namespace ocucp;

class ngap_amf_status_indication_test : public ngap_test
{
protected:
  ngap_amf_status_indication_test() { report_fatal_error_if_not(run_ng_setup(), "NG Setup failed"); }

  /// GUAMI that the AMF serves after the NG Setup.
  const guami_t served_guami = {.plmn          = plmn_identity::test_value(),
                                .amf_set_id    = 1,
                                .amf_pointer   = 0,
                                .amf_region_id = 2};
};

TEST_F(ngap_amf_status_indication_test, when_indication_is_received_then_unavailable_guami_is_stored)
{
  ngap->handle_message(generate_amf_status_indication(served_guami, "amf-backup"));

  const ngap_context_t& context = ngap->get_ngap_context();
  ASSERT_EQ(context.unavailable_guami_list.size(), 1);
  ASSERT_EQ(context.unavailable_guami_list[0].guami, served_guami);
  ASSERT_EQ(context.unavailable_guami_list[0].backup_amf_name, "amf-backup");
}

TEST_F(ngap_amf_status_indication_test, when_indication_repeats_a_guami_then_the_stored_guami_is_replaced)
{
  ngap->handle_message(generate_amf_status_indication(served_guami, "amf-backup"));
  ngap->handle_message(generate_amf_status_indication(served_guami));

  const ngap_context_t& context = ngap->get_ngap_context();
  ASSERT_EQ(context.unavailable_guami_list.size(), 1);
  ASSERT_FALSE(context.unavailable_guami_list[0].backup_amf_name.has_value());
}

TEST_F(ngap_amf_status_indication_test, when_served_guami_list_is_updated_then_unavailable_guamis_are_cleared)
{
  ngap->handle_message(generate_amf_status_indication(served_guami));

  ngap_message msg                  = generate_amf_configuration_update();
  auto&        update               = msg.pdu.init_msg().value.amf_cfg_upd();
  update->served_guami_list_present = true;
  asn1::ngap::served_guami_item_s served_guami_item;
  served_guami_item.guami.plmn_id = served_guami.plmn.to_bytes();
  served_guami_item.guami.amf_region_id.from_number(served_guami.amf_region_id);
  served_guami_item.guami.amf_set_id.from_number(served_guami.amf_set_id);
  served_guami_item.guami.amf_pointer.from_number(served_guami.amf_pointer);
  update->served_guami_list.push_back(served_guami_item);
  ngap->handle_message(msg);

  ASSERT_TRUE(ngap->get_ngap_context().unavailable_guami_list.empty());
}
