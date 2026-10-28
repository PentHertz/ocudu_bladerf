// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ngap/ngap_overload.h"
#include "ocudu/ngap/ngap_setup.h"
#include "ocudu/ran/cu_cp_types.h"
#include "ocudu/ran/gnb_id.h"
#include "ocudu/ran/guami.h"
#include "ocudu/ran/plmn_identity.h"
#include "ocudu/ran/supported_tracking_area.h"
#include "ocudu/support/io/transport_layer_address.h"
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace ocudu::ocucp {

/// GUAMI that the AMF marks as unavailable (TS 38.413 section 9.2.6.10).
struct ngap_unavailable_guami_item {
  guami_t                    guami;
  std::optional<std::string> backup_amf_name;
};

/// \brief NGAP context
struct ngap_context_t {
  gnb_id_t                             gnb_id = {0, 22};
  std::string                          ran_node_name;
  std::string                          amf_name;
  cu_cp_amf_index_t                    amf_index;
  std::vector<supported_tracking_area> supported_tas;
  transport_layer_address              amf_addr; // address of the SCTP association with the AMF
  /// GUAMIs that the AMF serves (TS 38.413 section 9.2.6.2).
  std::vector<ngap_served_guami_item> served_guami_list;
  /// Capacity of the AMF relative to the other AMFs of its AMF Set (TS 38.413 section 9.3.1.32).
  uint16_t relative_amf_capacity = 0;
  /// PLMNs and slices that the AMF supports (TS 38.413 section 9.2.6.2).
  std::vector<ngap_plmn_support_item> plmn_support_list;
  /// GUAMIs that the AMF marks as unavailable (TS 38.413 section 8.7.6.2). A new served GUAMI list clears it.
  std::vector<ngap_unavailable_guami_item> unavailable_guami_list;
  /// Overload state of the AMF. Absent if the AMF is not overloaded (TS 38.413 sections 8.7.7 and 8.7.8).
  std::optional<ngap_amf_overload> overload;
  uint16_t                         default_paging_drx = 256; // default paging drx
  std::chrono::milliseconds procedure_timeout; // Time that the NGAP waits for a response from the AMF in milliseconds
  std::chrono::seconds      request_pdu_session_timeout; // timeout for requesting a PDU session in seconds
  byte_buffer               lmf_routing_id;

  std::vector<plmn_identity> get_supported_plmns() const
  {
    std::vector<plmn_identity> supported_plmns;
    for (const auto& ta : supported_tas) {
      for (const auto& plmn : ta.plmn_list) {
        supported_plmns.push_back(plmn.plmn_id);
      }
    }

    return supported_plmns;
  }
};

} // namespace ocudu::ocucp
