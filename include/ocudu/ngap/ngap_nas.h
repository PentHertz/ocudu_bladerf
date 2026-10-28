// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/adt/byte_buffer.h"
#include "ocudu/ran/cu_cp_types.h"

namespace ocudu::ocucp {

struct ngap_dl_nas_transport_message {
  cu_cp_ue_index_t ue_index = cu_cp_ue_index_t::invalid;
  byte_buffer      nas_pdu;
  bool             ue_cap_info_request = false;
};

/// Request of the source AMF to reroute the INITIAL UE MESSAGE of a UE to a target AMF of an AMF Set (TS 38.413
/// section 8.6.5).
struct ngap_reroute_nas_request {
  cu_cp_ue_index_t ue_index = cu_cp_ue_index_t::invalid;
  /// AMF Set ID of the target AMF.
  uint16_t amf_set_id = 0;
  /// Packed REROUTE NAS REQUEST. The NGAP of the target AMF builds the rerouted INITIAL UE MESSAGE from it.
  byte_buffer packed_reroute_nas_request;
};

} // namespace ocudu::ocucp
