// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ran/plmn_identity.h"

namespace ocudu {

/// 3GPP TS 38.413 section 9.3.3.3, GUAMI (Globally unique AMF identifier)
struct guami_t {
  plmn_identity plmn          = plmn_identity::test_value();
  uint16_t      amf_set_id    = 0;
  uint8_t       amf_pointer   = 0;
  uint8_t       amf_region_id = 0;

  bool operator==(const guami_t& other) const
  {
    return plmn == other.plmn and amf_set_id == other.amf_set_id and amf_pointer == other.amf_pointer and
           amf_region_id == other.amf_region_id;
  }
  bool operator!=(const guami_t& other) const { return not(*this == other); }
};

/// Origin of a GUAMI: native, or mapped from an EPS GUTI (TS 38.413 section 9.2.6.2, TS 38.331 section 6.2.2).
enum class guami_type_t { native, mapped };

} // namespace ocudu
