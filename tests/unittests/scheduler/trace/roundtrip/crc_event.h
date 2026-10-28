// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "common.h"         // IWYU pragma: keep
#include "harq_ack_event.h" // IWYU pragma: keep
#include "lib/scheduler/trace/event_converter.h"
#include "rach_indication.h" // IWYU pragma: keep
#include "roundtrip_test.h"
#include "ocudu/scheduler/input/crc_inputs.h"

namespace ocudu::schedtrace::roundtrip_test {

template <>
struct test_values<ul_crc_status> {
  static std::vector<ul_crc_status> get() { return {ul_crc_status::ok, ul_crc_status::ko, ul_crc_status::dtx}; }
};

template <>
struct roundtrip_traits<ul_crc_event> {
  static constexpr auto members = std::make_tuple(field("ue_index", &ul_crc_event::ue_index),
                                                  field("rnti", &ul_crc_event::rnti),
                                                  field("cell_index", &ul_crc_event::cell_index),
                                                  field("sl_rx", &ul_crc_event::sl_rx),
                                                  field("h_id", &ul_crc_event::h_id),
                                                  field("crc", &ul_crc_event::crc),
                                                  field("tbs", &ul_crc_event::tbs),
                                                  field("ul_sinr_db", &ul_crc_event::ul_sinr_db),
                                                  field("ul_rsrp_dbfs", &ul_crc_event::ul_rsrp_dbfs),
                                                  field("time_advance_offset", &ul_crc_event::time_advance_offset));

  static ul_crc_event roundtrip(const ul_crc_event& event)
  {
    return roundtrip_via<fbs::CrcEvent>(
        event, convert_ul_crc_event_to_fb, [&event](ul_crc_event& result, const fbs::CrcEvent& fb_event) {
          convert_fb_to_ul_crc_event(result, fb_event, event.sl_rx.scs(), event.cell_index);
        });
  }
};

} // namespace ocudu::schedtrace::roundtrip_test
