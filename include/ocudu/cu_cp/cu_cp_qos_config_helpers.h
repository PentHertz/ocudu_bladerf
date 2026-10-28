// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/cu_cp/cu_cp_types.h"
#include "ocudu/ran/qos/five_qi.h"
#include <map>

namespace ocudu::config_helpers {

/// Creates a Tx configuration with the given SN size and discard timer value.
inline pdcp_config::tx_config make_tx_configuration(pdcp_sn_size sn_size, pdcp_discard_timer discard_timer)
{
  return pdcp_config::tx_config{.sn_size                = sn_size,
                                .direction              = pdcp_security_direction::downlink,
                                .discard_timer          = discard_timer,
                                .status_report_required = false};
}

/// Creates a Rx configuration with the given SN size and reordering value.
inline pdcp_config::rx_config make_rx_configuration(pdcp_sn_size sn_size, pdcp_t_reordering reordering)
{
  return pdcp_config::rx_config{.sn_size               = sn_size,
                                .direction             = pdcp_security_direction::uplink,
                                .out_of_order_delivery = false,
                                .t_reordering          = reordering};
}

/// Creates a PDCP default custom configuration.
inline pdcp_custom_config make_pdcp_default_custom_config()
{
  return {.metrics_period   = std::chrono::milliseconds(0U),
          .metrics_notifier = nullptr,
          .tx               = pdcp_custom_config_tx{.warn_on_drop = false, .test_mode = false},
          .rx               = pdcp_custom_config_rx{.warn_on_drop = false}};
}

/// Creates a PDCP configuration with the given parameters.
inline pdcp_config make_pdcp_config(pdcp_rb_type       rb_type,
                                    pdcp_rlc_mode      rlc_mode,
                                    pdcp_sn_size       sn_size,
                                    pdcp_discard_timer discard_timer,
                                    pdcp_t_reordering  reordering)
{
  return {.rb_type                       = rb_type,
          .rlc_mode                      = rlc_mode,
          .header_compression            = std::nullopt,
          .integrity_protection_required = false,
          .ciphering_required            = true,
          .tx                            = make_tx_configuration(sn_size, discard_timer),
          .rx                            = make_rx_configuration(sn_size, reordering),
          .custom                        = make_pdcp_default_custom_config()};
}

/// \brief Generates default QoS configuration used by gNB CU-CP. The default configuration should be valid.
/// The default values are picked to try to achieve the QoS requirements defined in
/// TS 23.501 -- System architecture for the 5G System, table 5.7.4-1.
///
/// Dependencies between timers should be considered:
///   * discardTimer: How long the PDCP will wait for a transmit/acknowledgement notification,
///                   before requesting a discard to the RLC. Should not exceed the packet
///                   delay budget, if the 5QI is delay sensitive.
///   * t-Reordering: How long the PDCP will wait for an out-of-order PDU. When using RLC UM,
///                   this value should be larger than the RLC's t-Reassembly. When using AM,
///                   this value should be larger than a few RLC retransmissions, see the RLC
///                   timers for details.
inline std::unordered_map<five_qi_t, ocucp::cu_cp_qos_config> make_default_cu_cp_qos_config_list()
{
  std::unordered_map<five_qi_t, ocucp::cu_cp_qos_config> qos_list;
  //
  // Guaranteed Bitrate 5QIs.
  //

  // 5QI=1 e.g. Conversational Voice.
  // PDB=100ms PER=10^-2.
  qos_list[uint_to_five_qi(1)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::um,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms100,
                                                           pdcp_t_reordering::ms1)};

  // 5QI=2 e.g. conversational video.
  // PDB=150ms PER=10^-3.
  qos_list[uint_to_five_qi(2)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::um,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms150,
                                                           pdcp_t_reordering::ms1)};

  // 5QI=3 e.g. real time gaming.
  // PDB=50ms PER=10^-3.
  qos_list[uint_to_five_qi(3)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::um,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms50,
                                                           pdcp_t_reordering::ms1)};

  // 5QI = 4 e.g. non-conversational video.
  // PDB = 300ms PER = 10^-6.
  qos_list[uint_to_five_qi(4)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::am,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms300,
                                                           pdcp_t_reordering::ms140)};

  // 5QI = 65 e.g. Mission Critical user plane Push To Talk.
  // PDB = 75ms PER = 10^-2.
  qos_list[uint_to_five_qi(65)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms75,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 66 e.g. Non-Mission-Critical user plane Push To Talk voice.
  // PDB = 100ms PER = 10^-2.
  qos_list[uint_to_five_qi(66)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms100,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 67 e.g. Mission Critical Video user plane.
  // PDB = 100ms PER = 10^-3.
  qos_list[uint_to_five_qi(67)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms100,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 71 e.g "Live" Uplink Streaming.
  // PDB = 150ms PER = 10^-6.
  qos_list[uint_to_five_qi(71)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::am,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms150,
                                                            pdcp_t_reordering::ms80)};

  // 5QI = 72 e.g "Live" Uplink Streaming.
  // PDB = 300ms PER = 10^-4.
  qos_list[uint_to_five_qi(72)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::am,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms300,
                                                            pdcp_t_reordering::ms140)};

  // 5QI = 73 e.g "Live" Uplink Streaming.
  // PDB = 300ms PER = 10^-8.
  qos_list[uint_to_five_qi(73)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::am,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms300,
                                                            pdcp_t_reordering::ms140)};

  // 5QI = 74 e.g "Live" Uplink Streaming.
  // PDB = 500ms PER = 10^-8.
  qos_list[uint_to_five_qi(74)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::am,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms500,
                                                            pdcp_t_reordering::ms220)};

  // 5QI = 76 e.g "Live" Uplink Streaming.
  // PDB = 500ms PER = 10^-4.
  qos_list[uint_to_five_qi(76)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::am,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms500,
                                                            pdcp_t_reordering::ms220)};

  //
  // Non-Guaranteed Bitrate 5QIs.
  //

  // 5QI = 5 e.g IMS signaling.
  // PDB = 100ms PER = 10^-6.
  qos_list[uint_to_five_qi(5)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::am,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms100,
                                                           pdcp_t_reordering::ms500)};

  // 5QI = 6 e.g. Video (Buffered Streaming).
  // PDB = 300ms PER = 10^-6.
  qos_list[uint_to_five_qi(6)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::am,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms300,
                                                           pdcp_t_reordering::ms100)};

  // 5QI = 7 e.g. Voice,Video (Live Streaming).
  // PDB = 100ms PER = 10^-3.
  qos_list[uint_to_five_qi(7)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::um,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms1500,
                                                           pdcp_t_reordering::ms50)};

  // 5QI = 8 e.g. Video (Buffered Streaming).
  // PDB = 300ms PER = 10^-6.
  qos_list[uint_to_five_qi(8)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::am,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms300,
                                                           pdcp_t_reordering::ms100)};

  // 5QI = 9 e.g. Video (Buffered Streaming).
  // PDB = 300ms PER = 10^-6.
  qos_list[uint_to_five_qi(9)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                           pdcp_rlc_mode::am,
                                                           pdcp_sn_size::size18bits,
                                                           pdcp_discard_timer::ms1500,
                                                           pdcp_t_reordering::ms220)};

  // 5QI = 10 e.g. Video (Buffered Streaming).
  // PDB = 1100ms PER = 10^-6.
  qos_list[uint_to_five_qi(10)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::am,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms1500,
                                                            pdcp_t_reordering::ms500)};

  // 5QI = 69 e.g Mission Critical delay sensitive signalling.
  // PDB = 60ms PER = 10^-6.
  qos_list[uint_to_five_qi(69)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::am,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms60,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 70 e.g Mission Critical Data.
  // PDB = 200ms PER = 10^-6.
  qos_list[uint_to_five_qi(70)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::am,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms200,
                                                            pdcp_t_reordering::ms100)};

  // 5QI = 79 e.g V2X messages.
  // PDB = 50ms PER = 10^-2.
  qos_list[uint_to_five_qi(79)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size18bits,
                                                            pdcp_discard_timer::ms50,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 80 e.g Low Latency eMBB Applications.
  // PDB = 10ms PER = 10^-6.
  qos_list[uint_to_five_qi(80)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms10,
                                                            pdcp_t_reordering::ms1)};

  //
  // Delay-critical Guaranteed Bitrate 5QIs.
  //

  // 5QI = 82 e.g Discrete Automation.
  // PDB = 10ms PER = 10^-4.
  qos_list[uint_to_five_qi(82)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms10,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 83 e.g Discrete Automation.
  // PDB = 10ms PER = 10^-4.
  qos_list[uint_to_five_qi(83)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms10,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 84 e.g Intelligent transport systems.
  // PDB = 30ms PER = 10^-5.
  qos_list[uint_to_five_qi(84)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms30,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 85 e.g. Electricity Distribution-high voltage.
  // PDB = 5ms PER = 10^-5.
  qos_list[uint_to_five_qi(85)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms10,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 86 e.g V2X messages.
  // PDB = 5ms PER = 10^-4.
  qos_list[uint_to_five_qi(86)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms10,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 87 e.g. Interactive Service-Motion tracking data.
  // PDB = 5ms PER = 10^-3.
  qos_list[uint_to_five_qi(87)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms10,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 88 e.g. Interactive Service-Motion tracking data.
  // PDB = 10ms PER = 10^-3.
  qos_list[uint_to_five_qi(88)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms10,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 89 e.g. Visual content for cloud/edge/split rendering.
  // PDB = 15ms PER = 10^-4.
  qos_list[uint_to_five_qi(89)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms20,
                                                            pdcp_t_reordering::ms1)};

  // 5QI = 90 e.g. Visual content for cloud/edge/split rendering.
  // PDB = 20ms PER = 10^-4.
  qos_list[uint_to_five_qi(90)] = {.pdcp = make_pdcp_config(pdcp_rb_type::drb,
                                                            pdcp_rlc_mode::um,
                                                            pdcp_sn_size::size12bits,
                                                            pdcp_discard_timer::ms20,
                                                            pdcp_t_reordering::ms1)};

  return qos_list;
}

} // namespace ocudu::config_helpers
