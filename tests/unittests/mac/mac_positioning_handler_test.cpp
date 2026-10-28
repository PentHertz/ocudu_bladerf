// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "lib/mac/mac_sched/positioning_handler.h"
#include "tests/ocudu_test_requirements.h"
#include "tests/test_doubles/utils/test_rng.h"
#include "ocudu/adt/format.h"
#include "ocudu/adt/span.h"
#include "ocudu/mac/mac_positioning_measurement_handler.h"
#include "ocudu/ocudulog/ocudulog.h"
#include "ocudu/scheduler/scheduler_positioning_handler.h"
#include "ocudu/support/async/async_test_utils.h"
#include "ocudu/support/executors/manual_task_worker.h"
#include <deque>
#include <gtest/gtest.h>

using namespace ocudu;

namespace {

class dummy_scheduler_positioning_handler : public scheduler_positioning_handler
{
public:
  std::optional<positioning_measurement_request>    last_req;
  std::vector<positioning_measurement_request>      received_reqs;
  std::vector<positioning_measurement_stop_request> stop_reqs;

  void handle_positioning_measurement_request(const positioning_measurement_request& req) override
  {
    last_req = req;
    received_reqs.push_back(req);
  }

  void handle_positioning_measurement_stop(const positioning_measurement_stop_request& req) override
  {
    stop_reqs.push_back(req);
  }
};

srs_config make_test_srs_config(unsigned sequence_id = 1)
{
  srs_config cfg{};
  cfg.srs_res_list.resize(1);
  cfg.srs_res_list[0].id.ue_res_id   = srs_config::MIN_SRS_RES_ID;
  cfg.srs_res_list[0].id.cell_res_id = 0;
  cfg.srs_res_list[0].sequence_id    = sequence_id;
  cfg.srs_res_set_list.resize(1);
  cfg.srs_res_set_list[0].res_type.emplace<srs_config::srs_resource_set::periodic_resource_type>();
  return cfg;
}

mac_srs_indication_message make_srs_ind(rnti_t               rnti,
                                        std::optional<float> azimuth_aoa_deg = std::nullopt,
                                        std::optional<float> zenith_aoa_deg  = std::nullopt)
{
  mac_srs_indication_message msg;
  msg.sl_rx = {0, 0};
  msg.srss.resize(1);
  msg.srss[0].rnti   = rnti;
  msg.srss[0].report = mac_srs_pdu::positioning_report{
      phy_time_unit::from_units_of_Tc(1000), std::nullopt, azimuth_aoa_deg, zenith_aoa_deg};
  return msg;
}

/// Creates an SRS indication carrying a positioning report with UL-AoA only, i.e. without UL-RTOA.
mac_srs_indication_message make_aoa_only_srs_ind(rnti_t rnti, float azimuth_aoa_deg)
{
  mac_srs_indication_message msg;
  msg.sl_rx = {0, 0};
  msg.srss.resize(1);
  msg.srss[0].rnti   = rnti;
  msg.srss[0].report = mac_srs_pdu::positioning_report{std::nullopt, std::nullopt, azimuth_aoa_deg, std::nullopt};
  return msg;
}

/// Creates an SRS indication carrying one positioning report per given RNTI, each with its own azimuth.
mac_srs_indication_message make_multi_ue_srs_ind(span<const std::pair<rnti_t, float>> rnti_azimuths)
{
  mac_srs_indication_message msg;
  msg.sl_rx = {0, 0};
  for (const auto& [rnti, azimuth_deg] : rnti_azimuths) {
    auto& pdu = msg.srss.emplace_back();
    pdu.rnti  = rnti;
    pdu.report =
        mac_srs_pdu::positioning_report{phy_time_unit::from_units_of_Tc(1000), std::nullopt, azimuth_deg, std::nullopt};
  }
  return msg;
}

/// Number of reserved RNTIs that a cell uses for the measurements of neighbor UEs.
constexpr unsigned nof_reserved_rntis =
    static_cast<unsigned>(rnti_t::MAX_RESERVED_RNTI) - static_cast<unsigned>(rnti_t::MIN_RESERVED_RNTI) + 1;

mac_positioning_measurement_request make_positioning_request_for_neighbor_ue()
{
  mac_positioning_measurement_request req;
  req.cells.resize(1);
  req.cells[0].cell_index  = to_du_cell_index(0);
  req.cells[0].srs_to_meas = make_test_srs_config();
  return req;
}

mac_positioning_measurement_request make_positioning_request_for_connected_ue(unsigned nof_cells = 1)
{
  mac_positioning_measurement_request req;
  req.cells.resize(nof_cells);
  for (unsigned i = 0; i != nof_cells; ++i) {
    req.cells[i].cell_index  = to_du_cell_index(i);
    req.cells[i].rnti        = to_rnti(0x4601 + i);
    req.cells[i].ue_index    = to_du_ue_index(i);
    req.cells[i].srs_to_meas = make_test_srs_config();
  }
  return req;
}

/// Creates a request for a second UE connected to cell 0.
mac_positioning_measurement_request make_positioning_request_for_second_connected_ue()
{
  mac_positioning_measurement_request req = make_positioning_request_for_connected_ue();
  req.cells[0].rnti                       = to_rnti(0x4602);
  req.cells[0].ue_index                   = to_du_ue_index(1);
  return req;
}

} // namespace

class positioning_handler_test
{
protected:
  positioning_handler_test() :
    logger(ocudulog::fetch_basic_logger("SCHED")),
    pos_handler(create_positioning_handler(sched, worker, timers, logger))
  {
  }

  dummy_scheduler_positioning_handler sched;
  manual_task_worker                  worker{128};
  timer_manager                       timers;
  ocudulog::basic_logger&             logger;

  std::unique_ptr<positioning_handler> pos_handler;

  std::vector<std::unique_ptr<cell_positioning_handler>> cells;

  /// Tasks of the requests, and the launchers that wait for their responses. A launcher refers to its task, so both
  /// stay in place.
  std::deque<async_task<mac_positioning_measurement_response>>         tasks;
  std::deque<lazy_task_launcher<mac_positioning_measurement_response>> launchers;

  /// Sends a positioning measurement request and starts waiting for its response.
  lazy_task_launcher<mac_positioning_measurement_response>& launch(const mac_positioning_measurement_request& req)
  {
    tasks.push_back(pos_handler->handle_positioning_measurement_request(req));
    return launchers.emplace_back(tasks.back());
  }

  /// Takes every reserved RNTI of the given cell with measurements of neighbor UEs.
  std::vector<lazy_task_launcher<mac_positioning_measurement_response>*>
  exhaust_reserved_rntis(du_cell_index_t cell_index)
  {
    mac_positioning_measurement_request req = make_positioning_request_for_neighbor_ue();
    req.cells[0].cell_index                 = cell_index;
    std::vector<lazy_task_launcher<mac_positioning_measurement_response>*> ret;
    for (unsigned i = 0; i != nof_reserved_rntis; ++i) {
      ret.push_back(&launch(req));
    }
    return ret;
  }

  /// Advances time by \c nof_ms milliseconds and runs the tasks that the expired timers scheduled.
  void tick(unsigned nof_ms)
  {
    for (unsigned i = 0; i != nof_ms; ++i) {
      timers.tick();
      worker.run_pending_tasks();
    }
  }
};

class single_cell_positioning_handler_test : public positioning_handler_test, public ::testing::Test
{
public:
  single_cell_positioning_handler_test() { cells.push_back(pos_handler->add_cell(to_du_cell_index(0))); }
};

TEST_F(single_cell_positioning_handler_test,
       when_connected_ue_initiates_positioning_measurement_then_scheduler_is_notified)
{
  mac_positioning_measurement_request req = make_positioning_request_for_connected_ue();
  launch(req);

  // Scheduler gets notified.
  ASSERT_TRUE(sched.last_req.has_value());
  ASSERT_EQ(sched.last_req->cells.size(), 1);
  const auto& cell_req = sched.last_req->cells.front();
  ASSERT_EQ(cell_req.ue_index, req.cells[0].ue_index);
  ASSERT_EQ(cell_req.pos_rnti, req.cells[0].rnti);
  ASSERT_TRUE(is_crnti(cell_req.pos_rnti));
  ASSERT_EQ(cell_req.srs_to_measure, req.cells[0].srs_to_meas);
}

TEST_F(single_cell_positioning_handler_test,
       when_positioning_starts_for_neighbor_cell_ue_then_scheduler_is_notified_with_fake_rnti)
{
  mac_positioning_measurement_request req = make_positioning_request_for_neighbor_ue();

  launch(req);

  // Scheduler gets notified.
  ASSERT_TRUE(sched.last_req.has_value());
  ASSERT_EQ(sched.last_req->cells.size(), 1);
  const auto& cell_req = sched.last_req->cells.front();
  ASSERT_FALSE(cell_req.ue_index.has_value());
  ASSERT_FALSE(is_crnti(cell_req.pos_rnti));
  ASSERT_EQ(cell_req.srs_to_measure, req.cells[0].srs_to_meas);
}

TEST_F(single_cell_positioning_handler_test,
       when_connected_ue_initiates_positioning_measurement_then_it_only_completes_when_measurement_completes)
{
  OCUDU_TEST_REQUIREMENTS("MVP-FUNC-POS-16-3-b");

  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  rnti_t                              rnti = req.cells[0].rnti.value();

  auto& t_launcher = launch(req);

  // Procedure has not completed yet.
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher.ready());
  ASSERT_TRUE(sched.stop_reqs.empty());

  // Forward SRS indication.
  mac_srs_indication_message srs_ind = make_srs_ind(rnti);
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();

  // Positioning has been completed.
  ASSERT_TRUE(t_launcher.ready());
  ASSERT_TRUE(t_launcher.result.has_value());
  ASSERT_EQ(t_launcher.result.value().sl_rx, srs_ind.sl_rx);
  ASSERT_EQ(t_launcher.result.value().cell_results.size(), 1);
  ASSERT_EQ(t_launcher.result.value().cell_results[0].ul_srs_pos_meass.size(), 1);
  ASSERT_EQ(t_launcher.result.value().cell_results[0].ul_srs_pos_meass[0].ul_rtoa,
            std::get<mac_srs_pdu::positioning_report>(srs_ind.srss[0].report).ul_rtoa.value());

  // Scheduler was notified to stop positioning measurements.
  ASSERT_FALSE(sched.stop_reqs.empty());
  ASSERT_EQ(sched.stop_reqs.back().completed_meas.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas[0].second, rnti);
}

TEST_F(single_cell_positioning_handler_test,
       when_positioning_measurement_of_neighbor_cell_ue_is_initiated_then_it_only_completes_when_measurement_completes)
{
  OCUDU_TEST_REQUIREMENTS("MVP-FUNC-POS-16-3-b");

  mac_positioning_measurement_request req = make_positioning_request_for_neighbor_ue();

  auto& t_launcher = launch(req);
  worker.run_pending_tasks();
  ASSERT_TRUE(sched.last_req.has_value());
  ASSERT_EQ(sched.last_req->cells.size(), 1);
  const rnti_t pos_rnti = sched.last_req->cells.front().pos_rnti;

  // Procedure has not completed yet.
  ASSERT_FALSE(t_launcher.ready());
  ASSERT_TRUE(sched.stop_reqs.empty());

  mac_srs_indication_message srs_ind = make_srs_ind(pos_rnti);
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();

  // Positioning has been completed.
  ASSERT_TRUE(t_launcher.ready());
  ASSERT_TRUE(t_launcher.result.has_value());
  ASSERT_EQ(t_launcher.result.value().sl_rx, srs_ind.sl_rx);
  ASSERT_EQ(t_launcher.result.value().cell_results.size(), 1);
  ASSERT_EQ(t_launcher.result.value().cell_results[0].ul_srs_pos_meass.size(), 1);
  ASSERT_EQ(t_launcher.result.value().cell_results[0].ul_srs_pos_meass[0].ul_rtoa,
            std::get<mac_srs_pdu::positioning_report>(srs_ind.srss[0].report).ul_rtoa.value());

  // Scheduler was notified to stop positioning measurements.
  ASSERT_EQ(sched.stop_reqs.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas[0].second, pos_rnti);
}

TEST_F(single_cell_positioning_handler_test,
       when_neighbor_ue_positioning_completes_then_pos_rnti_becomes_available_again)
{
  OCUDU_TEST_REQUIREMENTS("MVP-FUNC-POS-16-3-b");

  // First positioning run.
  mac_positioning_measurement_request req        = make_positioning_request_for_neighbor_ue();
  auto&                               t_launcher = launch(req);
  const rnti_t                        pos_rnti1  = sched.last_req->cells[0].pos_rnti;
  mac_srs_indication_message          srs_ind    = make_srs_ind(pos_rnti1);
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher.ready());

  // Second positioning run.
  auto&        t_launcher2 = launch(req);
  const rnti_t pos_rnti2   = sched.last_req->cells[0].pos_rnti;
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher2.ready()) << "Results were not correctly reset";
  srs_ind = make_srs_ind(pos_rnti2);
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher2.ready());

  // The released RNTIs are reused in release order, so the second run gets another one.
  ASSERT_NE(pos_rnti1, pos_rnti2);

  // Over a full cycle of the reserved RNTIs, no request is discarded and the first RNTI comes back.
  bool first_rnti_reused = false;
  for (unsigned i = 0; i != nof_reserved_rntis; ++i) {
    // Let the quarantine of the earliest released RNTIs end. 14 periods of 10 ms exceed the 128 ms quarantine.
    tick(10);
    const size_t nof_reqs     = sched.received_reqs.size();
    auto&        t_i_launcher = launch(req);
    ASSERT_EQ(sched.received_reqs.size(), nof_reqs + 1) << "A reserved RNTI was not released";
    const rnti_t pos_rnti = sched.last_req->cells[0].pos_rnti;
    first_rnti_reused |= pos_rnti == pos_rnti1;
    cells[0]->handle_srs_indication(make_srs_ind(pos_rnti));
    worker.run_pending_tasks();
    ASSERT_TRUE(t_i_launcher.ready());
  }
  ASSERT_TRUE(first_rnti_reused);
}

TEST_F(single_cell_positioning_handler_test, when_srs_indication_contains_ul_aoa_then_result_carries_ul_aoa)
{
  OCUDU_TEST_REQUIREMENTS("MVP-FUNC-POS-16-3-b", "MVP-FUNC-POS-16-3-c");

  static constexpr float azimuth_deg = 123.4F;
  static constexpr float zenith_deg  = 56.7F;

  // Start a positioning measurement for a UE connected to the cell.
  mac_positioning_measurement_request req        = make_positioning_request_for_connected_ue();
  rnti_t                              rnti       = req.cells[0].rnti.value();
  auto&                               t_launcher = launch(req);

  // Forward an SRS indication carrying both UL-RTOA and UL-AoA.
  mac_srs_indication_message srs_ind = make_srs_ind(rnti, azimuth_deg, zenith_deg);
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();

  // The AoA values are forwarded unchanged in the positioning measurement result.
  ASSERT_TRUE(t_launcher.ready());
  const auto& meas = t_launcher.result.value().cell_results[0].ul_srs_pos_meass[0];
  ASSERT_TRUE(meas.ul_rtoa.has_value());
  ASSERT_TRUE(meas.azimuth_aoa_deg.has_value());
  EXPECT_FLOAT_EQ(meas.azimuth_aoa_deg.value(), azimuth_deg);
  ASSERT_TRUE(meas.zenith_aoa_deg.has_value());
  EXPECT_FLOAT_EQ(meas.zenith_aoa_deg.value(), zenith_deg);
}

TEST_F(single_cell_positioning_handler_test, when_srs_indication_contains_only_ul_aoa_then_it_is_not_dropped)
{
  OCUDU_TEST_REQUIREMENTS("MVP-FUNC-POS-16-3-c");

  static constexpr float azimuth_deg = 200.1F;

  // Start a positioning measurement for a UE connected to the cell.
  mac_positioning_measurement_request req        = make_positioning_request_for_connected_ue();
  rnti_t                              rnti       = req.cells[0].rnti.value();
  auto&                               t_launcher = launch(req);

  // Forward an SRS indication carrying UL-AoA only, i.e. without UL-RTOA.
  mac_srs_indication_message srs_ind = make_aoa_only_srs_ind(rnti, azimuth_deg);
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();

  // The report is not dropped despite missing UL-RTOA.
  ASSERT_TRUE(t_launcher.ready());
  const auto& meas = t_launcher.result.value().cell_results[0].ul_srs_pos_meass[0];
  ASSERT_FALSE(meas.ul_rtoa.has_value());
  ASSERT_TRUE(meas.azimuth_aoa_deg.has_value());
  EXPECT_FLOAT_EQ(meas.azimuth_aoa_deg.value(), azimuth_deg);
}

TEST_F(single_cell_positioning_handler_test, when_no_srs_indication_arrives_then_request_times_out)
{
  mac_positioning_measurement_request req = make_positioning_request_for_connected_ue();
  req.timeout                             = std::chrono::milliseconds{20};

  auto& t_launcher = launch(req);

  // No SRS indication arrives. The request is still pending just before the timeout expires.
  tick(req.timeout.count() - 1);
  ASSERT_FALSE(t_launcher.ready());

  // The guard timer expires and completes the request.
  tick(1);
  ASSERT_TRUE(t_launcher.ready());
}

TEST_F(single_cell_positioning_handler_test, when_request_times_out_then_response_is_empty)
{
  // Run a first measurement to completion, so that a result is stored.
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  rnti_t                              rnti = req.cells[0].rnti.value();
  req.timeout                              = std::chrono::milliseconds{20};

  auto& t_launcher = launch(req);
  cells[0]->handle_srs_indication(make_srs_ind(rnti, 123.4F));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher.ready());
  ASSERT_FALSE(t_launcher.result.value().cell_results.empty());

  // Let a second measurement time out. Its response must not carry the results of the first one.
  auto& t_launcher2 = launch(req);
  tick(req.timeout.count());

  ASSERT_TRUE(t_launcher2.ready());
  ASSERT_TRUE(t_launcher2.result.value().cell_results.empty());
}

TEST_F(single_cell_positioning_handler_test, when_request_times_out_then_scheduler_is_told_to_stop)
{
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  rnti_t                              rnti = req.cells[0].rnti.value();
  req.timeout                              = std::chrono::milliseconds{20};

  auto& t_launcher = launch(req);
  ASSERT_TRUE(sched.stop_reqs.empty());

  tick(req.timeout.count());

  ASSERT_TRUE(t_launcher.ready());
  ASSERT_EQ(sched.stop_reqs.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas[0].second, rnti);
}

TEST_F(single_cell_positioning_handler_test, when_request_times_out_then_a_new_request_is_accepted)
{
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  rnti_t                              rnti = req.cells[0].rnti.value();
  req.timeout                              = std::chrono::milliseconds{20};

  // Let a first request time out.
  auto& t_launcher = launch(req);
  tick(req.timeout.count());
  ASSERT_TRUE(t_launcher.ready());

  // A new request is accepted and completes on the next SRS indication.
  sched.last_req.reset();
  auto& t_launcher2 = launch(req);
  ASSERT_TRUE(sched.last_req.has_value()) << "Request was refused as a duplicate";

  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher2.ready());
  ASSERT_FALSE(t_launcher2.result.value().cell_results.empty());
}

TEST_F(single_cell_positioning_handler_test, when_request_completes_in_time_then_it_does_not_time_out)
{
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  rnti_t                              rnti = req.cells[0].rnti.value();
  req.timeout                              = std::chrono::milliseconds{20};

  auto& t_launcher = launch(req);

  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher.ready());
  ASSERT_EQ(sched.stop_reqs.size(), 1);

  // The guard timer of the completed request does not fire.
  tick(req.timeout.count() * 2);
  ASSERT_EQ(sched.stop_reqs.size(), 1);
}

TEST_F(single_cell_positioning_handler_test, when_srs_indication_arrives_after_the_timeout_then_it_is_ignored)
{
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  rnti_t                              rnti = req.cells[0].rnti.value();
  req.timeout                              = std::chrono::milliseconds{20};

  auto& t_launcher = launch(req);
  tick(req.timeout.count());
  ASSERT_TRUE(t_launcher.ready());

  // The SRS of the timed out request arrives late. It must not complete the next request.
  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();

  auto& t_launcher2 = launch(req);
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher2.ready()) << "A stray SRS indication completed the new request";

  // Only a new SRS indication completes it.
  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher2.ready());
}

TEST_F(single_cell_positioning_handler_test, when_two_ues_are_measured_concurrently_then_each_request_gets_its_own_srs)
{
  static constexpr float azimuth1_deg = 10.0F;
  static constexpr float azimuth2_deg = 20.0F;

  mac_positioning_measurement_request req1  = make_positioning_request_for_connected_ue();
  mac_positioning_measurement_request req2  = make_positioning_request_for_second_connected_ue();
  const rnti_t                        rnti1 = req1.cells[0].rnti.value();
  const rnti_t                        rnti2 = req2.cells[0].rnti.value();

  auto& t_launcher1 = launch(req1);
  auto& t_launcher2 = launch(req2);
  ASSERT_EQ(sched.received_reqs.size(), 2) << "The second request was not forwarded to the scheduler";

  // The SRS of the second UE completes only the second request.
  cells[0]->handle_srs_indication(make_srs_ind(rnti2, azimuth2_deg));
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher1.ready());
  ASSERT_TRUE(t_launcher2.ready());
  ASSERT_EQ(t_launcher2.result.value().cell_results.size(), 1);
  EXPECT_FLOAT_EQ(t_launcher2.result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), azimuth2_deg);
  ASSERT_EQ(sched.stop_reqs.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas.size(), 1);
  EXPECT_EQ(sched.stop_reqs.back().completed_meas[0].second, rnti2);

  // The SRS of the first UE completes the first request.
  cells[0]->handle_srs_indication(make_srs_ind(rnti1, azimuth1_deg));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher1.ready());
  EXPECT_FLOAT_EQ(t_launcher1.result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), azimuth1_deg);
  ASSERT_EQ(sched.stop_reqs.size(), 2);
  EXPECT_EQ(sched.stop_reqs.back().completed_meas[0].second, rnti1);
}

TEST_F(single_cell_positioning_handler_test,
       when_a_late_report_of_a_neighbor_ue_arrives_then_it_does_not_complete_the_measurement_of_another_neighbor_ue)
{
  static constexpr float azimuth_ue_a_deg = 10.0F;
  static constexpr float azimuth_ue_b_deg = 20.0F;

  // The measurement of neighbor UE A completes.
  mac_positioning_measurement_request req          = make_positioning_request_for_neighbor_ue();
  auto&                               t_a_launcher = launch(req);
  const rnti_t                        pos_rnti_a   = sched.last_req->cells[0].pos_rnti;
  cells[0]->handle_srs_indication(make_srs_ind(pos_rnti_a, azimuth_ue_a_deg));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_a_launcher.ready());

  // A late report of UE A waits in the report channel while the measurement of neighbor UE B starts.
  cells[0]->handle_srs_indication(make_srs_ind(pos_rnti_a, azimuth_ue_a_deg));
  auto&        t_b_launcher = launch(req);
  const rnti_t pos_rnti_b   = sched.last_req->cells[0].pos_rnti;
  ASSERT_NE(pos_rnti_a, pos_rnti_b) << "The reserved RNTI of UE A was reused right after its release";

  worker.run_pending_tasks();
  ASSERT_FALSE(t_b_launcher.ready()) << "A late report of UE A completed the measurement of UE B";

  // Only the report of UE B completes its measurement.
  cells[0]->handle_srs_indication(make_srs_ind(pos_rnti_b, azimuth_ue_b_deg));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_b_launcher.ready());
  EXPECT_FLOAT_EQ(t_b_launcher.result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(),
                  azimuth_ue_b_deg);
}

TEST_F(single_cell_positioning_handler_test,
       when_all_free_reserved_rntis_are_in_quarantine_then_a_neighbor_ue_request_is_discarded_until_it_ends)
{
  // Value of the reserved RNTI quarantine in the MAC.
  static constexpr unsigned quarantine_ms = 128;

  // Take every reserved RNTI of the cell.
  const auto meas_launchers = exhaust_reserved_rntis(to_du_cell_index(0));
  ASSERT_EQ(sched.received_reqs.size(), nof_reserved_rntis);

  // One measurement completes, and its RNTI enters quarantine.
  const rnti_t released_rnti = sched.received_reqs.front().cells[0].pos_rnti;
  cells[0]->handle_srs_indication(make_srs_ind(released_rnti));
  worker.run_pending_tasks();
  ASSERT_TRUE(meas_launchers.front()->ready());

  mac_positioning_measurement_request req = make_positioning_request_for_neighbor_ue();

  // Before the quarantine ends, a new request for a neighbor UE is discarded.
  tick(quarantine_ms - 1);
  auto& t_early_launcher = launch(req);
  ASSERT_TRUE(t_early_launcher.ready());
  ASSERT_TRUE(t_early_launcher.result.value().cell_results.empty());
  ASSERT_EQ(sched.received_reqs.size(), nof_reserved_rntis);

  // Once the quarantine ends, the request gets the released RNTI.
  tick(1);
  auto& t_late_launcher = launch(req);
  ASSERT_FALSE(t_late_launcher.ready());
  ASSERT_EQ(sched.received_reqs.size(), nof_reserved_rntis + 1);
  ASSERT_EQ(sched.last_req->cells[0].pos_rnti, released_rnti);
}

TEST_F(single_cell_positioning_handler_test,
       when_one_srs_indication_carries_the_reports_of_two_measured_ues_then_both_requests_complete)
{
  static constexpr float azimuth1_deg = 10.0F;
  static constexpr float azimuth2_deg = 20.0F;

  mac_positioning_measurement_request req1 = make_positioning_request_for_connected_ue();
  mac_positioning_measurement_request req2 = make_positioning_request_for_second_connected_ue();

  auto& t_launcher1 = launch(req1);
  auto& t_launcher2 = launch(req2);

  const std::pair<rnti_t, float> reports[] = {{req1.cells[0].rnti.value(), azimuth1_deg},
                                              {req2.cells[0].rnti.value(), azimuth2_deg}};
  cells[0]->handle_srs_indication(make_multi_ue_srs_ind(reports));
  worker.run_pending_tasks();

  ASSERT_TRUE(t_launcher1.ready());
  ASSERT_TRUE(t_launcher2.ready());
  EXPECT_FLOAT_EQ(t_launcher1.result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), azimuth1_deg);
  EXPECT_FLOAT_EQ(t_launcher2.result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), azimuth2_deg);
}

TEST_F(single_cell_positioning_handler_test,
       when_the_rnti_is_already_measured_then_a_second_request_for_it_waits_until_the_first_completes)
{
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  const rnti_t                        rnti = req.cells[0].rnti.value();

  auto& t_launcher1 = launch(req);

  // A second request for the same RNTI with another SRS configuration waits, and does not reach the scheduler.
  mac_positioning_measurement_request other_req = req;
  other_req.cells[0].srs_to_meas                = make_test_srs_config(2);
  auto& t_launcher2                             = launch(other_req);
  ASSERT_FALSE(t_launcher2.ready());
  ASSERT_EQ(sched.received_reqs.size(), 1);
  ASSERT_TRUE(sched.stop_reqs.empty()) << "The queued request stopped the running measurement";

  // The SRS completes the first request, and the second one starts.
  cells[0]->handle_srs_indication(make_srs_ind(rnti, 10.0F));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher1.ready());
  EXPECT_FLOAT_EQ(t_launcher1.result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), 10.0F);
  ASSERT_FALSE(t_launcher2.ready());
  ASSERT_EQ(sched.received_reqs.size(), 2);
  ASSERT_EQ(sched.last_req->cells[0].srs_to_measure, other_req.cells[0].srs_to_meas);

  // The next SRS completes the second request.
  cells[0]->handle_srs_indication(make_srs_ind(rnti, 20.0F));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher2.ready());
  EXPECT_FLOAT_EQ(t_launcher2.result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), 20.0F);
}

TEST_F(single_cell_positioning_handler_test, when_an_indication_reports_one_rnti_twice_then_the_first_report_is_kept)
{
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  const rnti_t                        rnti = req.cells[0].rnti.value();

  auto& t_launcher = launch(req);

  const std::pair<rnti_t, float> reports[] = {{rnti, 10.0F}, {rnti, 20.0F}};
  cells[0]->handle_srs_indication(make_multi_ue_srs_ind(reports));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher.ready());
  EXPECT_FLOAT_EQ(t_launcher.result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), 10.0F);
}

TEST_F(single_cell_positioning_handler_test, when_identical_requests_arrive_then_they_run_one_after_the_other)
{
  static constexpr unsigned nof_reqs = 3;

  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  const rnti_t                        rnti = req.cells[0].rnti.value();
  std::vector<lazy_task_launcher<mac_positioning_measurement_response>*> req_launchers;
  for (unsigned i = 0; i != nof_reqs; ++i) {
    req_launchers.push_back(&launch(req));
  }
  ASSERT_EQ(sched.received_reqs.size(), 1);

  // Each SRS completes the oldest request, and starts the next one.
  for (unsigned i = 0; i != nof_reqs; ++i) {
    const float azimuth_deg = 10.0F * (i + 1);
    cells[0]->handle_srs_indication(make_srs_ind(rnti, azimuth_deg));
    worker.run_pending_tasks();
    for (unsigned j = 0; j != nof_reqs; ++j) {
      ASSERT_EQ(req_launchers[j]->ready(), j <= i);
    }
    EXPECT_FLOAT_EQ(req_launchers[i]->result.value().cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(),
                    azimuth_deg);
    ASSERT_EQ(sched.received_reqs.size(), std::min(i + 2, nof_reqs));
  }
}

TEST_F(single_cell_positioning_handler_test, when_a_request_waits_in_the_queue_then_its_timeout_runs_from_its_arrival)
{
  mac_positioning_measurement_request req1 = make_positioning_request_for_connected_ue();
  mac_positioning_measurement_request req2 = req1;
  const rnti_t                        rnti = req1.cells[0].rnti.value();
  req1.timeout                             = std::chrono::milliseconds{40};
  req2.timeout                             = std::chrono::milliseconds{30};

  auto& t_launcher1 = launch(req1);
  auto& t_launcher2 = launch(req2);

  // The first request completes, and the second one starts.
  tick(10);
  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher1.ready());
  ASSERT_EQ(sched.received_reqs.size(), 2);

  // The second request times out at its timeout from its arrival.
  tick(req2.timeout.count() - 10 - 1);
  ASSERT_FALSE(t_launcher2.ready());
  tick(1);
  ASSERT_TRUE(t_launcher2.ready());
  ASSERT_TRUE(t_launcher2.result.value().cell_results.empty());
  ASSERT_EQ(sched.stop_reqs.size(), 2);
}

TEST_F(single_cell_positioning_handler_test,
       when_a_queued_request_times_out_then_it_leaves_the_queue_without_reaching_the_scheduler)
{
  mac_positioning_measurement_request req1 = make_positioning_request_for_connected_ue();
  mac_positioning_measurement_request req2 = req1;
  const rnti_t                        rnti = req1.cells[0].rnti.value();
  req1.timeout                             = std::chrono::milliseconds{40};
  req2.timeout                             = std::chrono::milliseconds{20};

  auto& t_launcher1 = launch(req1);
  auto& t_launcher2 = launch(req2);

  tick(req2.timeout.count());
  ASSERT_TRUE(t_launcher2.ready());
  ASSERT_TRUE(t_launcher2.result.value().cell_results.empty());
  ASSERT_FALSE(t_launcher1.ready());
  ASSERT_TRUE(sched.stop_reqs.empty()) << "The queued request stopped the running measurement";

  // The first request completes, and no other measurement starts.
  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher1.ready());
  ASSERT_EQ(sched.received_reqs.size(), 1);
}

TEST_F(single_cell_positioning_handler_test, when_concurrent_requests_time_out_then_each_one_follows_its_own_timeout)
{
  mac_positioning_measurement_request req1 = make_positioning_request_for_connected_ue();
  mac_positioning_measurement_request req2 = make_positioning_request_for_second_connected_ue();
  req1.timeout                             = std::chrono::milliseconds{20};
  req2.timeout                             = std::chrono::milliseconds{40};

  auto& t_launcher1 = launch(req1);
  auto& t_launcher2 = launch(req2);

  // Only the first request times out.
  tick(req1.timeout.count());
  ASSERT_TRUE(t_launcher1.ready());
  ASSERT_TRUE(t_launcher1.result.value().cell_results.empty());
  ASSERT_FALSE(t_launcher2.ready());
  ASSERT_EQ(sched.stop_reqs.size(), 1);
  EXPECT_EQ(sched.stop_reqs.back().completed_meas[0].second, req1.cells[0].rnti.value());

  // The second request still completes on the SRS of its UE.
  cells[0]->handle_srs_indication(make_srs_ind(req2.cells[0].rnti.value()));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher2.ready());
  ASSERT_FALSE(t_launcher2.result.value().cell_results.empty());
}

TEST_F(single_cell_positioning_handler_test,
       when_the_maximum_number_of_measurements_is_running_then_a_new_request_is_discarded)
{
  // Measurements of different UEs take every measurement context.
  static constexpr unsigned           max_meass = MAX_NOF_CONCURRENT_POSITIONING_MEASUREMENTS;
  mac_positioning_measurement_request req       = make_positioning_request_for_connected_ue();
  for (unsigned i = 0; i != max_meass; ++i) {
    req.cells[0].rnti     = to_rnti(0x4601 + i);
    req.cells[0].ue_index = to_du_ue_index(i);
    ASSERT_FALSE(launch(req).ready());
  }
  ASSERT_EQ(sched.received_reqs.size(), max_meass);

  // A request beyond the maximum is discarded.
  req.cells[0].rnti          = to_rnti(0x4601 + max_meass);
  req.cells[0].ue_index      = to_du_ue_index(max_meass);
  auto& t_discarded_launcher = launch(req);
  ASSERT_TRUE(t_discarded_launcher.ready());
  ASSERT_TRUE(t_discarded_launcher.result.value().cell_results.empty());
  ASSERT_EQ(sched.received_reqs.size(), max_meass);

  // Once a measurement completes, a new request is accepted.
  cells[0]->handle_srs_indication(make_srs_ind(to_rnti(0x4601)));
  worker.run_pending_tasks();
  ASSERT_TRUE(launchers.front().ready());

  auto& t_launcher = launch(req);
  ASSERT_FALSE(t_launcher.ready());
  ASSERT_EQ(sched.received_reqs.size(), max_meass + 1);
}

class multi_cell_positioning_handler_test : public positioning_handler_test, public ::testing::Test
{
public:
  multi_cell_positioning_handler_test()
  {
    OCUDU_TEST_REQUIREMENTS("MVP-FUNC-POS-16-3-b");

    cells.push_back(pos_handler->add_cell(to_du_cell_index(0)));
    cells.push_back(pos_handler->add_cell(to_du_cell_index(1)));
  }
};

TEST_F(multi_cell_positioning_handler_test,
       when_positioning_measurement_is_initiated_for_multiple_cells_then_it_completes_when_all_cells_report)
{
  mac_positioning_measurement_request req              = make_positioning_request_for_connected_ue(2);
  rnti_t                              pos_rnti1        = req.cells[0].rnti.value();
  rnti_t                              pos_rnti2        = req.cells[1].rnti.value();
  unsigned                            rand_slot_offset = test_rng::uniform_int(0, 100);
  auto&                               t_launcher       = launch(req);

  // Scheduler gets notified for all cells.
  ASSERT_TRUE(sched.last_req.has_value());
  ASSERT_EQ(sched.last_req->cells.size(), req.cells.size());
  ASSERT_EQ(sched.received_reqs.size(), 1);
  for (unsigned i = 0; i != req.cells.size(); ++i) {
    const auto& expected_cell = req.cells[i];
    const auto& notif_cell    = sched.last_req->cells[i];
    EXPECT_EQ(notif_cell.cell_index, expected_cell.cell_index);
    EXPECT_EQ(notif_cell.ue_index, expected_cell.ue_index);
    EXPECT_EQ(notif_cell.pos_rnti, expected_cell.rnti);
    EXPECT_TRUE(is_crnti(notif_cell.pos_rnti));
    EXPECT_EQ(notif_cell.srs_to_measure, expected_cell.srs_to_meas);
  }

  // Procedure has not completed yet.
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher.ready());
  ASSERT_TRUE(sched.stop_reqs.empty());

  // Cell 0 reports.
  mac_srs_indication_message srs_ind = make_srs_ind(pos_rnti1);
  srs_ind.sl_rx += rand_slot_offset;
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher.ready());

  // Cell 1 reports.
  srs_ind = make_srs_ind(pos_rnti2);
  srs_ind.sl_rx += rand_slot_offset;
  cells[1]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();

  // All cells reported.
  ASSERT_TRUE(t_launcher.ready());
  // Scheduler was notified to stop positioning measurements.
  ASSERT_EQ(sched.stop_reqs.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas.size(), 2);
  EXPECT_EQ(sched.stop_reqs.back().completed_meas[0].second, pos_rnti1);
  EXPECT_EQ(sched.stop_reqs.back().completed_meas[1].second, pos_rnti2);
}

TEST_F(
    multi_cell_positioning_handler_test,
    when_positioning_measurement_is_initiated_for_multiple_cells_then_it_does_not_complete_until_cells_reports_match_in_slots)
{
  mac_positioning_measurement_request req              = make_positioning_request_for_connected_ue(2);
  unsigned                            rand_slot_offset = test_rng::uniform_int(1, 100);
  rnti_t                              pos_rnti1        = req.cells[0].rnti.value();
  rnti_t                              pos_rnti2        = req.cells[1].rnti.value();
  auto&                               t_launcher       = launch(req);

  // Scheduler gets notified for all cells.
  ASSERT_TRUE(sched.last_req.has_value());
  ASSERT_EQ(sched.last_req->cells.size(), req.cells.size());
  ASSERT_EQ(sched.received_reqs.size(), 1);
  for (unsigned i = 0; i != req.cells.size(); ++i) {
    const auto& expected_cell = req.cells[i];
    const auto& notif_cell    = sched.last_req->cells[i];
    EXPECT_EQ(notif_cell.cell_index, expected_cell.cell_index);
    EXPECT_EQ(notif_cell.ue_index, expected_cell.ue_index);
    EXPECT_EQ(notif_cell.pos_rnti, expected_cell.rnti);
    EXPECT_TRUE(is_crnti(notif_cell.pos_rnti));
    EXPECT_EQ(notif_cell.srs_to_measure, expected_cell.srs_to_meas);
  }

  // Procedure has not completed yet.
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher.ready());

  // Cell 0 reports.
  mac_srs_indication_message srs_ind = make_srs_ind(pos_rnti1);
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher.ready());

  // Cell 1 reports.
  srs_ind = make_srs_ind(pos_rnti2);
  srs_ind.sl_rx += rand_slot_offset; // different slot.
  cells[1]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();

  // All cells reported, but they don't match yet in slot at which report was produced.
  ASSERT_FALSE(t_launcher.ready());
  ASSERT_TRUE(sched.stop_reqs.empty());

  // Cell 0 reports again, now matching cell 1 slot.
  srs_ind = make_srs_ind(pos_rnti1);
  srs_ind.sl_rx += rand_slot_offset;
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();

  // All cells reported, now matching in slot.
  ASSERT_TRUE(t_launcher.ready());
  // Scheduler was notified to stop positioning measurements.
  ASSERT_EQ(sched.stop_reqs.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas.size(), 2);
  EXPECT_EQ(sched.stop_reqs.back().completed_meas[0].second, pos_rnti1);
  EXPECT_EQ(sched.stop_reqs.back().completed_meas[1].second, pos_rnti2);
}

TEST_F(multi_cell_positioning_handler_test,
       when_a_cell_reports_a_newer_slot_then_the_measurement_matches_the_cells_on_the_newer_slot)
{
  mac_positioning_measurement_request req        = make_positioning_request_for_connected_ue(2);
  auto&                               t_launcher = launch(req);

  // Cell 0 reports two SRS occasions before cell 1 reports the first one.
  mac_srs_indication_message first_ind  = make_srs_ind(req.cells[0].rnti.value());
  mac_srs_indication_message second_ind = make_srs_ind(req.cells[0].rnti.value());
  second_ind.sl_rx += 20;
  cells[0]->handle_srs_indication(first_ind);
  cells[0]->handle_srs_indication(second_ind);
  worker.run_pending_tasks();

  // Only the last report of a cell is kept, so the first occasion of cell 1 does not complete the measurement.
  first_ind.srss[0].rnti = req.cells[1].rnti.value();
  cells[1]->handle_srs_indication(first_ind);
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher.ready());

  // Cell 1 reports the second occasion, which completes the measurement.
  second_ind.srss[0].rnti = req.cells[1].rnti.value();
  cells[1]->handle_srs_indication(second_ind);
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher.ready());
  ASSERT_EQ(t_launcher.result.value().sl_rx, second_ind.sl_rx);
}

TEST_F(multi_cell_positioning_handler_test, when_positioning_measurement_completes_then_resources_are_cleaned_up)
{
  // Launch positioning for 2 cells.
  mac_positioning_measurement_request req        = make_positioning_request_for_connected_ue(2);
  rnti_t                              pos_rnti1  = req.cells[0].rnti.value();
  rnti_t                              pos_rnti2  = req.cells[1].rnti.value();
  auto&                               t_launcher = launch(req);

  // Cell 0 and 1 report and positioning procedure completes.
  mac_srs_indication_message srs_ind = make_srs_ind(pos_rnti1);
  cells[0]->handle_srs_indication(srs_ind);
  srs_ind = make_srs_ind(pos_rnti2);
  cells[1]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher.ready());

  // A new positioning measurement can be initiated right away and so happens to match with previous indication slot.
  // This wrap-around is highly unlikely in practice, but we want to ensure that resources are correctly cleaned up.
  auto& t_launcher2 = launch(req);
  srs_ind           = make_srs_ind(pos_rnti1);
  cells[0]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();
  ASSERT_FALSE(t_launcher2.ready());
  srs_ind = make_srs_ind(pos_rnti2);
  cells[1]->handle_srs_indication(srs_ind);
  worker.run_pending_tasks();
  ASSERT_TRUE(t_launcher2.ready());
}

TEST_F(multi_cell_positioning_handler_test, when_neighbor_ue_request_times_out_then_pos_rntis_are_released)
{
  mac_positioning_measurement_request req;
  req.cells.resize(2);
  for (unsigned i = 0; i != 2; ++i) {
    req.cells[i].cell_index  = to_du_cell_index(i);
    req.cells[i].srs_to_meas = make_test_srs_config();
  }
  req.timeout = std::chrono::milliseconds{20};

  auto& t_launcher = launch(req);
  ASSERT_TRUE(sched.last_req.has_value());
  const rnti_t pos_rnti1 = sched.last_req->cells[0].pos_rnti;
  const rnti_t pos_rnti2 = sched.last_req->cells[1].pos_rnti;
  ASSERT_FALSE(is_crnti(pos_rnti1));

  tick(req.timeout.count());
  ASSERT_TRUE(t_launcher.ready());

  // Over a full cycle of the reserved RNTIs, every request that times out returns its RNTIs, so no request is
  // discarded and the RNTIs of the first request come back.
  bool first_rntis_reused = false;
  for (unsigned i = 0; i != nof_reserved_rntis; ++i) {
    const size_t nof_reqs     = sched.received_reqs.size();
    auto&        t_i_launcher = launch(req);
    ASSERT_EQ(sched.received_reqs.size(), nof_reqs + 1) << "A reserved RNTI was not released";
    first_rntis_reused |=
        sched.last_req->cells[0].pos_rnti == pos_rnti1 and sched.last_req->cells[1].pos_rnti == pos_rnti2;
    tick(req.timeout.count());
    ASSERT_TRUE(t_i_launcher.ready());
  }
  ASSERT_TRUE(first_rntis_reused);
}

TEST_F(multi_cell_positioning_handler_test,
       when_one_cell_has_no_free_reserved_rnti_then_the_request_is_discarded_without_taking_rntis_in_other_cells)
{
  // Take every reserved RNTI of cell 1.
  exhaust_reserved_rntis(to_du_cell_index(1));
  ASSERT_EQ(sched.received_reqs.size(), nof_reserved_rntis);

  // A multi-TRP request for both cells is discarded, as cell 1 has no free reserved RNTI.
  mac_positioning_measurement_request trp_req;
  trp_req.cells.resize(2);
  for (unsigned i = 0; i != 2; ++i) {
    trp_req.cells[i].cell_index  = to_du_cell_index(i);
    trp_req.cells[i].srs_to_meas = make_test_srs_config();
  }
  auto& t_trp_launcher = launch(trp_req);
  ASSERT_TRUE(t_trp_launcher.ready());
  ASSERT_TRUE(t_trp_launcher.result.value().cell_results.empty());
  ASSERT_EQ(sched.received_reqs.size(), nof_reserved_rntis);

  // Cell 0 lost no reserved RNTI: the next request in cell 0 gets the first one.
  launch(make_positioning_request_for_neighbor_ue());
  ASSERT_EQ(sched.received_reqs.size(), nof_reserved_rntis + 1);
  ASSERT_EQ(sched.last_req->cells[0].pos_rnti, rnti_t::MIN_RESERVED_RNTI);
}

TEST_F(single_cell_positioning_handler_test,
       when_the_caller_drops_its_task_then_its_response_does_not_reach_a_later_caller)
{
  // Number of measurements that complete in between, enough for the transaction IDs to wrap around.
  static constexpr unsigned nof_meass = 2 * MAX_NOF_CONCURRENT_POSITIONING_MEASUREMENTS;

  // The caller of UE A drops its task while the measurement of UE A runs.
  mac_positioning_measurement_request req_a  = make_positioning_request_for_connected_ue();
  const rnti_t                        rnti_a = req_a.cells[0].rnti.value();
  {
    auto t = pos_handler->handle_positioning_measurement_request(req_a);
    lazy_task_launcher<mac_positioning_measurement_response> t_launcher(t);
  }

  // Measurements of other UEs complete.
  mac_positioning_measurement_request req = make_positioning_request_for_connected_ue();
  for (unsigned i = 1; i != nof_meass; ++i) {
    req.cells[0].rnti     = to_rnti(0x4700 + i);
    req.cells[0].ue_index = to_du_ue_index(i);
    auto t                = pos_handler->handle_positioning_measurement_request(req);
    lazy_task_launcher<mac_positioning_measurement_response> t_launcher(t);
    cells[0]->handle_srs_indication(make_srs_ind(req.cells[0].rnti.value()));
    worker.run_pending_tasks();
    ASSERT_TRUE(t_launcher.ready());
  }

  // A new caller measures UE B.
  req.cells[0].rnti     = to_rnti(0x4800);
  req.cells[0].ue_index = to_du_ue_index(nof_meass);
  auto& t_b_launcher    = launch(req);

  // The SRS of UE A completes its measurement, whose caller is gone.
  cells[0]->handle_srs_indication(make_srs_ind(rnti_a));
  worker.run_pending_tasks();
  ASSERT_FALSE(t_b_launcher.ready()) << "The response of a dropped caller reached another caller";
}

TEST_F(single_cell_positioning_handler_test,
       when_the_caller_drops_its_task_while_waiting_then_the_measurement_still_ends)
{
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  const rnti_t                        rnti = req.cells[0].rnti.value();
  {
    // The caller starts waiting for the result and then drops its task.
    auto t = pos_handler->handle_positioning_measurement_request(req);
    lazy_task_launcher<mac_positioning_measurement_response> t_launcher(t);
    ASSERT_FALSE(t_launcher.ready());
  }

  // The measurement completes without a caller to resume, and releases its resources.
  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();
  ASSERT_EQ(sched.stop_reqs.size(), 1);

  launch(req);
  ASSERT_EQ(sched.received_reqs.size(), 2) << "The measurement of the dropped task kept its context";
}

TEST_F(single_cell_positioning_handler_test,
       when_the_caller_of_a_queued_request_drops_its_task_then_its_measurement_still_ends)
{
  mac_positioning_measurement_request req  = make_positioning_request_for_connected_ue();
  const rnti_t                        rnti = req.cells[0].rnti.value();

  auto& t1_launcher = launch(req);
  {
    // The caller of the queued request starts waiting for the result and then drops its task.
    auto t2 = pos_handler->handle_positioning_measurement_request(req);
    lazy_task_launcher<mac_positioning_measurement_response> t2_launcher(t2);
    ASSERT_FALSE(t2_launcher.ready());
  }

  // The first request completes, and the queued measurement starts without a caller.
  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();
  ASSERT_TRUE(t1_launcher.ready());
  ASSERT_EQ(sched.received_reqs.size(), 2);

  // The queued measurement completes without a caller to resume, and releases its resources.
  cells[0]->handle_srs_indication(make_srs_ind(rnti));
  worker.run_pending_tasks();
  ASSERT_EQ(sched.stop_reqs.size(), 2);

  launch(req);
  ASSERT_EQ(sched.received_reqs.size(), 3) << "The measurement of the dropped task kept its context";
}

TEST_F(multi_cell_positioning_handler_test, when_a_queued_request_times_out_then_the_requests_queued_behind_it_start)
{
  // UE A is measured in cell 1.
  mac_positioning_measurement_request req_a1 = make_positioning_request_for_connected_ue(2);
  req_a1.cells.erase(req_a1.cells.begin());
  launch(req_a1);

  // A request for UE A in both cells waits for the measurement in cell 1.
  mac_positioning_measurement_request req_both = make_positioning_request_for_connected_ue(2);
  req_both.timeout                             = std::chrono::milliseconds{20};
  auto& t_both_launcher                        = launch(req_both);

  // A request for UE A in cell 0 waits for the earlier queued one.
  mac_positioning_measurement_request req_a0 = make_positioning_request_for_connected_ue(1);
  launch(req_a0);
  ASSERT_EQ(sched.received_reqs.size(), 1);

  // The request for both cells times out, so the request for cell 0 waits for nothing and starts.
  tick(req_both.timeout.count());
  ASSERT_TRUE(t_both_launcher.ready());
  ASSERT_EQ(sched.received_reqs.size(), 2) << "A queued request did not start once nothing held it back";
  ASSERT_EQ(sched.last_req->cells[0].cell_index, to_du_cell_index(0));
}

TEST_F(multi_cell_positioning_handler_test,
       when_a_queued_request_has_no_free_reserved_rnti_when_it_can_start_then_it_is_discarded)
{
  // UE A is measured in cell 0.
  mac_positioning_measurement_request req_a        = make_positioning_request_for_connected_ue(1);
  auto&                               t_a_launcher = launch(req_a);

  // A multi-TRP request measures UE A in cell 0 and as a neighbor UE in cell 1. It waits for the measurement in cell 0.
  mac_positioning_measurement_request trp_req = req_a;
  trp_req.cells.emplace_back();
  trp_req.cells[1].cell_index  = to_du_cell_index(1);
  trp_req.cells[1].srs_to_meas = make_test_srs_config();
  auto& t_trp_launcher         = launch(trp_req);

  // Measurements of neighbor UEs take every reserved RNTI of cell 1.
  exhaust_reserved_rntis(to_du_cell_index(1));
  ASSERT_EQ(sched.received_reqs.size(), nof_reserved_rntis + 1);

  // The measurement in cell 0 completes. The multi-TRP request can start, but cell 1 has no free reserved RNTI.
  cells[0]->handle_srs_indication(make_srs_ind(req_a.cells[0].rnti.value()));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_a_launcher.ready());
  ASSERT_TRUE(t_trp_launcher.ready()) << "A queued request without a free reserved RNTI kept waiting";
  ASSERT_TRUE(t_trp_launcher.result.value().cell_results.empty());
  ASSERT_EQ(sched.received_reqs.size(), nof_reserved_rntis + 1);
}

TEST_F(multi_cell_positioning_handler_test, when_a_cell_is_removed_then_its_measurements_end_with_an_empty_response)
{
  // One measurement covers both cells and another one only cell 0.
  mac_positioning_measurement_request both_req         = make_positioning_request_for_connected_ue(2);
  auto&                               t_both_launcher  = launch(both_req);
  mac_positioning_measurement_request cell0_req        = make_positioning_request_for_neighbor_ue();
  auto&                               t_cell0_launcher = launch(cell0_req);
  ASSERT_EQ(sched.received_reqs.size(), 2);

  // The scheduler removes the cell before the MAC, so the MAC sends no stop for it.
  cells[1].reset();
  ASSERT_TRUE(t_both_launcher.ready());
  ASSERT_TRUE(t_both_launcher.result.value().cell_results.empty());
  ASSERT_EQ(sched.stop_reqs.size(), 1);
  ASSERT_EQ(sched.stop_reqs.back().completed_meas.size(), 1);
  EXPECT_EQ(sched.stop_reqs.back().completed_meas[0].first, to_du_cell_index(0));

  // The measurement of the remaining cell is not affected.
  ASSERT_FALSE(t_cell0_launcher.ready());
  cells[0]->handle_srs_indication(make_srs_ind(sched.last_req->cells[0].pos_rnti));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_cell0_launcher.ready());
  ASSERT_FALSE(t_cell0_launcher.result.value().cell_results.empty());
}

TEST_F(multi_cell_positioning_handler_test,
       when_a_cell_is_removed_then_its_queued_measurements_end_without_reaching_the_scheduler)
{
  // A second measurement of the same UE waits for the first one.
  mac_positioning_measurement_request req         = make_positioning_request_for_connected_ue(2);
  auto&                               t1_launcher = launch(req);
  auto&                               t2_launcher = launch(req);
  ASSERT_EQ(sched.received_reqs.size(), 1);

  // Both measurements end, and the queued one does not start.
  cells[1].reset();
  ASSERT_TRUE(t1_launcher.ready());
  ASSERT_TRUE(t1_launcher.result.value().cell_results.empty());
  ASSERT_TRUE(t2_launcher.ready());
  ASSERT_TRUE(t2_launcher.result.value().cell_results.empty());
  ASSERT_EQ(sched.received_reqs.size(), 1);
}

TEST_F(multi_cell_positioning_handler_test,
       when_a_neighbor_ue_and_a_connected_ue_are_measured_concurrently_then_both_complete)
{
  // Multi-TRP measurement of a neighbor UE in both cells.
  mac_positioning_measurement_request trp_req;
  trp_req.cells.resize(2);
  for (unsigned i = 0; i != 2; ++i) {
    trp_req.cells[i].cell_index  = to_du_cell_index(i);
    trp_req.cells[i].srs_to_meas = make_test_srs_config();
  }
  auto&        t_trp_launcher = launch(trp_req);
  const rnti_t pos_rnti_cell0 = sched.last_req->cells[0].pos_rnti;
  const rnti_t pos_rnti_cell1 = sched.last_req->cells[1].pos_rnti;

  // Measurement of a UE connected to cell 0.
  mac_positioning_measurement_request ue_req        = make_positioning_request_for_connected_ue(1);
  const rnti_t                        crnti         = ue_req.cells[0].rnti.value();
  auto&                               t_ue_launcher = launch(ue_req);
  ASSERT_EQ(sched.received_reqs.size(), 2);

  // Cell 0 reports both SRSs in the same indication. Only the single-cell measurement is complete.
  const std::pair<rnti_t, float> cell0_reports[] = {{pos_rnti_cell0, 10.0F}, {crnti, 20.0F}};
  cells[0]->handle_srs_indication(make_multi_ue_srs_ind(cell0_reports));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_ue_launcher.ready());
  ASSERT_FALSE(t_trp_launcher.ready());

  // Cell 1 reports the neighbor UE in the same slot, which completes the multi-TRP measurement.
  const std::pair<rnti_t, float> cell1_reports[] = {{pos_rnti_cell1, 30.0F}};
  cells[1]->handle_srs_indication(make_multi_ue_srs_ind(cell1_reports));
  worker.run_pending_tasks();
  ASSERT_TRUE(t_trp_launcher.ready());
  const auto& trp_resp = t_trp_launcher.result.value();
  ASSERT_EQ(trp_resp.cell_results.size(), 2);
  EXPECT_FLOAT_EQ(trp_resp.cell_results[0].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), 10.0F);
  EXPECT_FLOAT_EQ(trp_resp.cell_results[1].ul_srs_pos_meass[0].azimuth_aoa_deg.value(), 30.0F);
}
