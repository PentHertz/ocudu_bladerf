// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "test_utils/scheduler_test_simulator.h"
#include "tests/test_doubles/scheduler/scheduler_config_helper.h"
#include "ocudu/scheduler/scheduler_positioning_handler.h"
#include <gtest/gtest.h>
#include <set>

using namespace ocudu;

class scheduler_positioning_test : public ::testing::Test
{
protected:
  scheduler_positioning_test() { bench.add_cell(sched_config_helper::make_default_sched_cell_configuration_request()); }

  /// Builds the SRS configuration of a UE of another cell, with a periodic SRS at the given offset.
  srs_config make_neighbor_srs_config(srs_periodicity period, unsigned offset) const
  {
    srs_config cfg = bench.cfg_mng.get_default_ue_config_request()
                         .cfg.cells.value()
                         .front()
                         .ded->serv_cell_cfg.ul_config.value()
                         .init_ul_bwp.srs_cfg.value();
    cfg.srs_res_set_list.front().res_type.emplace<srs_config::srs_resource_set::periodic_resource_type>();
    srs_config::srs_resource& res = cfg.srs_res_list.front();
    res.res_type                  = srs_resource_type::periodic;
    res.periodicity_and_offset.emplace(srs_config::srs_periodicity_and_offset{period, static_cast<uint16_t>(offset)});
    return cfg;
  }

  scheduler_test_simulator bench;
};

TEST_F(scheduler_positioning_test, when_more_than_four_measurements_target_one_cell_then_all_of_them_are_scheduled)
{
  // More requests than the four that the scheduler accepted in a cell before concurrent measurements.
  static constexpr unsigned        nof_meas = 6;
  static constexpr srs_periodicity period   = srs_periodicity::sl10;
  std::set<rnti_t>                 expected_rntis;
  for (unsigned i = 0; i != nof_meas; ++i) {
    const rnti_t                    pos_rnti = to_rnti(static_cast<unsigned>(rnti_t::MIN_RESERVED_RNTI) + i);
    positioning_measurement_request req;
    req.cells.push_back(positioning_measurement_request::cell_info{
        pos_rnti, std::nullopt, to_du_cell_index(0), make_neighbor_srs_config(period, i)});
    bench.sched->handle_positioning_measurement_request(req);
    expected_rntis.insert(pos_rnti);
  }

  // Every measurement gets SRS PDUs that request a positioning report.
  std::set<rnti_t> scheduled_rntis;
  for (unsigned n = 0; n != 4 * static_cast<unsigned>(period); ++n) {
    bench.run_slot();
    for (const auto& srs : bench.last_sched_result()->ul.srss) {
      if (srs.positioning_report_requested) {
        scheduled_rntis.insert(srs.crnti);
      }
    }
  }
  ASSERT_EQ(scheduled_rntis, expected_rntis);
}
