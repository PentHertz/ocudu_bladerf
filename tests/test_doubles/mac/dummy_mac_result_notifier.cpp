// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "dummy_mac_result_notifier.h"
#include "ocudu/support/executors/task_executor.h"

using namespace ocudu;

phy_cell_test_dummy::phy_cell_test_dummy(task_executor& exec_) : test_exec(exec_) {}

void phy_cell_test_dummy::on_new_downlink_scheduler_results(const mac_dl_sched_result& dl_res)
{
  cached_dl_res = dl_res;
}

void phy_cell_test_dummy::on_new_downlink_data(const mac_dl_data_result& dl_data)
{
  cached_dl_data = dl_data;
}

void phy_cell_test_dummy::on_new_uplink_scheduler_results(const mac_ul_sched_result& ul_res)
{
  cached_ul_res = ul_res;
}

void phy_cell_test_dummy::on_cell_results_completion(slot_point slot)
{
  // The result lives in the slot of the scheduler grid, so it is copied out before the slot is reused.
  dl_sched_result_copy dl_sched_res;
  if (cached_dl_res.has_value()) {
    copy_result(cached_dl_res->dl_res, dl_sched_res);
  }
  ul_sched_result_copy ul_sched_res;
  if (cached_ul_res.has_value()) {
    copy_result(cached_ul_res->ul_res, ul_sched_res);
  }

  bool result = test_exec.execute([this,
                                   slot,
                                   dl_sched_res = std::move(dl_sched_res),
                                   ul_sched_res = std::move(ul_sched_res),
                                   dl_res_copy  = cached_dl_res,
                                   dl_data_copy = cached_dl_data,
                                   ul_res_copy  = cached_ul_res]() mutable {
    last_slot_res = slot;
    last_dl_res   = dl_res_copy;
    last_dl_data  = dl_data_copy;
    last_ul_res   = ul_res_copy;

    results->reset_slot(0, dl_sched_res.nof_dl_symbols, ul_sched_res.nof_ul_symbols);
    if (last_dl_res.has_value()) {
      copy_result(dl_sched_res, results->get_builder(0).dl());
      last_dl_res->dl_res = results->get_reader(0).dl();
    }
    if (last_ul_res.has_value()) {
      copy_result(ul_sched_res, results->get_builder(0).ul());
      last_ul_res->ul_res = results->get_reader(0).ul();
    }
  });
  report_fatal_error_if_not(result, "Failed to execute task");
  cached_dl_res  = {};
  cached_dl_data = {};
  cached_ul_res  = {};
}
