// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "positioning_handler.h"
#include "../mac_ctrl/spsc_metric_report_channel.h"
#include "ocudu/adt/format.h"
#include "ocudu/adt/ring_buffer.h"
#include "ocudu/adt/slotted_array.h"
#include "ocudu/adt/span.h"
#include "ocudu/adt/static_vector.h"
#include "ocudu/mac/mac_positioning_measurement_handler.h"
#include "ocudu/ocudulog/ocudulog.h"
#include "ocudu/scheduler/scheduler_positioning_handler.h"
#include "ocudu/support/async/async_no_op_task.h"
#include "ocudu/support/async/protocol_transaction_manager.h"
#include "ocudu/support/executors/execute_until_success.h"
#include "ocudu/support/executors/task_executor.h"
#include "ocudu/support/timers.h"
#include <chrono>
#include <optional>

using namespace ocudu;

namespace {

/// \brief [Implementation-defined] Time during which a released reserved POS-RNTI cannot be allocated again.
///
/// After a measurement stops, the SRS PDUs of its RNTI stay in the scheduler grid for up to the maximum UL slot
/// allocation delay, and the PHY reports them up to 80 slots later. A new measurement with the same RNTI would flag
/// these PDUs or receive their reports, so the value covers both delays at 15 kHz, about 112 slots. With 14 reserved
/// RNTIs per cell, a cell starts at most 14 measurements of neighbor UEs every 128 ms.
/// TODO: Cover the larger UL slot allocation delay of NTN cells.
constexpr std::chrono::milliseconds POS_RNTI_QUARANTINE{128};

/// MAC scheduler positioning handler.
class pos_handler_impl final : public positioning_handler
{
public:
  class cell_positioning_handler_impl final : public cell_positioning_handler
  {
  public:
    cell_positioning_handler_impl(du_cell_index_t cell_index_, pos_handler_impl& parent_) :
      cell_index(cell_index_), parent(parent_)
    {
    }
    ~cell_positioning_handler_impl() override { parent.rem_cell(cell_index); }

    void handle_srs_indication(const mac_srs_indication_message& msg) override
    {
      parent.handle_srs_indication(cell_index, msg);
    }

  private:
    du_cell_index_t   cell_index;
    pos_handler_impl& parent;
  };

  pos_handler_impl(scheduler_positioning_handler& sched_,
                   task_executor&                 ctrl_exec_,
                   timer_manager&                 timers_,
                   ocudulog::basic_logger&        logger_) :
    sched(sched_),
    ctrl_exec(ctrl_exec_),
    timers(timers_),
    logger(logger_),
    meass(MAX_NOF_CONCURRENT_POSITIONING_MEASUREMENTS),
    transactions(MAX_NOF_CONCURRENT_POSITIONING_MEASUREMENTS, timer_factory{timers_, ctrl_exec_})
  {
    // Destroying a timer from its own callback is unsafe, so the timers are created once.
    timer_factory timer_fact{timers_, ctrl_exec_};
    for (unsigned i = 0; i != meass.size(); ++i) {
      meass[i].timeout_timer = timer_fact.create_timer();
      meass[i].timeout_timer.set(std::chrono::milliseconds{0}, [this, i]() { handle_meas_timeout(i); });
    }
  }

  async_task<mac_positioning_measurement_response>
  handle_positioning_measurement_request(const mac_positioning_measurement_request& req) override;

  std::unique_ptr<cell_positioning_handler> add_cell(du_cell_index_t cell_index) override
  {
    ocudu_assert(not cells.contains(cell_index), "Cell already exists");
    cells.emplace(cell_index, *this, cell_index);
    return std::make_unique<cell_positioning_handler_impl>(cell_index, *this);
  }

private:
  using report_queue_type = spsc_metric_report_channel<mac_srs_indication_message>;

  /// Number of SRS indications that a cell can pass to the ctrl execution context before they are handled.
  static constexpr size_t MAX_PENDING_REPORTS_PER_CELL = 4;

  /// Number of reserved RNTIs that a cell uses for positioning measurements of UEs that are not connected to it.
  static constexpr size_t NOF_DUMMY_RNTIS =
      static_cast<size_t>(rnti_t::MAX_RESERVED_RNTI) - static_cast<size_t>(rnti_t::MIN_RESERVED_RNTI) + 1;

  /// Reserved RNTI that is free for positioning measurements of UEs that are not connected to the cell.
  struct free_dummy_rnti {
    rnti_t rnti;
    /// Time at which the RNTI was released. Empty if the RNTI was never used.
    std::optional<tick_point_t> released_at;
  };

  struct cell_meas_context {
    const du_cell_index_t cell_index;
    /// Channel used to transfer reports from lower layer execution context to ctrl execution context.
    report_queue_type channel;
    /// List of free RNTIs that can be used for positioning measurement for UEs that are not connected to the cell. This
    /// list is only accessed by the CTRL executor context. The RNTIs are reused in release order.
    ring_buffer<free_dummy_rnti> free_dummy_rnti_list;

    cell_meas_context(pos_handler_impl& parent, du_cell_index_t cell_index_);
  };

  /// Positioning report of one cell, received in a given slot.
  struct cell_report {
    slot_point                      sl_rx;
    mac_srs_pdu::positioning_report report;
  };

  /// Cell measured by a positioning measurement.
  struct meas_cell {
    du_cell_index_t cell_index;
    rnti_t          pos_rnti;
    /// Last report of this cell. The measurement completes when all its cells reported the same slot.
    std::optional<cell_report> last_report;
  };

  using meas_transaction = protocol_transaction<mac_positioning_measurement_response>;

  /// State of a positioning measurement context.
  enum class meas_state { idle, queued, running };

  /// Context of a positioning measurement.
  struct meas_context {
    meas_state state = meas_state::idle;
    /// Request that waits for a measurement of the same C-RNTI to complete.
    mac_positioning_measurement_request queued_req;
    /// Cells of the running measurement.
    static_vector<meas_cell, MAX_NOF_DU_CELLS> cells;
    /// Timer that bounds the wait for the SRS measurements, including the time in the queue.
    unique_timer timeout_timer;
  };

  void handle_srs_indication(du_cell_index_t cell_index, const mac_srs_indication_message& msg);

  void handle_pending_srs_reports();

  void handle_pending_srs_report(du_cell_index_t cell_index, const mac_srs_indication_message& msg);

  void handle_meas_report(meas_context&                          meas,
                          du_cell_index_t                        cell_index,
                          slot_point                             sl_rx,
                          const mac_srs_pdu::positioning_report& report);

  void handle_meas_timeout(unsigned meas_idx);

  void complete_meas(meas_context& meas, mac_positioning_measurement_response resp);

  /// Asserts that the cells of a request exist and appear only once.
  void assert_cell_list_valid(const mac_positioning_measurement_request& req) const;

  /// Checks whether a running measurement or one of the given queued ones measures a C-RNTI of the request in its cell.
  bool is_crnti_measured(const mac_positioning_measurement_request& req, span<const unsigned> queued);

  /// Checks whether each cell of the request that measures a neighbor UE has a free reserved RNTI.
  bool has_free_dummy_rntis(const mac_positioning_measurement_request& req) const;

  /// Allocates the POS-RNTIs and starts the measurement in the scheduler.
  void start_meas(meas_context& meas, const mac_positioning_measurement_request& req);

  /// Starts the queued measurements that no longer wait, in arrival order.
  void start_queued_meass();

  /// Returns a task that waits for the response delivered through the given transaction.
  static async_task<mac_positioning_measurement_response> wait_for_result(meas_transaction transaction);

  /// Releases the POS-RNTIs and frees the context.
  void release_meas_resources(meas_context& meas);

  /// Checks whether a free reserved RNTI is past its quarantine.
  bool is_dummy_rnti_available(const free_dummy_rnti& entry) const;

  meas_context* find_free_meas();

  meas_context* find_meas(du_cell_index_t cell_index, rnti_t rnti);

  void rem_cell(du_cell_index_t cell_index);

  scheduler_positioning_handler& sched;
  task_executor&                 ctrl_exec;
  timer_manager&                 timers;
  ocudulog::basic_logger&        logger;

  slotted_id_table<du_cell_index_t, cell_meas_context, MAX_NOF_DU_CELLS> cells;

  std::atomic<unsigned> job_count{0};

  /// Pool of positioning measurement contexts.
  std::vector<meas_context> meass;

  /// Indexes of the queued measurements, in arrival order.
  static_vector<unsigned, MAX_NOF_CONCURRENT_POSITIONING_MEASUREMENTS> queued_meass;

  /// Transactions that deliver the responses to the callers.
  protocol_transaction_manager<mac_positioning_measurement_response> transactions;
};

/// Checks whether an SRS PDU carries a positioning report with at least one supported measurement type.
bool has_supported_positioning_report(const mac_srs_pdu& pdu)
{
  const auto* pos_rep = std::get_if<mac_srs_pdu::positioning_report>(&pdu.report);
  return pos_rep != nullptr and (pos_rep->ul_rtoa.has_value() or pos_rep->azimuth_aoa.has_value());
}

pos_handler_impl::cell_meas_context::cell_meas_context(pos_handler_impl& parent, du_cell_index_t cell_index_) :
  cell_index(cell_index_), channel(MAX_PENDING_REPORTS_PER_CELL, parent.logger), free_dummy_rnti_list(NOF_DUMMY_RNTIS)
{
  // Fill free list of dummy RNTIs.
  for (rnti_t reserved_rnti = rnti_t::MIN_RESERVED_RNTI; reserved_rnti <= rnti_t::MAX_RESERVED_RNTI;
       reserved_rnti        = to_rnti((unsigned)reserved_rnti + 1U)) {
    free_dummy_rnti_list.push(free_dummy_rnti{reserved_rnti, std::nullopt});
  }
}

async_task<mac_positioning_measurement_response>
pos_handler_impl::handle_positioning_measurement_request(const mac_positioning_measurement_request& req)
{
  assert_cell_list_valid(req);

  meas_context* meas = find_free_meas();
  if (meas == nullptr) {
    logger.warning("Discarding positioning measurement request. Cause: {} measurements are already running or queued",
                   meass.size());
    return launch_no_op_task(mac_positioning_measurement_response{});
  }

  // Check every cell before taking any resource, so that a discarded request leaves no trace. The scheduler runs one
  // measurement per RNTI and cell, so a request for a C-RNTI that is already measured in the same cell waits until that
  // measurement completes.
  const bool queue_req = is_crnti_measured(req, queued_meass);
  if (not queue_req and not has_free_dummy_rntis(req)) {
    return launch_no_op_task(mac_positioning_measurement_response{});
  }

  // The transaction belongs to the caller task, so the response of a caller that is gone is dropped. Its ID is the
  // index of the context, so that the ID does not go to another caller while the measurement runs.
  const unsigned   meas_idx    = static_cast<unsigned>(meas - meass.data());
  meas_transaction transaction = transactions.create_transaction(meas_idx);
  if (not transaction.valid()) {
    logger.warning("Discarding positioning measurement request. Cause: The caller of the previous measurement of the "
                   "context still holds its transaction");
    return launch_no_op_task(mac_positioning_measurement_response{});
  }

  // Start the guard timer. If no SRS measurement arrives in time, complete the request with an empty response.
  meas->timeout_timer.set(req.timeout);
  meas->timeout_timer.run();

  if (queue_req) {
    meas->state      = meas_state::queued;
    meas->queued_req = req;
    queued_meass.push_back(meas_idx);
  } else {
    start_meas(*meas, req);
  }

  return wait_for_result(std::move(transaction));
}

void pos_handler_impl::start_meas(meas_context& meas, const mac_positioning_measurement_request& req)
{
  // Allocate a POS-RNTI in each cell.
  for (const auto& cell_req : req.cells) {
    rnti_t pos_rnti = rnti_t::INVALID_RNTI;
    if (cell_req.rnti.has_value()) {
      // It is a UE that belongs to the cell.
      pos_rnti = *cell_req.rnti;
    } else {
      // It is a measurement for a UE that does not belong to this cell. Use reserved RNTIs for this situation.
      auto& free_list = cells[cell_req.cell_index].free_dummy_rnti_list;
      pos_rnti        = free_list.top().rnti;
      free_list.pop();
    }
    meas.cells.push_back(meas_cell{cell_req.cell_index, pos_rnti, {}});
  }
  meas.state = meas_state::running;

  // Command scheduler to start reporting positioning measurements.
  positioning_measurement_request sched_req;
  for (unsigned i = 0, e = req.cells.size(); i != e; ++i) {
    auto& sched_cell          = sched_req.cells.emplace_back();
    sched_cell.cell_index     = req.cells[i].cell_index;
    sched_cell.ue_index       = req.cells[i].ue_index;
    sched_cell.pos_rnti       = meas.cells[i].pos_rnti;
    sched_cell.srs_to_measure = req.cells[i].srs_to_meas;
  }
  sched.handle_positioning_measurement_request(sched_req);
}

void pos_handler_impl::start_queued_meass()
{
  for (unsigned i = 0; i != queued_meass.size();) {
    meas_context& meas = meass[queued_meass[i]];
    if (std::any_of(meas.queued_req.cells.begin(), meas.queued_req.cells.end(), [this](const auto& c) {
          return not cells.contains(c.cell_index);
        })) {
      // A request for a removed cell is discarded. Its completion processes the rest of the queue.
      logger.warning("Discarding queued positioning measurement request. Cause: A cell of the request was removed");
      complete_meas(meas, {});
      return;
    }
    // A measurement also waits for the earlier queued ones of the same C-RNTI.
    if (is_crnti_measured(meas.queued_req, span<const unsigned>(queued_meass.data(), i))) {
      ++i;
      continue;
    }
    if (not has_free_dummy_rntis(meas.queued_req)) {
      // As on arrival, a request without a free reserved RNTI is discarded. Its completion processes the rest of the
      // queue, which its caller may change when it resumes.
      complete_meas(meas, {});
      return;
    }
    queued_meass.erase(queued_meass.begin() + i);
    start_meas(meas, meas.queued_req);
  }
}

async_task<mac_positioning_measurement_response> pos_handler_impl::wait_for_result(meas_transaction transaction)
{
  return launch_async([transaction = std::move(transaction)](
                          coro_context<async_task<mac_positioning_measurement_response>>& ctx) mutable {
    CORO_BEGIN(ctx);

    // Await for the completion of a positioning measurement report.
    CORO_AWAIT(transaction);

    // Return the measurement report. The transaction has no response only when the MAC is destroyed.
    CORO_RETURN(transaction.has_response() ? transaction.response() : mac_positioning_measurement_response{});
  });
}

void pos_handler_impl::handle_srs_indication(du_cell_index_t cell_index, const mac_srs_indication_message& msg)
{
  // This runs in the lower layer execution context, which cannot read the running measurements. The ctrl execution
  // context matches the positioning reports with the measurements, and drops the stray ones.
  bool has_pos_report = false;
  for (const auto& pdu : msg.srss) {
    if (has_supported_positioning_report(pdu)) {
      has_pos_report = true;
    } else if (std::holds_alternative<mac_srs_pdu::positioning_report>(pdu.report)) {
      logger.warning("rnti={} processing positioning SRS indication with no RTOA nor AoA", pdu.rnti);
    }
  }
  if (not has_pos_report) {
    // No positioning PDUs in this indication.
    return;
  }

  // Copy the SRS PDUs for positioning to the report channel.
  auto rep = cells[cell_index].channel.get_builder();
  rep->srss.clear();
  rep->sl_rx = msg.sl_rx;
  for (const auto& pdu : msg.srss) {
    if (has_supported_positioning_report(pdu)) {
      rep->srss.push_back(pdu);
    }
  }
  // Resetting pushes the report to the channel.
  rep.reset();

  // Schedule the processing of pending SRS reports in the ctrl executor, if not already scheduled.
  bool token_acquired = job_count.fetch_add(1, std::memory_order_acq_rel) == 0;
  if (not token_acquired) {
    return;
  }
  defer_until_success(ctrl_exec, timers, [this]() { handle_pending_srs_reports(); });
}

void pos_handler_impl::handle_pending_srs_reports()
{
  auto count = job_count.load(std::memory_order_acquire);
  while (count != 0) {
    bool handled_any = false;
    for (auto& c : cells) {
      auto rep = c.channel.pop();
      if (rep) {
        // Handle the popped SRS report.
        handle_pending_srs_report(c.cell_index, *rep);
        handled_any = true;
        // Decrement the number of pending jobs to handle.
        count = job_count.fetch_sub(1, std::memory_order_acq_rel) - 1;
        if (count == 0) {
          // No more jobs to handle.
          return;
        }
      }
    }
    if (not handled_any) {
      // Unexpected situation: job count is non-zero, but no reports to handle.
      logger.error("Positioning handler job count is inconsistent with pending reports");
      job_count.fetch_sub(count, std::memory_order_acq_rel);
      return;
    }
  }
}

void pos_handler_impl::handle_pending_srs_report(du_cell_index_t cell_index, const mac_srs_indication_message& msg)
{
  for (const auto& pdu : msg.srss) {
    // Note: A late report of a finished measurement completes a new measurement of the same C-RNTI. The report comes
    // from the same UE, a few slots before the new request, so it is accepted. Reserved RNTIs avoid this case through
    // their quarantine, as they move between UEs.
    meas_context* meas = find_meas(cell_index, pdu.rnti);
    if (meas == nullptr) {
      // The measurement of this RNTI already completed or timed out. This is a stray SRS indication.
      continue;
    }
    handle_meas_report(*meas, cell_index, msg.sl_rx, std::get<mac_srs_pdu::positioning_report>(pdu.report));
  }
}

void pos_handler_impl::handle_meas_report(meas_context&                          meas,
                                          du_cell_index_t                        cell_index,
                                          slot_point                             sl_rx,
                                          const mac_srs_pdu::positioning_report& report)
{
  // Store the last report of the cell.
  auto cell_it = std::find_if(
      meas.cells.begin(), meas.cells.end(), [cell_index](const meas_cell& c) { return c.cell_index == cell_index; });
  if (cell_it->last_report.has_value() and cell_it->last_report->sl_rx == sl_rx) {
    // The cell already reported this RNTI in this slot, so the first report is kept.
    return;
  }
  cell_it->last_report = cell_report{sl_rx, report};

  // Check if all cells have reported for this slot.
  for (const auto& c : meas.cells) {
    if (not c.last_report.has_value() or c.last_report->sl_rx != sl_rx) {
      // Report from this cell is missing.
      return;
    }
  }

  // All cells have reported for this slot. Generate response.
  mac_positioning_measurement_response resp;
  resp.sl_rx = sl_rx;
  resp.cell_results.resize(meas.cells.size());
  for (unsigned i = 0, e = meas.cells.size(); i != e; ++i) {
    const auto& rep     = meas.cells[i].last_report->report;
    auto&       out     = resp.cell_results[i].ul_srs_pos_meass.emplace_back();
    out.ul_rtoa         = rep.ul_rtoa;
    out.rsrp_dbfs       = rep.ul_rsrp_dBFS;
    out.azimuth_aoa_deg = rep.azimuth_aoa;
    out.zenith_aoa_deg  = rep.zenith_aoa;
  }

  complete_meas(meas, std::move(resp));
}

void pos_handler_impl::handle_meas_timeout(unsigned meas_idx)
{
  meas_context& meas = meass[meas_idx];
  if (meas.state == meas_state::idle) {
    return;
  }
  logger.warning("Positioning measurement request timed out after {}ms", meas.timeout_timer.duration().count());
  complete_meas(meas, {});
}

void pos_handler_impl::complete_meas(meas_context& meas, mac_positioning_measurement_response resp)
{
  meas.timeout_timer.stop();

  const unsigned meas_idx = static_cast<unsigned>(&meas - meass.data());
  if (meas.state == meas_state::running) {
    // Notify scheduler to stop scheduling new positioning measurement PDUs. A removed cell is skipped, as the scheduler
    // removes its cells before the MAC does.
    positioning_measurement_stop_request stop_req;
    for (const auto& c : meas.cells) {
      if (cells.contains(c.cell_index)) {
        stop_req.completed_meas.emplace_back(c.cell_index, c.pos_rnti);
      }
    }
    if (not stop_req.completed_meas.empty()) {
      sched.handle_positioning_measurement_stop(stop_req);
    }
  } else {
    queued_meass.erase(std::find(queued_meass.begin(), queued_meass.end(), meas_idx));
  }

  // Free the context before notifying the caller, as the caller may resume here and send a new request.
  release_meas_resources(meas);

  // The queued measurements that waited for this one start before the caller resumes, so that a new request of the
  // caller waits behind them.
  start_queued_meass();

  // Notify the completion of the positioning report.
  if (not transactions.set_response(meas_idx, std::move(resp))) {
    logger.debug("Positioning measurement completed, but its caller is gone");
  }
}

void pos_handler_impl::assert_cell_list_valid(const mac_positioning_measurement_request& req) const
{
  // The DU manager maps the TRPs of the request to the cells, and checks them.
  for (auto it = req.cells.begin(); it != req.cells.end(); ++it) {
    ocudu_assert(cells.contains(it->cell_index),
                 "cell={}: Positioning measurement for a cell that does not exist",
                 it->cell_index);
    ocudu_assert(
        std::none_of(it + 1, req.cells.end(), [it](const auto& other) { return other.cell_index == it->cell_index; }),
        "cell={}: Positioning measurement lists the cell more than once",
        it->cell_index);
  }
}

bool pos_handler_impl::is_crnti_measured(const mac_positioning_measurement_request& req, span<const unsigned> queued)
{
  for (const auto& cell_req : req.cells) {
    if (not cell_req.rnti.has_value()) {
      continue;
    }
    if (find_meas(cell_req.cell_index, *cell_req.rnti) != nullptr) {
      return true;
    }
    for (unsigned meas_idx : queued) {
      const auto& queued_cells = meass[meas_idx].queued_req.cells;
      if (std::any_of(queued_cells.begin(), queued_cells.end(), [&cell_req](const auto& c) {
            return c.cell_index == cell_req.cell_index and c.rnti == cell_req.rnti;
          })) {
        return true;
      }
    }
  }
  return false;
}

bool pos_handler_impl::has_free_dummy_rntis(const mac_positioning_measurement_request& req) const
{
  for (const auto& cell_req : req.cells) {
    if (cell_req.rnti.has_value()) {
      continue;
    }
    // The oldest released RNTI comes first, so if it is still in quarantine, all the others are too.
    const auto& free_list = cells[cell_req.cell_index].free_dummy_rnti_list;
    if (free_list.empty() or not is_dummy_rnti_available(free_list.top())) {
      logger.warning("cell={}: Received positioning measurement request for a neighbor UE, but no more reserved RNTIs"
                     " are available in the cell",
                     cell_req.cell_index);
      return false;
    }
  }
  return true;
}

void pos_handler_impl::release_meas_resources(meas_context& meas)
{
  for (const auto& c : meas.cells) {
    if (cells.contains(c.cell_index) and c.pos_rnti >= rnti_t::MIN_RESERVED_RNTI and
        c.pos_rnti <= rnti_t::MAX_RESERVED_RNTI) {
      // Late reports of this RNTI can still arrive, so the RNTI waits in quarantine at the end of the list.
      cells[c.cell_index].free_dummy_rnti_list.push(free_dummy_rnti{c.pos_rnti, timers.now()});
    }
  }
  meas.cells.clear();
  meas.queued_req = {};
  meas.state      = meas_state::idle;
}

bool pos_handler_impl::is_dummy_rnti_available(const free_dummy_rnti& entry) const
{
  if (not entry.released_at.has_value()) {
    return true;
  }
  // The tick difference stays correct across the wrap-around of the tick counter.
  const tick_point_t elapsed = timers.now() - *entry.released_at;
  return elapsed >= static_cast<tick_point_t>(POS_RNTI_QUARANTINE.count());
}

pos_handler_impl::meas_context* pos_handler_impl::find_free_meas()
{
  auto it = std::find_if(meass.begin(), meass.end(), [](const meas_context& m) { return m.state == meas_state::idle; });
  return it != meass.end() ? &*it : nullptr;
}

pos_handler_impl::meas_context* pos_handler_impl::find_meas(du_cell_index_t cell_index, rnti_t rnti)
{
  for (auto& m : meass) {
    if (m.state != meas_state::running) {
      continue;
    }
    for (const auto& c : m.cells) {
      if (c.cell_index == cell_index and c.pos_rnti == rnti) {
        return &m;
      }
    }
  }
  return nullptr;
}

void pos_handler_impl::rem_cell(du_cell_index_t cell_index)
{
  ocudu_assert(cells.contains(cell_index), "Cell does not exist");
  // The cell is erased first, so that a caller that the completion below resumes cannot start a new measurement on it.
  cells.erase(cell_index);

  // The SRS of a removed cell is no longer reported, so its measurements end with an empty response. The queue
  // processing discards the queued ones.
  // Note: The callers resume inside the cell handler destructor. The MAC removes cells only when the DU configures its
  // cells at startup.
  for (auto& m : meass) {
    if (m.state == meas_state::running and
        std::any_of(
            m.cells.begin(), m.cells.end(), [cell_index](const meas_cell& c) { return c.cell_index == cell_index; })) {
      logger.warning("cell={}: Positioning measurement stopped. Cause: The cell was removed", cell_index);
      complete_meas(m, {});
    }
  }
  start_queued_meass();
}

} // namespace

std::unique_ptr<positioning_handler> ocudu::create_positioning_handler(scheduler_positioning_handler& sched,
                                                                       task_executor&                 ctrl_exec,
                                                                       timer_manager&                 timers,
                                                                       ocudulog::basic_logger&        logger)
{
  return std::make_unique<pos_handler_impl>(sched, ctrl_exec, timers, logger);
}
