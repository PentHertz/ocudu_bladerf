// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "puxch_baseband_demodulator_scoped.h"
#include "puxch_processor_demodulator_notifier.h"
#include "puxch_processor_symbol_sequencer.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_ptr_view.h"
#include "ocudu/instrumentation/traces/ru_traces.h"
#include "ocudu/ocudulog/ocudulog.h"
#include "ocudu/ocuduvec/zero.h"
#include "ocudu/phy/lower/lower_phy_rx_symbol_context.h"
#include "ocudu/phy/lower/modulation/ofdm_demodulator.h"
#include "ocudu/phy/support/resource_grid_context.h"
#include "ocudu/phy/support/resource_grid_writer.h"
#include "ocudu/phy/support/shared_resource_grid.h"
#include "ocudu/support/executors/task_executor.h"
#include "ocudu/support/synchronization/stop_event.h"
#include <array>
#include <atomic>

namespace ocudu {

/// \brief Physical uplink channels demodulator.
///
/// This class asynchronously demodulates the OFDM symbols of a slot into a resource grid. The demodulation process is
/// split in tasks per port. The demodulated OFDM symbols are notified in ascending order through the \ref
/// puxch_processor_demodulator_notifier interface.
class puxch_baseband_demodulator : private puxch_baseband_demodulator_scoped::parent_interface
{
public:
  /// \brief Creates a physical uplink channel demodulator from all the required parameters.
  ///
  /// \param[in] cp           Cyclic prefix.
  /// \param[in] nof_ports_   Number of receive ports.
  /// \param[in] executor_    Task executor for processing baseband demodulation.
  /// \param[in] demodulator_ OFDM demodulator, called from the executor.
  /// \param[in] notifier_    Reference to the interface for notifying the demodulated OFDM symbols.
  puxch_baseband_demodulator(cyclic_prefix                         cp,
                             unsigned                              nof_ports_,
                             task_executor&                        executor_,
                             ofdm_symbol_demodulator&              demodulator_,
                             puxch_processor_demodulator_notifier& notifier_) :
    logger(ocudulog::fetch_basic_logger("PHY")),
    nof_ports(nof_ports_),
    nof_symbols_per_slot(get_nsymb_per_slot(cp)),
    sequencer(nof_symbols_per_slot),
    executor(executor_),
    demodulator(demodulator_),
    notifier(notifier_)
  {
  }

  /// \brief Starts the demodulation of a slot.
  ///
  /// The demodulator stays busy from the start of the slot until the returned scoped demodulator is released and all
  /// the demodulation tasks are completed.
  ///
  /// \param[in] grid    Resource grid, destination of the demodulated OFDM symbols.
  /// \param[in] context Resource grid context of the slot.
  /// \return A valid scoped demodulator that keeps the slot open if the demodulation of the slot started
  ///         successfully, otherwise an invalid one if the demodulator is busy with a previous slot.
  /// \remark An assertion is triggered if the number of ports contained in the resource grid differs from the number of
  ///         receive ports.
  puxch_baseband_demodulator_scoped start(const shared_resource_grid& grid, const resource_grid_context& context)
  {
    ocudu_assert(grid.get_writer().get_nof_ports() == nof_ports,
                 "The resource grid number of ports (i.e., {}) does not match the number of ports (i.e., {}).",
                 grid.get_writer().get_nof_ports(),
                 nof_ports);

    // Try transitioning to demodulate. The open slot counts as a pending task.
    uint32_t expected_state = state_idle;
    if (!current_state.compare_exchange_strong(expected_state, state_demodulate_mask + 1)) {
      return {};
    }

    // Prepare current context and grid for the asynchronous OFDM demodulation.
    current_context  = context;
    current_grid     = grid.copy();
    received_symbols = 0;
    sequencer.reset();

    return puxch_baseband_demodulator_scoped(*this);
  }

  /// Stops the demodulator. It blocks until all the pending demodulation tasks are completed.
  void stop() { stop_control.stop(); }

private:
  /// \brief Demodulates an OFDM symbol of the current slot.
  ///
  /// \param[in] samples Baseband samples of the OFDM symbol.
  /// \param[in] context Receive symbol context.
  /// \remark An assertion is triggered if the demodulator is not processing the slot of the OFDM symbol.
  /// \remark An assertion is triggered if the OFDM symbol has already been demodulated.
  void demodulate_symbol(const baseband_gateway_buffer_ptr_view& samples,
                         const lower_phy_rx_symbol_context&      context) override
  {
    ocudu_assert(samples.get_nof_channels() == nof_ports,
                 "The buffer number of channels (i.e., {}) does not match the number of ports (i.e., {}).",
                 samples.get_nof_channels(),
                 nof_ports);
    ocudu_assert(context.slot == current_context.slot,
                 "The symbol slot (i.e., {}) does not match with the demodulator slot (i.e., {}).",
                 context.slot,
                 current_context.slot);

    unsigned i_symbol    = context.nof_symbols;
    uint32_t symbol_mask = 1U << i_symbol;
    ocudu_assert((received_symbols & symbol_mask) == 0, "The OFDM symbol {} is already demodulated.", i_symbol);
    received_symbols |= symbol_mask;

    // Prepare the OFDM symbol entry for the asynchronous OFDM demodulation.
    symbol_entry& entry = symbols[i_symbol];
    entry.samples       = samples.clone();
    entry.context       = context;
    entry.pending_ports = nof_ports;
    entry.is_valid      = true;

    // Set the number of pending tasks.
    current_state.fetch_add(nof_ports);

    // Symbol index within the subframe.
    unsigned i_symbol_sf = i_symbol + context.slot.subframe_slot_index() * nof_symbols_per_slot;

    // Demodulate asynchronously each of the ports.
    for (unsigned i_port = 0; i_port != nof_ports; ++i_port) {
      // The task keeps a stop token until it is destroyed, so the demodulator does not stop with pending tasks.
      auto token = stop_control.get_token();

      // Skip the demodulation of the port if the demodulator is stopped.
      if (token.is_stop_requested()) {
        ocuduvec::zero(current_grid.get().get_writer().get_view(i_port, i_symbol_sf));
        entry.is_valid = false;
        complete_port_task(i_symbol);
        continue;
      }

      bool success = executor.defer([this, i_symbol, i_symbol_sf, i_port, tk = std::move(token)]() {
        // Start tracing.
        trace_point tp = ru_tracer.now();

        // OFDM demodulation.
        demodulator.demodulate(
            current_grid.get().get_writer(), symbols[i_symbol].samples.get_channel_buffer(i_port), i_port, i_symbol_sf);

        ru_tracer << trace_event("PUxCH demodulation", tp);

        // Indicate completion of the task.
        complete_port_task(i_symbol);
      });

      if (!success) {
        logger.error(context.slot.sfn(),
                     context.slot.slot_index(),
                     "Failed to enqueue demodulation task for symbol {} and port {}.",
                     i_symbol_sf,
                     i_port);
        ocuduvec::zero(current_grid.get().get_writer().get_view(i_port, i_symbol_sf));
        entry.is_valid = false;
        complete_port_task(i_symbol);
      }
    }
  }

  /// Closes the current slot, notifying pending symbols as invalid.
  void close() override
  {
    // Notify the OFDM symbols that have not been received as invalid.
    lower_phy_rx_symbol_context context = {
        .slot = current_context.slot, .sector = current_context.sector, .nof_symbols = 0, .is_valid = false};
    for (unsigned i_symbol = 0; i_symbol != nof_symbols_per_slot; ++i_symbol) {
      if ((received_symbols & (1U << i_symbol)) == 0) {
        context.nof_symbols = i_symbol;
        sequencer.set_symbol(context, [this](const lower_phy_rx_symbol_context& symbol_context) {
          notifier.on_demodulation_completion(current_grid, symbol_context);
        });
      }
    }

    // Closing the slot counts as completing a task.
    complete_task();
  }

  /// Collects the information of an OFDM symbol under demodulation.
  struct symbol_entry {
    /// Baseband samples of the OFDM symbol.
    baseband_gateway_buffer_ptr_view samples;
    /// Receive symbol context.
    lower_phy_rx_symbol_context context;
    /// Number of port tasks pending for completion.
    std::atomic<unsigned> pending_ports = 0;
    /// Set to true if the demodulation tasks of all ports could be executed.
    bool is_valid;
  };

  /// Helper method called upon the completion of the demodulation of an OFDM symbol port.
  void complete_port_task(unsigned i_symbol)
  {
    symbol_entry& entry = symbols[i_symbol];

    // Release the samples buffer if this is the last task of the symbol.
    if (entry.pending_ports.fetch_sub(1) == 1) {
      entry.samples.release();

      // The symbol is valid if all its ports are demodulated.
      entry.context.is_valid = entry.is_valid;

      // Notify the completed symbol to the sequencer, and trigger the notification of symbols enabled by this one.
      [[maybe_unused]] bool success =
          sequencer.set_symbol(entry.context, [this](const lower_phy_rx_symbol_context& symbol_context) {
            notifier.on_demodulation_completion(current_grid, symbol_context);
          });
      ocudu_assert(success, "The OFDM symbol {} is already completed.", entry.context.nof_symbols);
    }

    complete_task();
  }

  /// Helper method to handle the completion of a demodulation task.
  void complete_task()
  {
    // Decrement counter.
    uint32_t prev = current_state.fetch_sub(1);
    ocudu_assert((prev & state_demodulate_mask) == state_demodulate_mask, "Unexpected state 0x{:08x}.", prev);

    // Skip completion actions if the slot is open or the demodulation tasks are not completed.
    if (prev != state_completed) {
      return;
    }

    // The resource grid is no longer necessary.
    current_grid = {};

    // Transition demodulator state to idle which becomes available for the next use.
    [[maybe_unused]] uint32_t expected_mask = current_state.exchange(state_idle);
    ocudu_assert(expected_mask == state_demodulate_mask, "Unexpected state 0x{:08x}.", prev);
  }

  /// State value for when the demodulator is not processing any slot.
  static constexpr uint32_t state_idle = 0x0;
  /// State mask for when the demodulator is processing a slot and has not completed all the tasks yet.
  static constexpr uint32_t state_demodulate_mask = 0x80000000;
  /// Expected state when the last task of the demodulator fetches and decreases the count.
  static constexpr uint32_t state_completed = state_demodulate_mask + 1;

  /// Physical layer logger.
  ocudulog::basic_logger& logger;
  /// Number of receive ports.
  unsigned nof_ports;
  /// Number of symbols per slot.
  unsigned nof_symbols_per_slot;
  /// Current demodulator state.
  std::atomic<uint32_t> current_state = state_idle;
  /// Current slot processing context.
  resource_grid_context current_context;
  /// Current resource grid.
  shared_resource_grid current_grid;
  /// OFDM symbols of the slot that have been received, one bit per symbol.
  uint32_t received_symbols = 0;
  /// OFDM symbols under demodulation.
  std::array<symbol_entry, MAX_NSYMB_PER_SLOT> symbols;
  /// Sequences the notifications of the demodulated OFDM symbols.
  puxch_processor_symbol_sequencer sequencer;
  /// Executor for asynchronously demodulating.
  task_executor& executor;
  /// OFDM demodulator. Its implementation must be thread safe.
  ofdm_symbol_demodulator& demodulator;
  /// Notifier of the demodulated OFDM symbols.
  puxch_processor_demodulator_notifier& notifier;
  /// Stop control.
  rt_stop_event_source stop_control;
};

} // namespace ocudu
