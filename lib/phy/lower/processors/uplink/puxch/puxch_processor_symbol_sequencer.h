// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once
#include "ocudu/phy/lower/lower_phy_rx_symbol_context.h"
#include "ocudu/phy/support/shared_resource_grid.h"
#include "ocudu/ran/cyclic_prefix.h"
#include <array>
#include <atomic>

namespace ocudu {

/// \brief  PUxCH processor OFDM symbol notification sequencer.
///
/// The PUxCH processor demodulates OFDM symbols asynchronously, resulting in an arbitrary order of their completed
/// notification. This class implements a thread-safe OFDM symbol notifier: it receives notifications of completed OFDM
/// symbols for a slot, and notifies them in ascending order.
class puxch_processor_symbol_sequencer
{
public:
  puxch_processor_symbol_sequencer(unsigned nof_symbols_per_slot) : nof_symbols(nof_symbols_per_slot) {}

  /// \brief Marks a symbol as ready and notifies, in order, the symbols that become ready as a consequence. It is safe
  /// to call from different threads.
  ///
  /// \param[in] context Receive symbol context of the ready symbol.
  /// \param[in] notify  Callable invoked with the context of each symbol to notify, in ascending order.
  /// \return \c false if the symbol was already set, \c true otherwise.
  template <typename Notify>
  bool set_symbol(const lower_phy_rx_symbol_context& context, Notify&& notify)
  {
    uint32_t this_symbol_mask = 1U << context.nof_symbols;

    // Store the context before marking the symbol as ready.
    contexts[context.nof_symbols] = context;

    // Mark the symbol as ready and try to become the notifying thread.
    uint32_t previous = state.fetch_or(this_symbol_mask | state_notifying_mask);
    bool     is_new   = (previous & this_symbol_mask) == 0;

    // Another thread is notifying symbols, and it will notify this one too.
    if (is_notifying(previous)) {
      return is_new;
    }

    // Current state value, i.e., previous state with the new symbol and the notifying mask.
    uint32_t current = previous | this_symbol_mask | state_notifying_mask;

    // Check if new symbols were demodulated while triggering notifications. Keep the loop until no more symbols are
    // notified in the meanwhile.
    uint32_t idle_state;
    do {
      // Advance cursor to the next incomplete symbol. Notify the completion of each complete symbol and advance the
      // cursor to the next OFDM symbol.
      while ((cursor != nof_symbols) && (current & (1U << cursor))) {
        notify(contexts[cursor]);
        ++cursor;
      }

      // State after notifying, i.e., without the notifying flag.
      idle_state = current & (~state_notifying_mask);
    } while (!state.compare_exchange_weak(current, idle_state));

    return is_new;
  }

  /// Resets the sequencer for a new slot.
  void reset()
  {
    cursor = 0;
    state  = 0;
  }

private:
  /// State value mask when the sequencer is notifying symbols.
  static constexpr uint32_t state_notifying_mask = 1U << 31;

  /// Return true if the value contains the notifying mask.
  bool is_notifying(uint32_t value) { return (value & state_notifying_mask) != 0; }

  /// Number of OFDM symbols per slot.
  unsigned nof_symbols;
  /// Ready symbols, one bit per symbol.
  std::atomic<uint32_t> state = 0;
  /// Receive symbol context of each symbol.
  std::array<lower_phy_rx_symbol_context, MAX_NSYMB_PER_SLOT> contexts;
  /// Next symbol to notify. Only the notifying thread accesses it.
  unsigned cursor = 0;
};

} // namespace ocudu
