// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_ptr_view.h"
#include "ocudu/phy/lower/lower_phy_rx_symbol_context.h"
#include "ocudu/support/error_handling.h"
#include <utility>

namespace ocudu {

/// \brief Scoped access to a physical uplink channels demodulator processing a slot.
///
/// A valid instance represents an open slot in the demodulator. The slot is closed when the instance is released or
/// destroyed.
class puxch_baseband_demodulator_scoped
{
public:
  /// Interface of the demodulator that processes the slot.
  class parent_interface
  {
  public:
    /// Default destructor.
    virtual ~parent_interface() = default;

    /// Demodulates an OFDM symbol of the slot.
    virtual void demodulate_symbol(const baseband_gateway_buffer_ptr_view& samples,
                                   const lower_phy_rx_symbol_context&      context) = 0;

    /// Closes the slot.
    virtual void close() = 0;
  };

  /// Creates an invalid instance.
  puxch_baseband_demodulator_scoped() = default;

  /// Creates an instance that keeps the slot of the given demodulator open.
  explicit puxch_baseband_demodulator_scoped(parent_interface& parent_) : parent(&parent_) {}

  /// Copy constructor is deleted, as only one instance can close the slot.
  puxch_baseband_demodulator_scoped(const puxch_baseband_demodulator_scoped&) = delete;

  /// Copy assignment is deleted, as only one instance can close the slot.
  puxch_baseband_demodulator_scoped& operator=(const puxch_baseband_demodulator_scoped&) = delete;

  /// Move constructor, it transfers the slot ownership.
  puxch_baseband_demodulator_scoped(puxch_baseband_demodulator_scoped&& other) noexcept :
    parent(std::exchange(other.parent, nullptr))
  {
  }

  /// Move assignment, it closes the current slot, if any, and transfers the slot ownership.
  puxch_baseband_demodulator_scoped& operator=(puxch_baseband_demodulator_scoped&& other) noexcept
  {
    if (this != &other) {
      release();
      parent = std::exchange(other.parent, nullptr);
    }
    return *this;
  }

  /// Closes the slot, if any.
  ~puxch_baseband_demodulator_scoped() { release(); }

  /// Closes the slot, if any, and invalidates the instance.
  void release()
  {
    if (parent) {
      parent->close();
      parent = nullptr;
    }
  }

  /// Returns true if the instance keeps a slot open.
  bool is_valid() const { return parent != nullptr; }

  /// Returns true if the instance keeps a slot open.
  explicit operator bool() const noexcept { return is_valid(); }

  /// \brief Demodulates an OFDM symbol of the slot.
  /// \remark An error is reported if the instance is invalid.
  void demodulate_symbol(const baseband_gateway_buffer_ptr_view& samples, const lower_phy_rx_symbol_context& context)
  {
    report_error_if_not(is_valid(), "Invalid demodulator.");
    parent->demodulate_symbol(samples, context);
  }

private:
  /// Demodulator processing the slot. Set to \c nullptr if the instance is invalid.
  parent_interface* parent = nullptr;
};

} // namespace ocudu
