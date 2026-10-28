// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/gateways/baseband/baseband_gateway_timestamp.h"

namespace ocudu {

/// \brief Lower physical layer - control interface.
///
/// Provides methods for starting and stopping the lower physical layer. The stop can be split in two phases, \c
/// request_stop() and \c wait_stop(), so the caller can unblock the baseband device in between.
///
/// \remark The methods \c start(), \c request_stop() and \c wait_stop() must be called in this order and only once;
/// other uses will result in undefined behavior. Calling \c stop() is equivalent to calling \c request_stop() and \c
/// wait_stop().
class lower_phy_controller
{
public:
  /// Default destructor.
  virtual ~lower_phy_controller() = default;

  /// \brief Starts the lower physical layer operation.
  ///
  /// The fist uplink processing block is expected at \c init_time and the first downlink processing block transmission
  /// will be relative to this time.
  ///
  /// \param[in] init_time     Initial time in clock ticks.
  /// \param[in] sfn0_ref_time System Frame Number (SFN) zero reference time.
  virtual void start(baseband_gateway_timestamp init_time, baseband_gateway_timestamp sfn0_ref_time = 0) = 0;

  /// \brief Requests the lower physical layer to stop.
  ///
  /// This call does not block, it signals the lower physical layer that a stop is requested.
  virtual void request_stop() = 0;

  /// \brief Waits for the lower physical layer to stop.
  ///
  /// This call is blocking, it waits for the stop request to be effective, i.e., waits for all asynchronous processes
  /// to complete. It must be called after \c request_stop().
  virtual void wait_stop() = 0;

  /// Stops the lower physical layer operation. Combines \c request_stop() and \c wait_stop().
  void stop()
  {
    request_stop();
    wait_stop();
  }
};

} // namespace ocudu
