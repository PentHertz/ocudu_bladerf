// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/phy/lower/lower_phy_rx_symbol_context.h"

namespace ocudu {

class shared_resource_grid;

/// Physical uplink demodulator notifier interface.
class puxch_processor_demodulator_notifier
{
public:
  /// Default destructor.
  virtual ~puxch_processor_demodulator_notifier() = default;

  /// \brief Notifies the completion of the OFDM demodulation of a symbol.
  ///
  /// The OFDM symbols of a slot are notified in ascending order. The notifications of different slots might be
  /// concurrent.
  ///
  /// \param[in] grid    Resource grid containing the demodulated OFDM symbol.
  /// \param[in] context Receive symbol context of the demodulated OFDM symbol.
  virtual void on_demodulation_completion(const shared_resource_grid&        grid,
                                          const lower_phy_rx_symbol_context& context) = 0;
};

} // namespace ocudu
