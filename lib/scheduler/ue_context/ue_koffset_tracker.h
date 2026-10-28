// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ran/slot_point.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>

namespace ocudu {

/// Maximum Differential Koffset, as per TS 38.321, Section 6.1.3.57.
constexpr std::chrono::milliseconds MAX_DIFFERENTIAL_KOFFSET{63};

/// \brief Subframes from the HARQ-ACK of a Differential Koffset MAC CE until the UE applies its value.
///
/// The value applies from the first slot after the HARQ-ACK slot plus this many subframes, as per TS 38.213, Section 9.
constexpr unsigned DIFFERENTIAL_KOFFSET_APPLY_DELAY_SUBFRAMES = 3;

/// \brief Derives the Differential Koffset of a UE from its reported T_TA.
///
/// The UE Koffset still covers the reported T_TA plus \c margin, as per TS 38.300, Section 16.14.2.1.
///
/// \param[in] cell_koffset   Cell-specific Koffset.
/// \param[in] reported_ul_ta T_TA last reported by the UE.
/// \param[in] margin         Margin added to the reported T_TA.
/// \return The Differential Koffset, at most \c MAX_DIFFERENTIAL_KOFFSET.
inline std::chrono::milliseconds derive_differential_koffset(std::chrono::milliseconds cell_koffset,
                                                             std::chrono::microseconds reported_ul_ta,
                                                             std::chrono::milliseconds margin)
{
  const std::chrono::milliseconds reserved = std::chrono::ceil<std::chrono::milliseconds>(reported_ul_ta) + margin;
  if (reserved >= cell_koffset) {
    return std::chrono::milliseconds{0};
  }
  return std::min(cell_koffset - reserved, MAX_DIFFERENTIAL_KOFFSET);
}

/// \brief Tracks the Differential Koffset a UE applies, as per TS 38.321, Section 5.18.24.
///
/// Runtime state, like \c ue_ta_report_tracker.
///
/// A decoded value applies from the first slot three subframes past the HARQ-ACK of its PDSCH, as per TS 38.213,
/// Section 9. Late or missing feedback leaves it unknown until the HARQ resolves, see \c is_known_at(). A NACKed MAC
/// CE is not applied, and its retransmission moves the slot, as per TS 38.321, Section 5.3.2.2.
///
/// A larger value moves the UE uplink earlier, so the UE waits until the uplink it was already granted is behind it.
class ue_koffset_tracker
{
public:
  /// Returns the Differential Koffset the UE uplink is scheduled with.
  std::chrono::milliseconds applied_koffset() const { return applied; }

  /// Returns whether a Differential Koffset has been signalled and has not taken effect yet.
  bool has_pending_ce() const { return pending.has_value(); }

  /// \brief Checks whether a PDCCH in \c pdcch_slot can be given to the UE at the Differential Koffset it applies.
  ///
  /// False while uplink granted at a smaller Koffset reaches past the slots the current one leads to, and while a
  /// MAC CE in flight has had no HARQ-ACK reported. A UE that decodes a MAC CE may apply its value at once, and the
  /// two values place a grant in different slots.
  bool is_known_at(slot_point pdcch_slot) const
  {
    if (drain_until.valid() and pdcch_slot < drain_until) {
      return false;
    }
    return not pending.has_value() or pending->state != ce_state::awaiting_feedback;
  }

  /// \brief Records a Differential Koffset carried by a PDSCH.
  /// \param[in] koffset  Value carried by the MAC CE.
  /// \param[in] sl_apply Slot the UE applies the value from if it decodes this transmission.
  void handle_ce_sent(std::chrono::milliseconds koffset, slot_point sl_apply)
  {
    pending = pending_ce{koffset, sl_apply};
  }

  /// \brief Records a retransmission of the MAC CE, which moves the apply slot and holds the UE back again.
  ///
  /// The UE acts on the first transmission it decoded, as per TS 38.321, Section 5.3.2.2, so it may already be on the
  /// value. Nothing is scheduled against either until this transmission is reported.
  void handle_ce_retx(slot_point sl_apply)
  {
    if (pending.has_value() and pending->state != ce_state::acked) {
      pending->apply_slot = sl_apply;
      pending->state      = ce_state::awaiting_feedback;
    }
  }

  /// Records that the UE acknowledged the PDSCH carrying the MAC CE.
  void handle_ce_acked()
  {
    if (pending.has_value() and pending->state == ce_state::awaiting_feedback) {
      pending->state = ce_state::acked;
    }
  }

  /// Records that the UE reported it did not decode the PDSCH carrying the MAC CE, which a retransmission follows.
  void handle_ce_nacked()
  {
    if (pending.has_value() and pending->state == ce_state::awaiting_feedback) {
      pending->state = ce_state::nacked;
    }
  }

  /// Records that no further transmission of the MAC CE will be made, so its value is given up.
  void handle_ce_dropped() { pending.reset(); }

  /// Returns whether a MAC CE is in flight or the UE is held back, the cases \c slot_indication() acts on.
  bool is_active() const { return pending.has_value() or drain_until.valid(); }

  /// Returns whether a value in flight has not been acknowledged by the UE.
  bool has_unacknowledged_ce() const { return pending.has_value() and pending->state != ce_state::acked; }

  /// \brief Applies an acknowledged Differential Koffset once its apply slot is reached.
  /// \param[in] sl_tx        Current slot.
  /// \param[in] last_ul_slot Last slot granted to the UE for uplink transmission (PUSCH or HARQ-ACK). Invalid if none.
  void slot_indication(slot_point sl_tx, slot_point last_ul_slot)
  {
    if (drain_until.valid() and sl_tx >= drain_until) {
      drain_until.clear();
    }
    // The HARQ-ACK reaches the scheduler a round trip after the PDSCH, often past the apply slot of the value it
    // carries, so a value stays in flight until its HARQ resolves.
    if (not pending.has_value() or pending->state != ce_state::acked or sl_tx < pending->apply_slot) {
      return;
    }
    // A larger value moves the UE uplink earlier, so the uplink already granted reaches past the slots the new
    // value leads to. The UE waits those slots out, which keeps its transmissions in order.
    if (pending->koffset > applied and last_ul_slot.valid() and last_ul_slot > sl_tx) {
      drain_until = last_ul_slot + 1;
    }
    applied = pending->koffset;
    pending.reset();
  }

  /// \brief Discards the Differential Koffset of the UE.
  ///
  /// The UE does the same on MAC reset and on a satellite switch with re-synchronization, as per TS 38.321,
  /// Sections 5.12 and 5.2a.
  void reset()
  {
    applied = std::chrono::milliseconds{0};
    pending.reset();
    drain_until.clear();
  }

private:
  /// Delivery state of the MAC CE in flight.
  enum class ce_state : uint8_t {
    /// HARQ-ACK of the last transmission pending.
    awaiting_feedback,
    /// Decoded by the UE, applied from \c apply_slot.
    acked,
    /// Last transmission the UE reported it did not decode.
    nacked
  };

  /// Differential Koffset signalled to the UE and not applied yet.
  struct pending_ce {
    /// Value carried by the MAC CE.
    std::chrono::milliseconds koffset;
    /// Slot the UE applies the value from if it decoded the last transmission.
    slot_point apply_slot;
    /// Delivery state of the last transmission.
    ce_state state = ce_state::awaiting_feedback;
  };

  /// Differential Koffset the UE applies.
  std::chrono::milliseconds applied{0};
  /// Differential Koffset MAC CE signalled to the UE and not applied yet.
  std::optional<pending_ce> pending;
  /// Slot the UE is scheduled again from after its Differential Koffset grew. Invalid when it is not held back.
  slot_point drain_until;
};

} // namespace ocudu
