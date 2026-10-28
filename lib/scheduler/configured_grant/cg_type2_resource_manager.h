// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "../config/cell_configuration.h"
#include "ocudu/adt/circular_vector.h"
#include "ocudu/ocudulog/ocudulog.h"
#include "ocudu/ran/resource_allocation/rb_interval.h"
#include "ocudu/ran/rnti.h"
#include <optional>
#include <vector>

namespace ocudu {

struct cg_type2_resource {
  using res_id = unsigned;
  res_id       id;
  crb_interval crbs;
};

/// \brief Pool of the CG Type 2 resources a cell can hand out to its UEs.
///
/// The CG band is split into a fixed set of non-overlapping CRB blocks, one \ref cg_type2_resource each, and every
/// block is offered at each slot offset of the CG period that is usable for CG. A UE takes one of them when its
/// activation DCI is built, and holds it until the activation is released or the UE goes away; the manager tracks
/// which UE holds what, so that a resource is never handed out twice nor returned by a UE that does not hold it.
class cg_type2_resource_manager
{
public:
  explicit cg_type2_resource_manager(const cell_configuration& cell_cfg_);

  /// \brief Reserves a CG resource for \c rnti at the given slot offset.
  ///
  /// If \c rnti already holds a resource at that slot offset, an error is logged and that resource is returned, so that
  /// a repeated request does not take a second one.
  /// \param[in] cg_offset Slot offset within the CG period at which the resource is to be used.
  /// \param[in] rnti C-RNTI of the UE the resource is reserved for.
  /// \return The reserved resource, or nullopt if no resource is free at that slot offset.
  std::optional<cg_type2_resource> request_resource(unsigned cg_offset, rnti_t rnti);

  /// \brief Returns a CG resource held by \c rnti to the pool.
  ///
  /// \c rnti must be the UE the resource was reserved for. Freeing a resource that another UE still holds would hand
  /// its CRBs out a second time, putting two UEs on the same RBs in the same slot; such a call is rejected.
  /// \param[in] cg_offset Slot offset the resource was reserved at.
  /// \param[in] res_id Id of the resource to return.
  /// \param[in] rnti C-RNTI of the UE holding the resource.
  void deallocate_resource(unsigned cg_offset, cg_type2_resource::res_id res_id, rnti_t rnti);

  /// Total number of CG resources currently free across all the slot offsets of the wheel.
  unsigned get_nof_free_resources() const;

  /// Returns every CG resource to the pool, discarding all outstanding allocations.
  void reset();

private:
  // One entry per CG resource usable at a given slot offset.
  struct cg_res_occupancy {
    cg_type2_resource::res_id id;
    // This is set while the resource is reserved, so that a resource is never handed out twice, nor returned by a
    // UE that does not hold it.
    std::optional<rnti_t> owner;
  };

  const cell_configuration& cell_cfg;
  ocudulog::basic_logger&   logger;
  // Ring vector, one entry per slot offset within the CG period, that keeps track of the CG resources at that offset
  // and of the UE holding each of them. The entries at the offsets that compute_cg_usable_slot_offsets() excludes, as
  // they can fall on a PRACH or not full-UL slot, are left empty.
  circular_vector<std::vector<cg_res_occupancy>> resource_wheel;

  std::vector<cg_type2_resource> cg_resource_list;

  unsigned nof_free_resources;
};

} // namespace ocudu
