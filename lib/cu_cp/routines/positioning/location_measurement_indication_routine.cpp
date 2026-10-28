// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "location_measurement_indication_routine.h"

using namespace ocudu;
using namespace ocucp;

location_measurement_indication_routine::location_measurement_indication_routine(
    cu_cp_ue_index_t         ue_index_,
    f1ap_ue_context_manager& f1ap_ue_ctxt_mng_,
    rrc_ue_interface&        rrc_ue_,
    ocudulog::basic_logger&  logger_) :
  ue_index(ue_index_), f1ap_ue_ctxt_mng(f1ap_ue_ctxt_mng_), rrc_ue(rrc_ue_), logger(logger_)
{
}

void location_measurement_indication_routine::operator()(coro_context<async_task<void>>& ctx)
{
  CORO_BEGIN(ctx);

  logger.info("ue={}: \"{}\" started...", ue_index, name());

  {
    // Send the measurement config and the ongoing location measurements of the UE to the DU.
    ue_context_mod_request.ue_index                             = ue_index;
    ue_context_mod_request.cu_to_du_rrc_info.emplace().meas_cfg = rrc_ue.get_packed_meas_config();
    if (std::optional<byte_buffer> location_meas_info = rrc_ue.get_packed_location_meas_info()) {
      ue_context_mod_request.cu_to_du_rrc_info->ie_exts.emplace().location_meas_info = std::move(location_meas_info);
    }

    CORO_AWAIT_VALUE(ue_context_mod_response,
                     f1ap_ue_ctxt_mng.handle_ue_context_modification_request(ue_context_mod_request));

    if (!ue_context_mod_response.success) {
      logger.warning("ue={}: \"{}\" failed. Cause: DU rejected the UE context modification", ue_index, name());
      CORO_EARLY_RETURN();
    }

    if (ue_context_mod_response.du_to_cu_rrc_info.meas_gap_cfg.empty()) {
      logger.info("ue={}: \"{}\" finished. DU returned no measurement gap config", ue_index, name());
      CORO_EARLY_RETURN();
    }
  }

  {
    // Send the measurement gap config and the updated cell group config to the UE.
    rrc_reconfig_args.meas_gap_cfg = ue_context_mod_response.du_to_cu_rrc_info.meas_gap_cfg.copy();
    if (!ue_context_mod_response.du_to_cu_rrc_info.cell_group_cfg.empty()) {
      rrc_ue.update_cell_group_config(ue_context_mod_response.du_to_cu_rrc_info.cell_group_cfg.copy());
      rrc_reconfig_args.non_crit_ext.emplace().master_cell_group =
          ue_context_mod_response.du_to_cu_rrc_info.cell_group_cfg.copy();
    }

    CORO_AWAIT_VALUE(rrc_reconfig_result, rrc_ue.handle_rrc_reconfiguration_request(rrc_reconfig_args));

    if (!rrc_reconfig_result) {
      logger.warning("ue={}: \"{}\" failed. Cause: RRC reconfiguration failed", ue_index, name());
      CORO_EARLY_RETURN();
    }
  }

  logger.info("ue={}: \"{}\" finished successfully", ue_index, name());

  CORO_RETURN();
}
