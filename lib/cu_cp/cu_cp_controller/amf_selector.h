// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/adt/span.h"
#include "ocudu/ngap/ngap_context.h"
#include "ocudu/ngap/ngap_setup.h"
#include "ocudu/ran/amf_selection_request.h"
#include "ocudu/ran/cu_cp_types.h"
#include "ocudu/ran/guami.h"
#include "ocudu/ran/plmn_identity.h"
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

namespace ocudu::ocucp {

/// AMF that the CU-CP can select for a UE.
struct amf_candidate {
  cu_cp_amf_index_t amf_index = cu_cp_amf_index_t::invalid;
  /// Name of the AMF (TS 38.413 section 9.3.3.21).
  std::string amf_name;
  /// True if the NG-C interface instance to the AMF is set up.
  bool connected = false;
  /// PLMNs that the AMF supports and that the CU-CP serves through it.
  std::vector<plmn_identity> plmns;
  /// PLMNs and slices that the AMF supports (TS 38.413 section 9.2.6.2).
  std::vector<ngap_plmn_support_item> plmn_support_list;
  /// GUAMIs that the AMF serves.
  std::vector<ngap_served_guami_item> served_guamis;
  /// Served GUAMIs that the AMF marks as unavailable (TS 38.413 section 8.7.6.2).
  std::vector<ngap_unavailable_guami_item> unavailable_guamis;
  /// Capacity of the AMF relative to the other AMFs of its AMF Set (TS 38.413 section 9.3.1.32).
  uint16_t relative_amf_capacity = 0;
  /// Overload state of the AMF. Absent if the AMF is not overloaded (TS 38.413 section 8.7.7.2).
  std::optional<ngap_amf_overload> overload;
};

/// \brief Selects the AMF that serves a UE (NAS Node Selection Function).
///
/// The selection draws random numbers to weight the AMFs by their relative AMF capacity. The selector owns the
/// random generator.
class amf_selector
{
public:
  /// \param[in] seed Seed of the random generator.
  explicit amf_selector(uint32_t seed = std::random_device{}()) : rng(seed) {}

  /// \brief Decides which connected AMFs reject a new UE with the given establishment cause.
  ///
  /// An overloaded AMF rejects the establishment causes of its overload action. An AMF with a traffic load reduction
  /// rejects the UE with the probability of the reduction (TS 38.413 section 8.7.7.2).
  /// \return The AMFs that reject the UE, or std::nullopt if every connected AMF rejects the UE.
  std::optional<std::vector<cu_cp_amf_index_t>> admit_ue_setup(span<const amf_candidate> candidates,
                                                               establishment_cause_t     establishment_cause);

  /// \brief Checks if the AMF of an RRC_INACTIVE UE accepts its resume.
  ///
  /// The overload of the AMF also applies to the RRC Connection Resume (TS 23.501 section 5.19.5.2). For the slice
  /// overload, the Allowed NSSAI of the UE takes the place of the requested NSSAI. An RNA update sends no signalling to
  /// the AMF, so the AMF accepts it.
  bool is_ue_resume_accepted(const amf_candidate&  candidate,
                             resume_cause_t        resume_cause,
                             span<const s_nssai_t> allowed_nssai);

  /// \brief Selects the AMF for a UE that accesses the CU-CP (NAS Node Selection Function, TS 38.410 section 5.7 and
  /// TS 38.300 section 16.3.4.2).
  ///
  /// The selection first removes the AMFs whose overload action rejects the establishment cause of the UE, and the AMFs
  /// whose slice overload rejects the UE. An AMF with a traffic load reduction is removed with the probability of the
  /// reduction (TS 38.413 section 8.7.7.2). If the RRC Setup Request of the UE already decided the overload action, the
  /// selection removes the AMFs of this decision. Then the selection applies these rules in order. Each rule takes only
  /// the connected AMFs that support the selected PLMN.
  /// 1. An AMF that serves the registered GUAMI of the UE with the same GUAMI Type.
  /// 2. An AMF that serves the AMF Set ID and AMF Pointer of the 5G-S-TMSI in the selected PLMN.
  /// 3. If the UE identifies an AMF Set, an AMF of this AMF Set.
  /// 4. Any AMF. If the UE requests S-NSSAIs, an AMF that supports all of them comes first, then an AMF that supports
  ///    some of them. TS 38.300 section 16.3.4.2 and TS 23.501 section 6.3.5 let the NG-RAN node decide how the
  ///    requested S-NSSAIs select the AMF. If no AMF supports a requested S-NSSAI, any AMF serves the UE as a default
  ///    AMF (TS 23.501 section 5.15.5.2.1).
  ///
  /// If a rule finds several AMFs, the selection picks one at random, weighted by its relative AMF capacity. An AMF
  /// with capacity 0 is picked only if all the AMFs of the rule have capacity 0. In rule 4, the AMFs can belong to
  /// different AMF Sets. The selection then picks an AMF Set with a probability that is proportional to the total
  /// capacity of its AMFs. TS 23.501 section 6.3.5 leaves the selection of the AMF Set to local policy.
  ///
  /// In rules 1 and 2, the backup AMF serves the UE if the GUAMI is unavailable or its AMF is not connected. The backup
  /// AMF Name from the AMF Status Indication comes first, then the one from the served GUAMI list. Rule 3 skips an AMF
  /// whose served GUAMIs are all unavailable, and rule 4 uses such an AMF only if no other AMF is left. Then rule 4
  /// applies the requested S-NSSAIs. Within a rule, an overloaded AMF is picked only if no other AMF of the rule is
  /// left.
  /// \return The index of the AMF, or std::nullopt if no connected AMF of the selected PLMN accepts the UE.
  std::optional<cu_cp_amf_index_t> select_amf(span<const amf_candidate>    candidates,
                                              const amf_selection_request& request);

  /// \brief Finds the connected AMF that serves a GUAMI.
  ///
  /// The AMFs that serve the GUAMI come first, or their backup AMF, as in rule 1 of \ref select_amf. Otherwise, the
  /// selection picks a connected AMF of the same AMF Set, as in rule 4 of \ref select_amf.
  /// \return The index of the AMF, or std::nullopt if no AMF of the AMF Set is connected.
  std::optional<cu_cp_amf_index_t> select_amf_for_guami(span<const amf_candidate> candidates, const guami_t& guami);

  /// \brief Selects a connected AMF of an AMF Set that supports a PLMN.
  ///
  /// The AMF is picked at random and weighted by its relative AMF capacity, as in rule 4 of \ref select_amf.
  /// An AMF whose served GUAMIs are all unavailable is picked only if no other AMF of the AMF Set is connected. An
  /// overloaded AMF is picked only if no AMF that is not overloaded is left.
  /// \return The index of the AMF, or std::nullopt if no AMF of the AMF Set is connected.
  std::optional<cu_cp_amf_index_t>
  select_amf_for_amf_set(span<const amf_candidate> candidates, const plmn_identity& plmn, uint16_t amf_set_id);

private:
  /// Picks one of the candidates at random, weighted by the relative AMF capacity.
  std::optional<cu_cp_amf_index_t> pick_by_capacity(span<const amf_candidate* const> pool);

  /// Picks one of the AMFs with the lowest preference rank, as in \ref pick_by_capacity.
  std::optional<cu_cp_amf_index_t> pick_preferred_by_capacity(span<const amf_candidate* const> pool);

  /// Picks one of the connected AMFs that match, as in \ref pick_preferred_by_capacity.
  template <typename CandidateMatch>
  std::optional<cu_cp_amf_index_t> pick_matching_by_capacity(span<const amf_candidate> candidates,
                                                             const CandidateMatch&     matches);

  /// \brief Finds the AMF of a served GUAMI that matches the UE.
  ///
  /// The connected and eligible AMFs of the matching GUAMIs are picked as in \ref pick_by_capacity. If the matching
  /// GUAMIs are unavailable, or their AMFs are not connected, the connected backup AMF serves the UE. The backup AMF
  /// Name from the AMF Status Indication comes first (TS 38.413 sections 8.7.1.2 and 8.7.6.2).
  template <typename GuamiMatch, typename CandidateMatch>
  std::optional<cu_cp_amf_index_t>
  find_amf_of_guami(span<const amf_candidate> candidates, const GuamiMatch& matches, const CandidateMatch& is_eligible);

  /// Decides if the NG-RAN node rejects a UE whose signalling traffic an overload affects. With a traffic load
  /// reduction, the NG-RAN node rejects the UE with the probability of the reduction.
  bool draw_rejection(std::optional<uint8_t> traffic_load_reduction);

  /// Checks if the overload action of the AMF rejects the UE. With a traffic load reduction, the AMF rejects the UE
  /// with the probability of the reduction.
  template <typename Cause>
  bool is_rejected_by_amf_overload(const amf_candidate& candidate, Cause cause);

  /// \brief Checks if the slice overload of the AMF rejects the UE (TS 38.413 section 8.7.7.2).
  ///
  /// The slice overload affects a UE whose requested S-NSSAIs are all in the Overload Start NSSAI List. TS 38.413 does
  /// not define how several items combine. Each item that lists a requested S-NSSAI can reject the UE, and each item
  /// applies its own traffic load reduction.
  template <typename Cause>
  bool
  is_rejected_by_slice_overload(const ngap_amf_overload& overload, span<const s_nssai_t> requested_nssai, Cause cause);

  /// Checks if the AMF accepts a new UE.
  bool accepts_ue(const amf_candidate& candidate, const amf_selection_request& request);

  std::mt19937 rng;
};

} // namespace ocudu::ocucp
