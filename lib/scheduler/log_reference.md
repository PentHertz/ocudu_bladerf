# Scheduler Log Reference

The MAC scheduler writes to the `SCHED` logger. Two independent loggers produce its output, both defined in `logging/`:

- **Result logger** (`scheduler_result_logger`) — one `Slot decisions` line per scheduled slot, describing the grants
  written into the `sched_result` (PDCCH, PDSCH, PUSCH, PUCCH, SRS, ...).
- **Event logger** (`scheduler_event_logger`) — one `Processed slot events` line per slot in which UE/cell events were
  processed (CRC, HARQ-ACK, BSR, UE creation, PRACH, ...).

Both are per-cell and tag every line with `pci=<physical cell id>`.

## Verbosity

The output is controlled by the level of the `SCHED` logger:

| Level        | Result logger                                            | Event logger                                                                               |
| ------------ | -------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| `info`       | Summary line + compact one-line entries (`log_info`)     | Only events with an info formatter: PRACH, RACH indication, ErrorIndication, cell creation |
| `debug`      | Summary line + one detailed entry per line (`log_debug`) | All events, one detailed entry per line                                                    |
| below `info` | disabled                                                 | disabled                                                                                   |

`debug` is a superset of `info`: it logs more entry types and more fields per entry.

### `log_broadcast`

The result logger is constructed with a `log_broadcast` flag. When `false`, these broadcast/common allocations are
omitted: SSB, SIB, CSI-RS, DL-PRS, PRACH occasions and `SI-RNTI` DL PDCCH. This is normally enabled for a single cell
only, so recurring broadcast allocations do not flood the log across all cells.

## Log line anatomy

```text
2026-07-02T09:29:21.835770 [SCHED   ] [D] [     1.5] Slot decisions pci=0 t=0us (1 PDSCH, ...):
└─ timestamp                 └─ logger  │    │        └─ message
                                        │    └─ slot as <SFN>.<slot-in-frame>
                                        └─ level: [I]=info, [D]=debug
```

The `[   SFN.slot]` field is the transmit slot the decisions apply to. Event lines carry the slot in which the events
were processed.

If an entry does not fit in the log buffer segment, the logger drops it and writes an error line:
`Scheduler result log entry exceeded segment size and was dropped` or
`Scheduler event log entry exceeded segment size and was dropped`.

---

## Result logger — `Slot decisions`

### Summary line

A slot is logged only if it carries something worth reporting. Empty slots are skipped.

- **info:** a SIB (with `log_broadcast`), a RAR/MsgB, a UE DL grant, a PUSCH, a paging grant, or a failed allocation
  attempt.
- **debug:** a broadcast allocation (with `log_broadcast`: SSB, SIB, CSI-RS, DL-PRS, PRACH), a non-`SI-RNTI` DL PDCCH, a
  UL PDCCH, a paging grant, a RAR/MsgB, a UE DL grant, a PUSCH, a PUCCH, an SRS, or a failed allocation attempt.

**info:**

```text
Slot decisions pci=0 t=0us (1 PDSCH, 1 PUSCH, 1 PUCCH): <entries...>
```

**debug** (adds attempted-but-failed allocation counters):

```text
Slot decisions pci=1 t=0us (1 PDSCH, 0 PUSCHs, 0 PUCCHs, 0 attempted PDCCHs, 0 attempted UCIs):
- <entry>
- <entry>
```

| Field               | Meaning                                                                             |
| ------------------- | ----------------------------------------------------------------------------------- |
| `t=<n>us`           | Scheduler decision latency for this slot, microseconds                              |
| `N PDSCH`           | Number of PDSCH grants (UE grants + RAR/MsgB + paging, + SIBs when `log_broadcast`) |
| `N PUSCH`           | Number of PUSCH grants                                                              |
| `N PUCCH`           | Number of PUCCH grants                                                              |
| `N attempted PDCCH` | Failed DL+UL PDCCH allocation attempts (`debug` only)                               |
| `N attempted UCI`   | Failed UCI allocation attempts (`debug` only)                                       |

At **info** level the entries follow the summary on the _same line_, comma-separated. At **debug** level each entry is
on its own line prefixed with a `-` and a space.

### info-level entries (compact, comma-separated)

| Prefix                     | Format                                                                                                                                                                                                                                                                                                                      |
| -------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SIB1` / `SI-<n>` / `SI-?` | `SIB1: rb=<prbs> tbs=<bytes>`. `SI-?` when the SI message index is unknown.                                                                                                                                                                                                                                                 |
| `RAR` / `MsgB`             | `RAR: ra-rnti=<rnti> rb=<prbs> tbs=<bytes>`. A 2-step grant prints `MsgB: msgb-rnti=<rnti> ...`.                                                                                                                                                                                                                            |
| `DL`                       | `DL: ue=<idx> c-rnti=<rnti> h_id=<harq> ss_id=<ss> rb=<prbs> k1=<k1> cw[0]: newtx=<true/false> rv=<rv> tbs=<bytes>`, then `cw[1]: newtx= rv= tbs=` with a second codeword. On new data appends `ri=<layers> dl_bo=<buffer-occupancy>`. When part of a Rel-16 PDSCH repetition bundle appends `reps=<n> reps_remaining=<n>`. |
| `UL`                       | `UL: ue=<idx> rnti=<rnti> h_id=<harq> ss_id=<ss> rb=<prbs> newtx=<true/false> rv=<rv> tbs=<bytes>`, then `k2=<k2>`, or `msg3_delay=<n>` for a Msg3 grant (invalid UE index, first tx).                                                                                                                                      |
| `PG`                       | `PG: rb=<prbs> tbs=<bytes> ues: <cn\|ran>-pg-id=<0x..>, ...`                                                                                                                                                                                                                                                                |

### debug-level entries (one per line, `-` prefix)

**Control channels:**

```text
- DL PDCCH: rnti=0x4601 type=c-rnti cs_id=1 ss_id=2 format=1_1 cce=8 al=2 dci: h_id=0 ndi=1 rv=0 mcs=3 res_ind=0 tdra_idx=3 tpc=1 dai=0 vrb_prb_map_used=no ant=0
- DL PDCCH: rnti=0xfffe type=p-rnti cs_id=0 ss_id=1 format=1_0 cce=0 al=4 dci: paging_ind=yes short_msg_ind=no tdra_idx=0
- UL PDCCH: rnti=0x4601 type=c-rnti cs_id=1 ss_id=2 format=0_1 cce=10 al=2 dci: h_id=0 ndi=1 rv=0 mcs=9 tpc=1 dai=0 mimo=1 ant=2
```

| Field             | Meaning                                                                                 |
| ----------------- | --------------------------------------------------------------------------------------- |
| `type`            | RNTI type (`c-rnti`, `tc-rnti`, `si-rnti`, ...)                                         |
| `cs_id` / `ss_id` | CORESET id / SearchSpace id                                                             |
| `format`          | DCI format (`1_0`, `1_1`, `0_0`, `0_1`)                                                 |
| `cce` / `al`      | Starting CCE index / aggregation level (number of CCEs)                                 |
| `dci: ...`        | Decoded DCI fields. The content depends on the RNTI type and the DCI format. See below. |

The `dci:` fields:

- **DL C-RNTI and TC-RNTI:** `h_id` HARQ process, `ndi`, `rv`, `mcs`, `res_ind` (PUCCH resource indicator), `tdra_idx`
  (PDSCH time-domain-resource-allocation row index). Optional `tpc` (not for TC-RNTI). Format 1_1 adds `dai` (when
  present), `vrb_prb_map_used` and `ant` (antenna ports).
- **DL P-RNTI:** `paging_ind=yes|no short_msg_ind=yes|no`. Then `tdra_idx` when the DCI schedules a paging PDSCH, and
  `short_msg=0x.. (sysInfoMod=yes|no etwsCmas=yes|no)` when the DCI carries a short message.
- **DL SI-RNTI and RA-RNTI:** no `dci:` fields.
- **UL:** `h_id`, `ndi`, `rv`, `mcs`, `tpc`. Format 0_1 adds `dai`, `mimo` (layers), `ant` and `csi_req`. UL DCIs do not
  print `tdra_idx`.

**DL shared channel:**

```text
- SSB: ssbIdx=0 crbs=[5..26) symb=[2..6)
- CSI-RS: type=nzp crbs=[0..52) row=1 freq=0010 symb0=4 cdm_type=no_CDM freq_density=three scramb_id=1
- PRS: crbs=[0..52) symb=[2..14) n_id=0 comb_size=2 comb_offset=0
- SIB1 PDSCH: rb=[0..4) symb=[2..14) tbs=120 mcs=5 rv=0
- SI-1 PDSCH: rb=[0..4) symb=[2..14) tbs=120 mcs=5 rv=0
- RAR PDSCH: ra-rnti=0x2 rb=[0..8) symb=[1..14) tbs=32 mcs=0 rv=0 grants (1): tc-rnti=0x4601: rapid=5 ta=12 time_res=0
- UE PDSCH: ue=0 c-rnti=0x4601 h_id=0 ss_id=2 rb=[0..25) symb=[1..14) cw[0]: tbs=309 mcs=9 rv=0 nrtx=0 k1=4 ri=1 <pmi> dl_bo=1024 olla=0.03 grants: lcid=1: size=53, lcid=4: size=256
- PCCH: rb=[0..4) symb=[2..14) tbs=40 mcs=5 rv=0 ues: cn-pg-id=0x1234
```

- `SI-<n> PDSCH`: `SI-? PDSCH` when the SI message index is unknown.
- `RAR PDSCH`: a 2-step grant prints `MsgB PDSCH: msgb-rnti=<rnti> ...`. Each 2-step grant appends
  `result=successRAR|fallbackRAR`.
- `UE PDSCH`:
  - `nrtx` = retransmission count.
  - A second codeword appends `cw[1]: tbs= mcs= rv=`.
  - `ri`/PMI printed only when precoding present.
  - `dl_bo` (DL buffer occupancy) only on new data.
  - `olla` (outer-loop link-adaptation offset) when set.
  - `reps`/`reps_remaining` only when the grant is part of a Rel-16 PDSCH repetition bundle (`reps` = nominal bundle
    size, `reps_remaining` = repetitions still to be transmitted after this occasion, decrementing to 0 at the last
    one).
  - Per-logical-channel `grants:` list when a TB is built, then a `cw[1] grants: ...` list for the second TB.

**UL shared channel and control:**

```text
- UE PUSCH: ue=0 c-rnti=0x4601 h_id=0 ss_id=2 rb=[1..52) symb=[0..14) tbs=1121 rv=0 nrtx=0 nof_layers=1 cg=No olla=0 k2=4 uci: harq_bits=1 csi-1_bits=0 csi-2_present=No
- PUCCH: c-rnti=0x4601 format=0 prb=[0..1) symb=[12..14) uci: harq_bits=1 sr=0
- PUCCH: c-rnti=0x4601 format=1 prb=[0..1) prb2=50 symb=[0..14) cs=0 occ=0 uci: harq_bits=1 sr=0
- PUCCH: c-rnti=0x4601 format=2 prb=[1..2) symb=[12..14) uci: harq_bits=2 sr=0 csi-1_bits=4
- PUCCH: c-rnti=0x4601 format=4 prb=[2..3) symb=[0..14) occ=0/2 uci: harq_bits=2 sr=0 csi-1_bits=4
- SRS: c-rnti=0x4601 symb=[8..10) tx-comb=(n2 o=0 cs=0) c_srs=0 f_sh=0 seq_id=0 requests=[ch_mtx=yes pos=no]
- PRACH: pci=1 format=0 nof_occasions=1 nof_preambles=64
```

- `UE PUSCH`: a leading `t` before `c-rnti` (`tc-rnti=`) marks a temporary-C-RNTI (Msg3) grant; `cg=Yes` marks a
  configured grant; `olla` when set; `k2` is replaced by `msg3_delay` for the initial Msg3. `uci:` block present when
  UCI is multiplexed on PUSCH.
- `PUCCH`: `prb2` shown when frequency hopping. Format-specific fields:
  - F0: no extra fields.
  - F1: `cs`, `occ`.
  - F2, F3: `csi-1_bits`.
  - F4: `occ=<idx>/<len>`, `csi-1_bits`.

### Interval notation

`rb`, `symb`, `crbs` are printed as half-open intervals `[start..stop)` — e.g. `rb=[1..52)` is 51 PRBs starting at PRB
1; `symb=[0..14)` is all 14 OFDM symbols.

---

## Event logger — `Processed slot events`

Emitted once per slot in which one or more UE/cell events were processed.

**info** (comma-separated, only events with an info formatter):

```text
Processed slot events pci=1: prach(ra-rnti=0x2 preamble=5 ssb=0 tc-rnti=0x4601), RACH Ind slot_rx=100.3, ErrorIndication slot=100.5
```

**debug** (one event per line):

```text
Processed slot events pci=1:
- UE creation: ue=0 rnti=0x4601
- CRC: ue=0 rnti=0x4601 rx_slot=401.9 h_id=0 crc=ok sinr=100dB
- HARQ-ACK: ue=0 rnti=0x4601 slot_rx=401.5 h_id=0 ack=1 tbs=720
- BSR: ue=0 rnti=0x4601 type="Short BSR" report={2: 700}
- CSI: ue=0 rnti=0x4601: slot_rx=408.9 cqi=15
- RLC Buffer State: ue=0 lcid=4 pending_bytes=2400
```

### Event types

| Event                                         | Levels       | Notes                                                                                                                                                                                          |
| --------------------------------------------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Cell creation                                 | info + debug | Emitted once at logger construction. info: `Cell creation idx=<n>`; debug: `Cell creation: idx=<n>`                                                                                            |
| PRACH                                         | info + debug | info: `prach(ra-rnti\|msgb-rnti=<rnti> preamble=<n> ssb=<n> tc-rnti=<rnti>)`; debug: `slot=<s> preamble=<n> ssb=<n> ra-rnti\|msgb-rnti=<rnti> temp_crnti=<rnti> ta_cmd=<n>`                    |
| RACH ind                                      | info + debug | debug lists each preamble: `preamble`, `tc-rnti`, `freq_idx`, `start_symbol`, `TA=<x>s` (seconds)                                                                                              |
| ErrorIndication                               | info + debug | debug: `slot=<s>`, then `, PDCCH discarded`, `, PDSCH discarded` and `, PUSCH and PUCCH discarded` for each discarded channel                                                                  |
| UE creation / reconfiguration / removal       | debug        | `ue=<idx> rnti=<rnti>`                                                                                                                                                                         |
| UE dedicated config applied / UE deactivation | debug        | `ue=<idx> rnti=<rnti>`                                                                                                                                                                         |
| SR                                            | debug        | Scheduling request                                                                                                                                                                             |
| CSI                                           | debug        | `cqi` / `ri` / `pmi` when present, else `invalid`                                                                                                                                              |
| BSR                                           | debug        | `type` (Short/Long/...), `report={...}`. Long, long truncated or all-LCG reports list all 8 LCGs, comma-separated, with `-` for an LCG not reported. Other reports list `<lcg>: <bytes>` pairs |
| HARQ-ACK                                      | debug        | `ack` status (0/1/2); `tbs` shown on ACK                                                                                                                                                       |
| CRC                                           | debug        | `crc=ok\|ko\|dtx`, `sinr=<n>dB` or `sinr=N/A`                                                                                                                                                  |
| MAC CE                                        | debug        | `ue=<idx> lcid=<lcid>`                                                                                                                                                                         |
| RLC Buffer State                              | debug        | `ue=<idx> lcid=<lcid> pending_bytes=<n>`                                                                                                                                                       |
| PHR                                           | debug        | `ue=<idx> rnti=<rnti>`, then for each cell `scell=<id> ph=<n>dB` and optional `p_cmax=<n>dBm`                                                                                                  |
| SRS                                           | debug        | optional `tpmi_info=[...]`                                                                                                                                                                     |
| Slice Reconfig                                | debug        | `cell=<idx>`                                                                                                                                                                                   |

---

## Common field glossary

| Field                   | Meaning                                                                                           |
| ----------------------- | ------------------------------------------------------------------------------------------------- |
| `pci`                   | Physical cell id                                                                                  |
| `ue`                    | DU UE index (internal); `INVALID` denotes a temporary (pre-attach) context                        |
| `c-rnti` / `rnti`       | UE C-RNTI                                                                                         |
| `tc-rnti`               | Temporary C-RNTI (RA / Msg3 / Msg4)                                                               |
| `ra-rnti` / `msgb-rnti` | RA-RNTI (4-step) / MsgB-RNTI (2-step) derived from the PRACH occasion                             |
| `h_id`                  | HARQ process id                                                                                   |
| `ndi` / `rv`            | New-data indicator / redundancy version                                                           |
| `newtx`                 | Whether the grant carries new data (`true`/`false`)                                               |
| `mcs`                   | Modulation and coding scheme index                                                                |
| `tdra_idx`              | PDSCH time-domain-resource-allocation row index signalled in the DL DCI                           |
| `tbs`                   | Transport block size, bytes                                                                       |
| `rb`                    | Allocated PRBs, half-open interval                                                                |
| `symb`                  | Allocated OFDM symbols, half-open interval                                                        |
| `cw[0]` / `cw[1]`       | First / second codeword of a PDSCH                                                                |
| `k1`                    | PDSCH-to-HARQ-ACK slot offset                                                                     |
| `k2`                    | PDCCH/DCI-to-PUSCH slot offset                                                                    |
| `nrtx`                  | HARQ retransmission count                                                                         |
| `reps`                  | Nominal number of occasions in a Rel-16 PDSCH repetition bundle (1 = no repetition)               |
| `reps_remaining`        | Occasions of the bundle still to be transmitted after this one, counting only committed occasions |
| `dl_bo`                 | DL buffer occupancy (pending bytes)                                                               |
| `olla`                  | Outer-loop link-adaptation MCS offset                                                             |
| `ss_id` / `cs_id`       | SearchSpace id / CORESET id                                                                       |
