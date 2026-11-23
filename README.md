# Fixed-Point Streaming FIR Accelerator (FPGA DSP Core)

A synthesizable SystemVerilog digital-signal-processing IP block: a
parameterized, signed fixed-point, streaming finite-impulse-response (FIR)
filter with an independent integer-arithmetic Python reference model, a
self-checking SystemVerilog testbench, optional decimating mode, and a Yosys
synthesis flow.

The reference model and the RTL are bit-exact: every coefficient is
quantized the same way, intermediate widths are explicit, and rounding and
saturation rules are identical.

```
fir_fixed/
├── rtl/                 # SystemVerilog RTL (arithmetic / control / interface)
│   ├── fir_arith.sv     #   delay line, multipliers, adder tree, round/saturate
│   ├── fir_control.sv   #   valid/ready handshake, valid tracking, decimation
│   └── fir_filter.sv    #   top-level wrapper + output register
├── sim/
│   ├── tb_fir.sv        # self-checking data-driven testbench
│   ├── tb_perf.sv       # latency / throughput testbench
│   └── vectors/         # stimulus tapes, expected outputs, coeff ROMs, cfg header
├── scripts/
│   ├── ref_model.py     # independent integer reference model + vector generator
│   └── run_all.py       # full verification orchestrator
├── synth/
│   └── synth_flow.ys    # Yosys synthesis script
├── examples/            # small worked demonstrations
├── docs/                # technical document + verification report
├── Makefile
└── README.md
```

## 1. Features

- Parameterized number of taps (`TAPS`), sample width (`DW`), coefficient
  width (`CW`).
- Signed fixed point: default S1.15 samples and coefficients
  (FRAC_IN=FRAC_CO=FRAC_OUT=15).
- Explicit intermediate widths: 32-bit products, 36-bit accumulator with
  `D = ceil(log2(TAPS))` guard bits (provably overflow-free for every
  partial sum), 21-bit post-shift requantizer.
- Configurable rounding (round-half-up vs. truncate) and saturation
  (saturate vs. two's-complement wraparound), both implemented explicitly.
- Streaming AXI-Stream-like valid/ready handshake with a **locked** (no
  skid-buffer) protocol: `s_ready = m_ready | ~m_valid`. The whole datapath
  freezes in lockstep under backpressure, so no sample is ever lost or
  duplicated.
- Synchronous active-low reset with well-defined behavior.
- Coefficients via a packed parameter `COEFFS` (tap 0 in the MSBs), or via a
  `$readmemh` coefficient ROM file (compile with `-DFIR_COEFF_FILE="<file>"`).
- Optional decimating FIR mode (`DECIM_EN=1`, `DECIM_PERIOD`): filters every
  input sample but outputs one sample per `DECIM_PERIOD` accepted samples.
- Arithmetic, control, and interface logic in three separate modules.

## 2. Prerequisites

Tested on Ubuntu 22.04.5:

| Tool | Version tested | Purpose |
|---|---|---|
| Python | 3.12.11 | reference model, vector generation, harness |
| Icarus Verilog | 11.0 | RTL simulation (`iverilog`, `vvp`) |
| Yosys | 0.70+240 (oss-cad-suite 2026-10-27) | logic synthesis (optional) |

`iverilog`/`vvp` must be on `PATH`. Yosys is optional: the harness runs and
reports the synthesis result only if `yosys` is found (or `YOSYS` is set to
a binary path). The Ubuntu 22.04 apt `yosys 0.9` is **too old** to read this
RTL (`parameter int`); use the official YosysHQ oss-cad-suite instead:

```bash
curl -L -o /tmp/oss-cad-suite.tgz \
  https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2026-10-27/oss-cad-suite-linux-x64-20261027.tgz
mkdir -p /tmp/oss && tar -xzf /tmp/oss-cad-suite.tgz -C /tmp/oss/
export YOSYS=/tmp/oss/oss-cad-suite/bin/yosys
```

## 3. Quick start

```bash
cd fir_fixed

# full verification (16 scenarios + 2 performance tests + synthesis if yosys available)
python3 scripts/run_all.py
# or: make test

# just regenerate all stimulus/expected-vector files
python3 scripts/ref_model.py --scenario all --outdir sim/vectors
# or: make gen

# latency/throughput testbench only
make perf
```

Expected result (with `YOSYS` set):

```
passed=19 failed=0 skipped=0
```

## 4. Fixed-point design (defaults)

| Quantity | Width | Format | Notes |
|---|---|---|---|
| input sample | DW=16 | S1.15 | 1 sign + 15 fractional bits |
| coefficient | CW=16 | S1.15 | DC gain normalized to 1 |
| product | PW=DW+CW=32 | S2.30 | exact, never truncated |
| accumulator | ACCW=PW+D=36 | S(2+D).30, D=4 | 4 guard bits, overflow-free |
| post-shift y | EXT=ACCW+1-DW-FD=21 | S(EXT+1-16).15 | floor shift (`>>>`) |
| output | DW=16 | S1.15 | rounded then saturated/truncated |

- **Accumulator width proof**: `|product| < 2^(PW-2)`; any partial sum of
  `k <= TAPS` products has `|sum| < 2^(PW-2) * k <= 2^(PW-2+D)`, and a signed
  ACCW-bit value holds `|x| < 2^(ACCW-1) = 2^(PW+D-1)`. Every pairwise add
  in the tree is therefore exact. The reference model asserts this invariant
  (`|acc| < 2^(ACCW-1)`) on every accepted sample.
- **Rounding**: optional round-half-up — add `1 << (FRAC_IN+FRAC_CO-FRAC_OUT-1)`
  = 2^14 before the final arithmetic right shift.
- **Saturation vs. wraparound**: with SAT=1 the **post-shift** value is
  clamped to `[-2^(DW-1), 2^(DW-1)-1]`; with SAT=0 the low DW bits of the
  shifted value are kept (explicit two's-complement wraparound). Saturation
  compares the post-shift value, not the pre-shift guard bits — a value like
  35283 that fits in the accumulator but not in the output is saturated to
  32767 (verified by the `lpf_random` scenario).

## 5. Architecture

```
                   +------------------------------------------------------+
 s_valid ──►      |  fir_control.sv                                       |
 s_data ──►       |   s_ready = m_ready | ~valid_out   (locked handshake) |
                  |   shift_en = s_valid & s_ready                         |
                  |   p / allow_p : CHAIN-deep position + decim flags      |
                  |   arrival = shift_en & p[CHAIN-3]                      |
                  |   due: latched arrival                                 |
                  |   capture_en = due & shift_en & allow_p[CHAIN-2]       |
                  |                & (m_ready | ~valid_out)                |
                  |   dcount: decimation counter (DECIM_PERIOD)            |
                  +------------------------------------------------------+
                        | shift_en                | capture_en
                        v                         v
                  +------------------------------------------------------+
                  |  fir_arith.sv                                        |
                  |   delay line dl[TAPS]  (gated by shift_en)           |
                  |   coeff[TAPS]  (packed param or $readmemh ROM)       |
                  |   products: dl_next[i] * coeff[i]  (32-bit, exact)   |
                  |   adder tree: D+1 levels of 36-bit adds              |
                  |   y_q = round >> FRAC_OUT, then saturate/wrap        |
                  +------------------------------------------------------+
                        | y_q (combinational)
                        v
                  +------------------------------------------------------+
                  |  fir_filter.sv (top)                                 |
                  |   m_data_r <= y_q when capture_en                    |
                  |   m_valid = valid_out   m_data = m_data_r            |
                  +------------------------------------------------------+
```

Latency: **D+1 = 5 clock cycles** from acceptance of a sample to the cycle in
which the corresponding output is valid (measured by `tb_perf`; D = 4 for 16
taps). Throughput: **1 sample/cycle** in steady state with both handshakes
ready; **1/DECIM_PERIOD** samples/cycle when decimating.

## 6. Reference model and vector generation

`scripts/ref_model.py` is an independent, cycle-accurate integer model of the
RTL. It mirrors the control registers (`p`, `allow_p`, `due`, `valid_out`,
`dcount`), uses the same quantization (`q_format`, `clamp_sat`, round-half-up
then floor shift), and reproduces the RTL's numerical behavior bit-for-bit.

Scenarios (all with seed `0xC0FFEE`, 400 stimulus cycles + drain):

| Scenario | What it exercises |
|---|---|
| `impulse` | unit impulse; exact impulse response |
| `constant` | constant input; DC gain check |
| `alternating` | alternating +max/−min; worst-case sign flips |
| `saturation` | constant +max; positive saturation path |
| `near_sat` | values near the saturation boundary |
| `random` | uniform random input, always-ready |
| `backpressure` | random input, bursty ready (3-on/2-off then random) |
| `reset_stream` | mid-stream synchronous reset at cycle 60 |
| `decim3` | decimating mode, period 3, always-ready |
| `decim3_bp` | decimating mode, period 3, bursty ready |
| `taps7_random` | TAPS=7 random input (different pipeline depth) |
| `taps7_impulse` | TAPS=7 impulse |
| `lpf_impulse` | Hamming low-pass (16 taps, fc=0.25) impulse |
| `lpf_random` | Hamming LPF random input, bursty ready (saturation boundary) |
| `wrap_saturation` | SAT=0 wraparound, +max input |
| `wrap_constant` | SAT=0 wraparound, constant input |

Regenerate everything:

```bash
python3 scripts/ref_model.py --list          # list scenarios
python3 scripts/ref_model.py --scenario all  # regenerate all tapes + expected
```

Each scenario produces `<name>.tape` (cycles of `rst_n s_valid s_data m_ready`),
`<name>.exp` (expected output samples) and `<name>.hex` (coefficient ROM in
`$readmemh` format, tap 0 first). A shared `sim/vectors/cfg_defs.svh` header
captures the exact configuration of the last generated scenario; the harness
regenerates it per scenario before every compile (see Troubleshooting).

## 7. Running the tests by hand

```bash
# 1) generate a scenario's vectors (this also rewrites cfg_defs.svh)
python3 scripts/ref_model.py --scenario backpressure --outdir sim/vectors

# 2) compile the self-checking testbench
iverilog -g2012 -Isim/vectors -o sim/results/tb_fir.vvp \
  sim/tb_fir.sv rtl/fir_arith.sv rtl/fir_control.sv rtl/fir_filter.sv

# 3) run it
vvp sim/results/tb_fir.vvp +tape=sim/vectors/backpressure.tape +exp=sim/vectors/backpressure.exp
# -> RESULT: PASS

# latency/throughput
make perf

# synthesis (needs a modern yosys)
YOSYS=/tmp/oss/oss-cad-suite/bin/yosys make synth
```

The testbench checks, for every scenario: every output sample against the
reference model, no lost/duplicated samples (`pushed == accepted` for
non-decimating scenarios), correct ordering, `m_valid` low during reset, and
correct samples around mid-stream resets.

## 8. Examples

See `examples/` for:

- `examples/run_impulse.sh` — single-scenario command line, prints the first
  impulse-response taps and the pass/fail verdict;
- `examples/example_impulse.txt` — recorded output of that run;
- `examples/example_synth.txt` — recorded Yosys synthesis statistics.

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ERROR: Can't open script file synth/synth_flow.ys` | run from the project root, or the file is missing (`make clean` does not delete it) |
| `syntax error, unexpected TOK_ID` in Yosys | your yosys is too old (apt 0.9). Install oss-cad-suite ≥ 0.4x (see section 2) |
| Scenario fails with `got N outputs, expected M` when run by hand | stale `cfg_defs.svh` — always regenerate the scenario vectors before compiling (step 1 of section 7) |
| `yosys` not found in `run_all.py` | `synth` is reported as FAIL/skipped; set `YOSYS=/path/to/yosys` |
| Perf test reports wrong latency expectation | `CFG_LAT` is derived from `TAPS`; re-run `python3 scripts/ref_model.py --scenario random` to refresh the header |

## 10. Known limitations

- Locked handshake (no skid buffer): if `m_ready` is low while `m_valid` is
  high, the **input** is stalled too. This is by design (no loss), but it is
  not a fully independent AXI-Stream slave; a decoupled design would add a
  FIFO.
- Coefficients are compile-time constants (packed parameter or `$readmemh`
  ROM); there is no runtime coefficient reconfiguration port.
- The multiplier array is fully parallel: one 32×32→32 multiplier per tap.
  A serial or time-multiplexed MAC datapath would trade throughput for area.
- The Yosys flow reports technology-mapped statistics (gates/FFs) but does
  not run ABC gate optimization (too slow with 16 parallel 36-bit
  multipliers) and has no place-and-route; see `docs/verification.md`.

## 11. License / provenance

Independent implementation for the "FPGA DSP Core — Fixed-Point Streaming FIR
Accelerator" engineering contract. No third-party RTL is included.
