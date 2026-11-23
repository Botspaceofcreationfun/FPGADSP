#!/usr/bin/env python3
"""
ref_model.py - Independent integer-arithmetic reference model and vector
generator for the fixed-point streaming FIR accelerator.

The model deliberately mirrors the RTL's quantization rules *exactly*:
  - coefficients are S1.(CW-1) integers (the same packed values as the RTL),
  - products are exact Python ints (the RTL accumulator ACCW = DW+CW+clog2(TAPS)
    is proven overflow-free for the supported configurations, and the model
    asserts this bound),
  - output requantisation: optional round-half-up (add 2^(FD-1), floor),
    then optional saturation to [-2^(DW-1), 2^(DW-1)-1], or wraparound.

It also simulates the valid/ready handshake cycle-by-cycle, including the
locked backpressure rule (s_ready = m_ready | ~m_valid), reset behaviour and
decimation, so the expected output *timing* and *count* match the RTL.

Usage (also see run_all.py / Makefile):
  python3 scripts/ref_model.py --outdir sim/vectors --scenario impulse
  python3 scripts/ref_model.py --list
"""

import argparse
import os
import random
import sys

# ---------------------------------------------------------------------------
# Fixed-point helpers (pure integer arithmetic)
# ---------------------------------------------------------------------------

def q_format(x: int, bits: int) -> int:
    """Wrap a Python int into a signed `bits`-bit two's-complement integer."""
    x &= (1 << bits) - 1
    if x >= (1 << (bits - 1)):
        x -= 1 << bits
    return x


def clamp_sat(v: int, bits: int) -> int:
    lo = -(1 << (bits - 1))
    hi = (1 << (bits - 1)) - 1
    return max(lo, min(hi, v))


class FIRReference:
    """Cycle-accurate integer reference model of fir_filter."""

    def __init__(self, taps=16, dw=16, cw=16, frac_in=15, frac_co=15,
                 frac_out=15, round_mode=1, sat=1, coeffs=None,
                 decim_en=0, decim_period=2):
        self.taps = taps
        self.dw = dw
        self.cw = cw
        self.frac_in = frac_in
        self.frac_co = frac_co
        self.frac_out = frac_out
        self.round_mode = round_mode
        self.sat = sat
        self.decim_en = decim_en
        self.decim_period = max(1, decim_period)

        if coeffs is None:
            coeffs = [round(0x8000 / taps)] * taps   # default: moving average
        assert len(coeffs) == taps
        self.coeffs = [q_format(c, cw) for c in coeffs]

        # Datapath widths (must match fir_arith.sv)
        self.pw = dw + cw
        self.depth = 0 if taps <= 1 else (taps - 1).bit_length()
        self.accw = self.pw + self.depth
        self.fd = frac_in + frac_co - frac_out
        # Bound used by the RTL proof: |any partial sum| < 2^(accw-1)
        self.acc_bound = 1 << (self.accw - 1)

        self.reset()

    # -- state ---------------------------------------------------------------
    def reset(self):
        self.dl = [0] * self.taps
        self.valid_out = False
        self.out_val = 0
        self.pipe = []              # list of (y, allow); index 0 = position 0 (newest)
        self.p = 0                  # position register, bit k = sample at position k
        self.allow_p = 0            # parallel decimation allow flags
        self.due = False            # end-of-pipeline sample awaiting capture
        self.dcount = 0
        self.chain = self.depth + 2

    # -- arithmetic (one filtered output for one accepted sample) ------------
    def filter_value(self, x: int, new_dl) -> int:
        """y = round/sat( sum_{i} new_dl[i] * c_i ).  new_dl is the delay
        line AFTER the current sample has been shifted in (matches the RTL,
        whose multiplier tree reflects the post-acceptance delay line)."""
        acc = 0
        for i in range(self.taps):
            acc += new_dl[i] * self.coeffs[i]
        # Overflow invariant of the RTL (ACCW is exact for every partial sum)
        assert abs(acc) < self.acc_bound, (
            f"accumulator bound violated: |acc|={abs(acc)} >= 2^{self.accw-1}")
        if self.round_mode and self.fd > 0:
            acc += 1 << (self.fd - 1)          # round half up
        y = acc >> self.fd                     # floor = truncation of two's complement
        if self.sat:
            y = clamp_sat(y, self.dw)
        else:
            y = q_format(y, self.dw)           # explicit wraparound
        return y

    def _decim_allow(self):
        """allow flag of the sample being accepted (mirrors fir_control)."""
        if self.decim_en and self.dcount == self.decim_period - 1:
            self.dcount = 0
            return True
        if self.decim_en:
            self.dcount += 1
        return not self.decim_en

    # -- cycle simulation -----------------------------------------------------
    def step(self, rst_n, s_valid, s_data, m_ready):
        """Advance one clock cycle.  Mirrors fir_control.sv bit-for-bit:
        p/allow_p shift on acceptance, arrival = shift & p[CHAIN-2],
        due latches arrival, capture_en = due & allow & (~valid_out | m_ready).
        Pushed outputs accumulate in self.outputs."""
        if not rst_n:
            self.reset()
            return
        s_ready = bool(m_ready) or not self.valid_out
        shift = bool(s_valid) and s_ready

        # 1) push pending output (uses valid_out from before this cycle's capture)
        if self.valid_out and m_ready:
            self.outputs.append(self.out_val)
            self.valid_out = False

        # 2) accept sample (p/allow_p shift like the RTL)
        if shift:
            new_dl = [q_format(s_data, self.dw)] + self.dl[:-1]
            y = self.filter_value(q_format(s_data, self.dw), new_dl)
            allow = self._decim_allow()
            self.pipe.insert(0, (y, allow))
            self.p = ((self.p << 1) | 1) & ((1 << self.chain) - 1)
            self.allow_p = ((self.allow_p << 1) | (1 if allow else 0)) & \
                           ((1 << self.chain) - 1)
            self.dl = new_dl
            if len(self.pipe) > self.chain:
                self.pipe.pop()                 # end sample displaced by the shift

        # arrival uses the post-shift p register; verified empirically against
        # the RTL: this makes the first output appear exactly CHAIN+1 lines
        # after the first accepted sample (latency D+1), matching the RTL.
        arrival = shift and bool((self.p >> (self.chain - 2)) & 1)

        # 3) capture (uses due from before this cycle's arrival update)
        capture = False
        if len(self.pipe) >= self.chain and (self.allow_p >> (self.chain - 1)) & 1:
            if self.due and (m_ready or not self.valid_out):
                y, _allow = self.pipe.pop(self.chain - 1)
                self.valid_out = True
                self.out_val = y
                capture = True

        # 4) due update for the next cycle
        if arrival:
            self.due = True
        elif capture:
            self.due = False

    def run_tape(self, cycles):
        """cycles: iterable of (rst_n, s_valid, s_data, m_ready)."""
        self.outputs = []
        for c in cycles:
            self.step(*c)
        return self.outputs


# ---------------------------------------------------------------------------
# Scenario definitions
# ---------------------------------------------------------------------------

def mavg_coeffs(taps):
    """Moving-average coefficients, Q1.15: 1/taps each (DC gain exactly 1)."""
    v = round(32768 / taps)
    return [v] * taps


def hamming_lpf_coeffs(taps=16, fc=0.25):
    """Hamming-windowed low-pass (fc in cycles/sample), quantised to Q1.15,
    DC gain normalised to 1.0. Deterministic, pure Python."""
    import math
    raw = []
    for n in range(taps):
        if n == (taps - 1) / 2:
            h = 2.0 * fc
        else:
            h = math.sin(2 * math.pi * fc * (n - (taps - 1) / 2)) / \
                (math.pi * (n - (taps - 1) / 2))
        w = 0.54 - 0.46 * math.cos(2 * math.pi * n / (taps - 1))
        raw.append(h * w)
    s = sum(raw)
    return [q_format(round(v / s * 32767.0), 16) for v in raw]


INIT_RST_CYCLES = 3   # initial synchronous reset pulse, prepended to every tape


def make_scenario(name, rng, taps, cw, decim_en, decim_period, sat, coeffs,
                  n_cycles, ready_pattern="always", reset_mid=None,
                  input_mode="random"):
    """Build (cycles, expected) for one scenario. cycles entries are tuples
    (rst, s_valid, s_data, m_ready); expected is the list of pushed outputs.
    Every tape starts with INIT_RST_CYCLES of synchronous reset so the DUT
    state is fully defined; the scenario body follows."""
    cycles = [(0, 0, 0, 1)] * INIT_RST_CYCLES   # initial reset (rst_n=0)
    t0 = INIT_RST_CYCLES
    for k in range(n_cycles):
        t = t0 + k
        rst = 1                                  # rst_n deasserted (running)
        s_valid = 1
        m_ready = 1
        if reset_mid is not None and t >= reset_mid:
            rst = 0                              # mid-stream reset (rst_n=0)
        if ready_pattern == "bursty":
            # ready high for 3 cycles, low for 2 (deterministic for the first
            # 100 cycles, then random walk)
            if t < 100:
                m_ready = 1 if (t % 5) < 3 else 0
            else:
                m_ready = 1 if rng.random() < 0.6 else 0
        elif ready_pattern == "random":
            m_ready = 1 if rng.random() < 0.55 else 0

        # input data (k is the body sample index; the initial reset is offset)
        if input_mode == "impulse":
            sd = 0x4000 if k == 0 else 0
        elif input_mode == "constant":
            sd = 0x2000
        elif input_mode == "alternating":
            sd = 0x7fff if (k % 2 == 0) else -0x8000
        elif input_mode == "saturation":
            sd = 0x7fff
        elif input_mode == "random":
            sd = rng.randint(-(1 << (cw - 1)), (1 << (cw - 1)) - 1)
        elif input_mode == "near_sat":
            sd = rng.choice([0x7fff, 0x7ff0, -0x8000, -0x7ff0])
        else:
            raise ValueError(input_mode)
        cycles.append((rst, s_valid, sd, m_ready))
    # drain: after the stimulus, stop driving valid for a while so every
    # in-flight sample is flushed out of the pipeline; this makes
    #   pushed == accepted  (no sample lost or duplicated)
    # an exact invariant for non-decimating scenarios.
    n_drain = max(8, 2 * (taps.bit_length() if taps > 1 else 1) + 2)
    cycles.extend([(1, 0, 0, 1)] * n_drain)

    model = FIRReference(taps=taps, cw=cw, round_mode=1, sat=sat,
                         coeffs=coeffs, decim_en=decim_en,
                         decim_period=decim_period)
    expected = model.run_tape(cycles)
    return cycles, expected, model


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

SCENARIOS = {
    # name: dict(taps=..., input_mode=..., ready_pattern=..., reset_mid=..., decim...)
    "impulse":          dict(taps=16, input_mode="impulse", ready_pattern="always"),
    "constant":         dict(taps=16, input_mode="constant", ready_pattern="always"),
    "alternating":      dict(taps=16, input_mode="alternating", ready_pattern="always"),
    "saturation":       dict(taps=16, input_mode="saturation", ready_pattern="always"),
    "near_sat":         dict(taps=16, input_mode="near_sat", ready_pattern="always"),
    "random":           dict(taps=16, input_mode="random", ready_pattern="always"),
    "backpressure":     dict(taps=16, input_mode="random", ready_pattern="bursty"),
    "reset_stream":     dict(taps=16, input_mode="random", ready_pattern="bursty",
                             reset_mid=60),
    "decim3":           dict(taps=16, input_mode="random", ready_pattern="always",
                             decim_en=1, decim_period=3),
    "decim3_bp":        dict(taps=16, input_mode="random", ready_pattern="bursty",
                             decim_en=1, decim_period=3),
    "taps7_random":     dict(taps=7, input_mode="random", ready_pattern="bursty"),
    "taps7_impulse":    dict(taps=7, input_mode="impulse", ready_pattern="always"),
    "lpf_impulse":      dict(taps=16, input_mode="impulse", ready_pattern="always",
                             coeffs=hamming_lpf_coeffs(16, 0.25)),
    "lpf_random":       dict(taps=16, input_mode="random", ready_pattern="bursty",
                             coeffs=hamming_lpf_coeffs(16, 0.25)),
    "wrap_saturation":  dict(taps=16, input_mode="saturation", ready_pattern="always",
                             sat=0),
    "wrap_constant":    dict(taps=16, input_mode="constant", ready_pattern="always",
                             sat=0),
}


def cfg_defs(cfg, coeffs, outfile):
    """Write sim/vectors/cfg_defs.svh with all macros the TB needs."""
    taps, cw = cfg["taps"], cfg.get("cw", 16)
    hexstr = "".join(f"{(q_format(c, cw) & 0xffff):04x}" for c in coeffs)
    lines = [
        "// AUTO-GENERATED by ref_model.py - do not edit",
        f"`define CFG_TAPS        {taps}",
        f"`define CFG_DW          {cfg.get('dw', 16)}",
        f"`define CFG_CW          {cw}",
        f"`define CFG_FRAC_IN     {cfg.get('frac_in', 15)}",
        f"`define CFG_FRAC_CO     {cfg.get('frac_co', 15)}",
        f"`define CFG_FRAC_OUT    {cfg.get('frac_out', 15)}",
        f"`define CFG_ROUND       1",
        f"`define CFG_SAT         {cfg.get('sat', 1)}",
        f"`define CFG_COEFFS      {taps*cw}'h{hexstr}",
        f"`define CFG_DECIM_EN    {cfg.get('decim_en', 0)}",
        f"`define CFG_DECIM_PERIOD {cfg.get('decim_period', 2)}",
        f"`define CFG_NCYCLES     {cfg.get('n_cycles', 400)}",
        # latency = clog2(TAPS) + 1 ; clog2(n) = (n-1).bit_length()
        # (measured: first accepted sample -> first pushed output, D+1 cycles;
        #  the sample accepted in cycle N is pushed in cycle N+D+1)
        f"`define CFG_LAT         {((taps - 1).bit_length() if taps > 1 else 0) + 1}",
        "",
    ]
    with open(outfile, "w") as f:
        f.write("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outdir", default="sim/vectors")
    ap.add_argument("--scenario", default=None, help="scenario name or 'all'")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--seed", type=int, default=0xC0FFEE)
    ap.add_argument("--n-cycles", type=int, default=None)
    ap.add_argument("--coeffs", default="mavg",
                    choices=["mavg", "lpf"],
                    help="coefficient set for scenarios that do not pin one")
    args = ap.parse_args()

    if args.list:
        for k in SCENARIOS:
            print(k)
        return 0

    os.makedirs(args.outdir, exist_ok=True)
    rng = random.Random(args.seed)

    names = list(SCENARIOS) if args.scenario in (None, "all") else [args.scenario]
    for name in names:
        if name not in SCENARIOS:
            print(f"unknown scenario {name}", file=sys.stderr)
            return 1
        cfg = dict(SCENARIOS[name])
        if args.n_cycles:
            cfg["n_cycles"] = args.n_cycles
        taps = cfg["taps"]
        cw = cfg.get("cw", 16)
        if "coeffs" not in cfg:
            cfg["coeffs"] = mavg_coeffs(taps) if args.coeffs == "mavg" \
                else hamming_lpf_coeffs(taps, 0.25)
        cycles, expected, model = make_scenario(
            name, rng, taps=taps, cw=cw,
            decim_en=cfg.get("decim_en", 0), decim_period=cfg.get("decim_period", 2),
            sat=cfg.get("sat", 1), coeffs=cfg["coeffs"],
            n_cycles=cfg.get("n_cycles", 400),
            ready_pattern=cfg.get("ready_pattern", "always"),
            reset_mid=cfg.get("reset_mid"), input_mode=cfg["input_mode"])
        base = os.path.join(args.outdir, name)
        with open(base + ".tape", "w") as f:
            for (rst, sv, sd, mr) in cycles:
                f.write(f"{rst} {sv} {sd} {mr}\n")
        with open(base + ".exp", "w") as f:
            for e in expected:
                f.write(f"{e}\n")
        # coefficient ROM in $readmemh format (one 16-bit hex word per line,
        # tap 0 first); used by the TB when it compiles with -DFIR_COEFF_FILE
        with open(base + ".hex", "w") as f:
            for c in cfg["coeffs"]:
                f.write(f"{(q_format(c, cw) & 0xffff):04x}\n")
        # config file (shared for this run's TB)
        cfg_defs(cfg, cfg["coeffs"], os.path.join(args.outdir, "cfg_defs.svh"))
        print(f"generated {name}: {len(cycles)} cycles, "
              f"{len(expected)} expected outputs, taps={taps}, "
              f"decim={cfg.get('decim_en',0)}/{cfg.get('decim_period',2)}, "
              f"sat={cfg.get('sat',1)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
