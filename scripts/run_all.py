#!/usr/bin/env python3
"""
run_all.py - End-to-end verification orchestrator for the FIR accelerator.

For every scenario it:
  1. generates the stimulus tape and expected outputs with the independent
     Python reference model (ref_model.py),
  2. writes the matching cfg_defs.svh parameter header,
  3. compiles the self-checking testbench with iverilog,
  4. runs it and checks "RESULT: PASS/FAIL".

It also runs the latency/throughput performance testbench (plain and
decimating configurations) and, if a yosys binary is available, a real
synthesis run with reported resource usage.

Run from the project root:   python3 scripts/run_all.py
"""

import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)

PY    = sys.executable
IVL  = shutil.which("iverilog") or "iverilog"
VVP  = shutil.which("vvp") or "vvp"
YOSYS = os.environ.get("YOSYS", shutil.which("yosys"))

RTL  = ["rtl/fir_arith.sv", "rtl/fir_control.sv", "rtl/fir_filter.sv"]
VDIR = "sim/vectors"
RDIR = "sim/results"

SCENARIOS = ["impulse", "constant", "alternating", "saturation", "near_sat",
             "random", "backpressure", "reset_stream",
             "decim3", "decim3_bp",
             "taps7_random", "taps7_impulse",
             "lpf_impulse", "lpf_random",
             "wrap_saturation", "wrap_constant"]

# Scenario -> extra iverilog defines (exercises the $readmemh coefficient
# initialization mechanism documented in the README)
COEFF_FILE_SCEN = "lpf_impulse"          # uses FIR_COEFF_FILE instead of COEFFS
PERF_CFGS = [("random",  False),        # 16-tap moving average, no decimation
             ("decim3",  True)]         # 16-tap moving average, /3 decimation


def run(cmd, **kw):
    print("  $ " + " ".join(cmd))
    p = subprocess.run(cmd, capture_output=True, text=True, **kw)
    return p


def generate(scenario):
    p = run([PY, "scripts/ref_model.py", "--scenario", scenario,
             "--outdir", VDIR])
    return p.returncode == 0


def compile_tb(tb, defines=None):
    os.makedirs(RDIR, exist_ok=True)
    cmd = [IVL, "-g2012", f"-I{VDIR}"] + (defines or []) + \
          ["-o", f"{RDIR}/{tb}.vvp", f"sim/{tb}.sv"] + RTL
    p = run(cmd)
    return p.returncode == 0, p


def run_vvp(tb, extra_args):
    p = run([VVP, f"{RDIR}/{tb}.vvp"] + extra_args)
    return p


def scenario_check(scenario):
    print(f"[scenario] {scenario}")
    if not generate(scenario):
        print(f"  FAIL: vector generation for {scenario}")
        return False
    defines = []
    if scenario == COEFF_FILE_SCEN:
        # coefficient ROM loaded from a hex file via $readmemh
        hexf = os.path.abspath(f"{VDIR}/{scenario}.hex")
        defines = [f'-DFIR_COEFF_FILE="{hexf}"']
    ok, p = compile_tb("tb_fir", defines)
    if not ok:
        print(p.stdout + p.stderr)
        return False
    q = run_vvp("tb_fir", [f"+tape={VDIR}/{scenario}.tape",
                           f"+exp={VDIR}/{scenario}.exp"])
    out = q.stdout + q.stderr
    print(out.strip())
    return "RESULT: PASS" in out


def perf_check(scenario, decim):
    print(f"[perf] cfg={scenario} decim={decim}")
    # the cfg_defs.svh for this scenario is already in place from scenario_check
    ok, p = compile_tb("tb_perf")
    if not ok:
        print(p.stdout + p.stderr)
        return False
    q = run_vvp("tb_perf", [])
    out = q.stdout + q.stderr
    print(out.strip())
    return "RESULT: PASS" in out


def synth_check():
    if not YOSYS:
        print("[synth] yosys not found - synthesis skipped (documented in "
              "verification report)")
        return None
    # per-scenario cfg (16-tap MA) is in sim/vectors/cfg_defs.svh; synth uses
    # the RTL defaults, so it does not depend on it.
    script = os.path.join(ROOT, "synth", "synth_flow.ys")
    # NB: no -q: the ===SYNTH_OK=== sentinel is a `log` message, which quiet
    # mode suppresses.
    p = run([YOSYS, "-s", script])
    out = p.stdout + p.stderr
    print(out.strip()[-4000:])
    ok = "===SYNTH_OK===" in out and p.returncode == 0
    return ok


def main():
    results = {}
    for s in SCENARIOS:
        results[s] = scenario_check(s)
    for s, decim in PERF_CFGS:
        # ensure the cfg matches the perf config
        generate(s)
        results[f"perf:{s}"] = perf_check(s, decim)
    # restore a default cfg for user convenience (16-tap MA, no decimation)
    generate("random")
    results["synth"] = synth_check()

    print("\n================ SUMMARY ================")
    npass = sum(1 for v in results.values() if v is True)
    nfail = sum(1 for v in results.values() if v is False)
    nskip = sum(1 for v in results.values() if v is None)
    for k, v in results.items():
        print(f"  {'PASS' if v is True else ('SKIP' if v is None else 'FAIL'):6s}  {k}")
    print(f"passed={npass} failed={nfail} skipped={nskip}")
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
