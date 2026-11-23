#!/usr/bin/env bash
# Single-scenario demonstration: impulse response of the 16-tap moving average.
set -euo pipefail
cd "$(dirname "$0")/.."
python3 scripts/ref_model.py --scenario impulse --outdir sim/vectors >/dev/null
iverilog -g2012 -Isim/vectors -o sim/results/tb_fir.vvp \
  sim/tb_fir.sv rtl/fir_arith.sv rtl/fir_control.sv rtl/fir_filter.sv
vvp sim/results/tb_fir.vvp +tape=sim/vectors/impulse.tape +exp=sim/vectors/impulse.exp
echo "--- first 6 impulse-response taps (S1.15, expected 2048 each for the MA) ---"
head -6 sim/vectors/impulse.exp
