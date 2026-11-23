# Makefile for the fixed-point streaming FIR accelerator
#
# Targets:
#   make          - generate vectors, run all tests (alias of `test`)
#   make test     - full end-to-end verification (reference model + iverilog TBs)
#   make gen      - regenerate all stimulus/expected-vector files
#   make synth    - run Yosys synthesis (if yosys is installed)
#   make perf     - run only the latency/throughput testbench
#   make clean    - remove build artifacts
#   make zip      - create a distributable archive of the project

PYTHON  ?= python3
IVERILOG?= iverilog
VVP     ?= vvp
ZIP     ?= zip

.PHONY: all test gen synth perf clean zip

all: test

test:
	$(PYTHON) scripts/run_all.py

gen:
	$(PYTHON) scripts/ref_model.py --scenario all

perf:
	$(PYTHON) scripts/ref_model.py --scenario random
	$(IVERILOG) -g2012 -Isim/vectors -o sim/results/tb_perf.vvp sim/tb_perf.sv \
	  rtl/fir_arith.sv rtl/fir_control.sv rtl/fir_filter.sv
	$(VVP) sim/results/tb_perf.vvp

synth:
	$(PYTHON) scripts/ref_model.py --scenario random
	-yosys -q -s synth/synth_flow.ys

clean:
	rm -rf sim/results sim/vectors docs/_tmp

zip:
	$(PYTHON) scripts/ref_model.py --scenario all >/dev/null
	rm -f fir_fixed_project.zip
	$(ZIP) -r fir_fixed_project.zip \
	  README.md Makefile \
	  rtl sim scripts docs examples synth \
	  -x "sim/results/*" -x "sim/vectors/*.tape" -x "sim/vectors/*.exp" -x "sim/vectors/*.hex"
	@echo "Created fir_fixed_project.zip"
