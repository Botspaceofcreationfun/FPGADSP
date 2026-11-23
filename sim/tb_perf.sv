//============================================================================
// tb_perf.sv - Latency and throughput measurement for the FIR filter
//============================================================================
//
// Measures (and self-checks):
//   - pipeline latency in clock cycles: first acceptance cycle -> first
//     pushed output cycle (expected CFG_LAT = ceil(log2(TAPS)) + 1);
//   - steady-state throughput: pushes per cycle over a fixed window with
//     s_valid = m_ready = 1 (expected 1 sample/cycle, or 1/DECIM_PERIOD
//     when decimation is enabled).
//============================================================================

`timescale 1ns/1ps
`include "cfg_defs.svh"

module tb_perf;

  localparam int TAPS        = `CFG_TAPS;
  localparam int DW          = `CFG_DW;
  localparam int CW          = `CFG_CW;
  localparam int DECIM_EN    = `CFG_DECIM_EN;
  localparam int DECIM_PERIOD= `CFG_DECIM_PERIOD;
  localparam int LAT         = `CFG_LAT;
  localparam int RESET_CYCLES = 3;
  localparam int TOTAL_CYCLES = 400;

  logic clk = 0;
  always #5 clk = ~clk;

  logic rst_n, s_valid, m_ready;
  logic signed [DW-1:0] s_data;
  logic s_ready, m_valid;
  logic signed [DW-1:0] m_data;

  fir_filter #(
    .TAPS(TAPS), .DW(DW), .CW(CW),
    .FRAC_IN(`CFG_FRAC_IN), .FRAC_CO(`CFG_FRAC_CO), .FRAC_OUT(`CFG_FRAC_OUT),
    .ROUND(`CFG_ROUND), .SAT(`CFG_SAT), .COEFFS(`CFG_COEFFS),
    .DECIM_EN(DECIM_EN), .DECIM_PERIOD(DECIM_PERIOD)
  ) dut (
    .clk(clk), .rst_n(rst_n), .s_valid(s_valid), .s_ready(s_ready),
    .s_data(s_data), .m_valid(m_valid), .m_ready(m_ready), .m_data(m_data)
  );

  int cycle_idx = 0;
  int first_accept = -1, first_push = -1;
  int pushes = 0, window_start = 100, window_end = 400, window_pushes = 0;
  int expected_latency = 0;
  int errors = 0;

  initial begin
    rst_n = 0; s_valid = 0; s_data = 0; m_ready = 1;
    // reset phase
    repeat (RESET_CYCLES) begin
      @(posedge clk); #1; cycle_idx++;
    end
    rst_n = 1; s_valid = 1; m_ready = 1;
    s_data = 16'sd100;

    while (cycle_idx < TOTAL_CYCLES) begin
      s_data = $signed(s_data) + 16'sd1;          // slow ramp (no overflow)
      @(posedge clk); #1;
      cycle_idx++;
      if (s_valid && s_ready && first_accept < 0) first_accept = cycle_idx;
      if (m_valid && m_ready) begin
        if (first_push < 0) first_push = cycle_idx;
        pushes++;
        if (cycle_idx >= window_start && cycle_idx < window_end) window_pushes++;
      end
    end

    //----------------------------------------------------------------------
    // Checks
    //----------------------------------------------------------------------
    // With decimation the first accepted sample (s0) is not an output sample:
    // the first output corresponds to the (DECIM_PERIOD-1)-th accepted sample,
    // so the acceptance->push latency measured from the FIRST accepted sample
    // includes the decimation offset.
    if (DECIM_EN == 0)
      expected_latency = LAT;
    else
      expected_latency = LAT + DECIM_PERIOD - 1;
    $display("INFO: first_accept_cycle=%0d first_push_cycle=%0d latency=%0d (expect %0d)",
             first_accept, first_push, first_push - first_accept, expected_latency);
    if (first_push - first_accept != expected_latency) begin
      $display("FAIL: measured latency %0d != expected %0d", first_push - first_accept, expected_latency);
      errors++;
    end

    if (DECIM_EN == 0) begin
      $display("INFO: pushes=%0d over %0d cycles (expect %0d)", pushes, TOTAL_CYCLES, TOTAL_CYCLES - RESET_CYCLES - LAT);
      if (window_pushes != (window_end - window_start)) begin
        $display("FAIL: throughput window pushes=%0d, expected %0d (not 1/cycle)",
                 window_pushes, window_end - window_start);
        errors++;
      end else
        $display("PASS: throughput = 1 sample/cycle in steady state");
    end else begin
      $display("INFO: decimation pushes=%0d (expect ~1/%0d per cycle)",
               pushes, DECIM_PERIOD);
      // window length 300 cycles -> expect exactly window/DECIM_PERIOD pushes
      // (aligned because the decimation counter restarts at the reset)
      if (window_pushes != (window_end - window_start) / DECIM_PERIOD) begin
        $display("FAIL: decimated throughput window_pushes=%0d, expected %0d",
                 window_pushes, (window_end - window_start) / DECIM_PERIOD);
        errors++;
      end else
        $display("PASS: decimated throughput = 1/%0d sample/cycle", DECIM_PERIOD);
    end

    if (errors == 0) $display("RESULT: PASS");
    else             $display("RESULT: FAIL");
    $finish;
  end

  initial begin
    #(2_000_000); $display("ERROR: timeout"); $finish;
  end

endmodule
