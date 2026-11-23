//============================================================================
// fir_filter.sv - Streaming fixed-point FIR filter (top-level wrapper)
//============================================================================
//
// Streaming interface (AXI-Stream-like valid/ready):
//   s_valid / s_ready : input sample handshake (s_data must be stable when
//                       s_valid is high)
//   m_valid / m_ready : output sample handshake (m_data is stable while
//                       m_valid is high; it is held under backpressure)
//
// Modules:
//   fir_arith    : delay line, multipliers, adder tree, round/saturate
//   fir_control  : handshake, pipeline-valid tracking, decimation
//   (this file)  : interface logic: output register + wiring
//
// Latency: D + 1 clock cycles (D = ceil(log2(TAPS))) from the cycle in which
// a sample is accepted to the cycle in which the corresponding output is
// valid (measured by tb_perf; the first output after reset appears D+1
// cycles after the first accepted sample).
// Throughput: one accepted sample and one output per cycle in steady state
// (both handshakes continuously ready); with decimation, one output per
// DECIM_PERIOD accepted samples.
//============================================================================

module fir_filter #(
  parameter int                TAPS           = 16,
  parameter int                DW             = 16,
  parameter int                CW             = 16,
  parameter int                FRAC_IN        = 15,
  parameter int                FRAC_CO        = 15,
  parameter int                FRAC_OUT       = 15,
  parameter int                ROUND          = 1,
  parameter int                SAT            = 1,
  parameter [TAPS*CW-1:0]      COEFFS         = {16'h0800,16'h0800,16'h0800,16'h0800,
                                                 16'h0800,16'h0800,16'h0800,16'h0800,
                                                 16'h0800,16'h0800,16'h0800,16'h0800,
                                                 16'h0800,16'h0800,16'h0800,16'h0800},
  parameter                    USE_COEFF_FILE = 0,
  parameter                    DECIM_EN       = 0,
  parameter int                DECIM_PERIOD   = 2
)(
  input  logic               clk,
  input  logic               rst_n,
  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [DW-1:0] s_data,
  output logic               m_valid,
  input  logic               m_ready,
  output logic signed [DW-1:0] m_data
);

  logic               shift_en;
  logic               capture_en;
  logic signed [DW-1:0] y_q;
  logic signed [DW-1:0] m_data_r;

  //--------------------------------------------------------------------------
  // Arithmetic datapath
  //--------------------------------------------------------------------------
  fir_arith #(
    .TAPS          (TAPS),
    .DW            (DW),
    .CW            (CW),
    .FRAC_IN       (FRAC_IN),
    .FRAC_CO       (FRAC_CO),
    .FRAC_OUT      (FRAC_OUT),
    .ROUND         (ROUND),
    .SAT           (SAT),
    .COEFFS        (COEFFS),
    .USE_COEFF_FILE(USE_COEFF_FILE)
  ) u_arith (
    .clk      (clk),
    .rst_n    (rst_n),
    .shift_en (shift_en),
    .xin      (s_data),
    .y_q      (y_q)
  );

  //--------------------------------------------------------------------------
  // Handshake / valid tracking / decimation
  //--------------------------------------------------------------------------
  fir_control #(
    .TAPS         (TAPS),
    .DECIM_EN     (DECIM_EN),
    .DECIM_PERIOD (DECIM_PERIOD)
  ) u_ctrl (
    .clk        (clk),
    .rst_n      (rst_n),
    .s_valid    (s_valid),
    .m_ready    (m_ready),
    .s_ready    (s_ready),
    .m_valid    (m_valid),
    .shift_en   (shift_en),
    .capture_en (capture_en)
  );

  //--------------------------------------------------------------------------
  // Output register: captures a new result only on capture_en; otherwise
  // holds its value, so m_data is stable while m_valid is asserted.
  //--------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n)      m_data_r <= '0;
    else if (capture_en) m_data_r <= y_q;
  end

  assign m_data = m_data_r;

endmodule
