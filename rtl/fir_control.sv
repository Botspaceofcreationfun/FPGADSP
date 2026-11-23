//============================================================================
// fir_control.sv - Handshake, pipeline-valid tracking and decimation control
//============================================================================
//
// Implements a locked (no skid buffer) streaming handshake:
//
//   s_ready = m_ready OR ~m_valid     (upstream stalls when an output is pending)
//
// Because the input is stalled whenever the output is stalled, the pipeline
// freezes in lockstep: no sample is ever lost or duplicated.
//
// Output-stage event model (mirrors the reference model exactly):
//   - `p` is a CHAIN-deep position shift register that advances only on an
//     accepted sample (shift_en), exactly tracking each sample through the
//     arithmetic datapath.  p[CHAIN-1] high means a sample is at the end of
//     the pipeline.
//   - `arrival` pulses when this cycle's shift moves a sample into the end
//     position:  arrival = shift_en & p[CHAIN-2].
//   - `due` latches the arrival: the end sample's quantised output y_q is
//     ready to be captured (the arithmetic latency is exactly CHAIN cycles
//     from acceptance, so arrival and y_q readiness align).
//   - `capture_en` fires when a NEW result must be loaded into the output
//     register: the end sample is due, it is an output sample (allow), and
//     the output slot is free (no pending output) or is being consumed this
//     same cycle (m_ready).  A capture without a fresh arrival clears `due`,
//     guaranteeing exactly one capture per output sample even at stream end.
//
//   capture_en = due & allow_p[CHAIN-1] & ( ~valid_out | m_ready )
//
// Decimation: when DECIM_EN=1 the filter still accepts and filters every
// input sample, but m_valid is asserted only for one out of every
// DECIM_PERIOD accepted samples.  The per-sample `allow` flag travels down
// the pipeline beside `p`, so backpressure cannot misalign it.
//
// Reset: synchronous, clears valid_out, due, the pipeline and the decimation
// counter.  After reset the first output appears D+1 cycles after the first
// accepted sample.
//============================================================================

module fir_control #(
  parameter int TAPS         = 16,
  parameter bit DECIM_EN     = 0,
  parameter int DECIM_PERIOD = 2
)(
  input  logic clk,
  input  logic rst_n,
  input  logic s_valid,
  input  logic m_ready,
  output logic s_ready,
  output logic m_valid,
  output logic shift_en,     // 1: sample accepted (s_valid & s_ready)
  output logic capture_en    // 1: load arithmetic output into the output register
);

  localparam int D     = (TAPS <= 1) ? 0 : $clog2(TAPS);
  localparam int CHAIN = D + 2;                // positions between acceptance and capture

  logic valid_out;
  logic due;                                   // end-of-pipeline sample awaiting capture
  logic [CHAIN-1:0] p;                         // pipeline position markers
  logic [CHAIN-1:0] allow_p;                   // per-position decimation allow flag
  int               dcount;                    // decimation counter (0..PERIOD-1)

  logic arrival;

  assign s_ready    = m_ready || !valid_out;
  assign shift_en   = s_valid && s_ready;
  assign m_valid    = valid_out;
  // arrival: a sample has moved into position CHAIN-2 (the "next to enter
  // the end").  p[CHAIN-2] is read AFTER this cycle's shift by the model
  // (shift && post-shift p[CHAIN-2]); in the RTL, p is a registered shift
  // register, so the post-shift value of p[CHAIN-2] equals the pre-shift
  // value of p[CHAIN-3].  Reading pre-shift p[CHAIN-2] would fire one cycle
  // later and would accept a sample the model rejects at a stall boundary.
  assign arrival = shift_en && p[CHAIN-3];

  // Capture when the accept moving a sample into the END position fires.  The
  // sample entering the end is at pre-accept position CHAIN-2; the reference
  // model pops pipe[CHAIN-1] in the SAME step the sample enters the end, so a
  // register-accurate RTL must capture from the pre-accept view (allow_p
  // [CHAIN-2]) and load y_q = tree[CHAIN-2], which is exactly the entering
  // sample's window.  The capture is qualified with shift_en so that at the
  // stream end (drain, no more accepts) no extra sample is captured from the
  // frozen pre-accept position.  `due` is a latched arrival, so the capture
  // stays eligible across backpressure; a capture with no simultaneous
  // arrival clears `due`, preventing duplicates.
  assign capture_en = due && shift_en && allow_p[CHAIN-2] && (m_ready || !valid_out);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      valid_out <= 1'b0;
      due       <= 1'b0;
      p         <= '0;
      allow_p   <= '0;
      dcount    <= 0;
    end else begin
      if (shift_en) begin
        p[0] <= 1'b1;
        for (int k = 1; k < CHAIN; k++) begin
          p[k]       <= p[k-1];
          allow_p[k] <= allow_p[k-1];
        end
        if (DECIM_EN && (dcount == DECIM_PERIOD-1)) begin
          allow_p[0] <= 1'b1;
          dcount     <= 0;
        end else begin
          allow_p[0] <= DECIM_EN ? 1'b0 : 1'b1;
          dcount     <= DECIM_EN ? dcount + 1 : dcount;
        end
      end

      if (arrival)             due <= 1'b1;
      else if (capture_en)     due <= 1'b0;

      if (capture_en)          valid_out <= 1'b1;
      else if (m_valid && m_ready) valid_out <= 1'b0;
    end
  end

endmodule
