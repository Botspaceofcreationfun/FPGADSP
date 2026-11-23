//============================================================================
// fir_arith.sv - Fixed-point arithmetic datapath of the streaming FIR filter
//============================================================================
//
// Module role: arithmetic only (no handshake, no streaming interface).
//   - Delay line (sample shift register).
//   - Coefficient store (packed parameter, or $readmemh init file).
//   - Parallel multiplier array (one per tap).
//   - Pipelined binary adder tree.
//   - Output requantisation: optional rounding, optional saturation.
//
// Fixed-point formats (defaults):
//   sample      : S1.15   -> DW bits  (1 sign bit, DW-1 fractional bits)
//   coefficient : S1.15   -> CW bits  (1 sign bit, CW-1 fractional bits)
//   product     : S2.30   -> PW = DW + CW bits  (exact, never truncated)
//   accumulator : S(2+D).30 -> ACCW = PW + D bits, D = ceil(log2(TAPS))
//   output      : S1.15   -> DW bits
//
// Why ACCW = PW + D is sufficient (no overflow, exact for every partial sum):
//   |product| <= (2^(DW-1)-1)(2^(CW-1)-1) < 2^(PW-2).
//   Any partial sum of k <= TAPS products has |sum| < 2^(PW-2) * k <= 2^(PW-2+D).
//   A signed ACCW-bit value holds |x| < 2^(ACCW-1) = 2^(PW+D-1), so every
//   partial sum (and every pairwise add inside the tree) is exact.
//
// Pipeline (one clock per stage):
//   cycle c        : sample accepted (shift_en=1) -> delay line updates
//   cycle c+1      : products of the new delay-line state are registered
//   cycle c+2..c+D+1 : adder-tree levels 1..D are registered (level D = the
//                    oldest in-flight sample's window)
//   cycle c+D+1    : the wrapper captures y_q (combinational from tree[D]),
//                    so the measured end-to-end latency from acceptance to a
//                    valid output is D+1 cycles; the tree is fully gated by
//                    shift_en, so it freezes in lockstep with the position
//                    pipeline under backpressure.
//
// All widths are explicit; every add/multiply is done with $signed() operands
// so sign extension is deterministic. Coefficient tap 0 lives in the MSBs of
// the packed COEFFS parameter: tap i is bits [((TAPS-1-i)*CW) +: CW].
//============================================================================

module fir_arith #(
  parameter int                TAPS          = 16,
  parameter int                DW            = 16,  // sample width (S1.(DW-1))
  parameter int                CW            = 16,  // coefficient width (S1.(CW-1))
  parameter int                FRAC_IN       = 15,  // sample fractional bits
  parameter int                FRAC_CO       = 15,  // coefficient fractional bits
  parameter int                FRAC_OUT      = 15,  // output fractional bits
  parameter int                ROUND         = 1,   // 0 = truncate, 1 = round half up
  parameter int                SAT           = 1,   // 1 = saturate, 0 = wraparound
  parameter [TAPS*CW-1:0]      COEFFS        = {16'h0800,16'h0800,16'h0800,16'h0800,
                                                16'h0800,16'h0800,16'h0800,16'h0800,
                                                16'h0800,16'h0800,16'h0800,16'h0800,
                                                16'h0800,16'h0800,16'h0800,16'h0800},
  parameter                    USE_COEFF_FILE = 0   // 1 -> load from $readmemh file
)(
  input  logic               clk,
  input  logic               rst_n,
  input  logic               shift_en,               // 1: accept xin into delay line
  input  logic signed [DW-1:0] xin,
  output logic signed [DW-1:0] y_q                    // requantised output (combinational)
);

  localparam int PW   = DW + CW;                            // product width (exact)
  localparam int D    = (TAPS <= 1) ? 0 : $clog2(TAPS);     // tree depth
  localparam int ACCW = PW + D;                             // accumulator width
  localparam int FD   = FRAC_IN + FRAC_CO - FRAC_OUT;       // frac bits dropped on output
  localparam int EXT  = ACCW + 1 - DW - FD;                 // guard bits above the DW-bit result

  //----------------------------------------------------------------------------
  // Delay line
  //----------------------------------------------------------------------------
  logic signed [DW-1:0] dl [TAPS];

  //----------------------------------------------------------------------------
  // Coefficients: packed parameter slice, or external hex file (see README)
  //----------------------------------------------------------------------------
  logic signed [CW-1:0] coeff [TAPS];

`ifdef FIR_COEFF_FILE
  initial $readmemh(`FIR_COEFF_FILE, coeff);
`else
  initial begin
    for (int i = 0; i < TAPS; i++)
      coeff[i] = $signed(COEFFS[((TAPS-1-i)*CW) +: CW]);   // tap 0 in MSBs
  end
`endif

  //----------------------------------------------------------------------------
  // Pipelined multiply-accumulate tree
  // tree[s][i]: level s, node i; stored at full ACCW width (sign-extended).
  //
  // The tree has D+1 stages (levels 0..D) so that tree[D] holds the window
  // of the sample at pre-accept position CHAIN-2 = D (the sample that enters
  // the END position when this cycle's accept fires).  The capture (which
  // uses the pre-accept view, see fir_control) loads tree[D] -- exactly the
  // entering sample's window, matching the reference model's
  // y(s_k) = FIR(delay line after s_k's acceptance).  All registers advance
  // ONLY on shift_en, in lockstep with the position register p, so stalls
  // freeze the whole datapath and the capture always reads the due sample's
  // window (an ungated tree would keep advancing through newer windows and
  // the capture would load a stale value after backpressure).
  //----------------------------------------------------------------------------
  localparam int TREE = D + 1;                       // == CHAIN-1 in fir_control
  logic signed [ACCW-1:0] tree [TREE][TAPS];

  // Number of live nodes at tree level s (handles non-power-of-two TAPS).
  function automatic int nin_at(input int s);
    nin_at = (TAPS + (1 << (s-1)) - 1) >> (s-1);
  endfunction

  // Post-shift delay line (combinational): what the delay line will contain
  // after the current sample is accepted.
  logic signed [DW-1:0] dl_next [TAPS];
  always_comb begin
    dl_next[0] = xin;
    for (int i = 1; i < TAPS; i++) dl_next[i] = dl[i-1];
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (int i = 0; i < TAPS; i++) dl[i] <= '0;
      for (int s = 0; s < TREE; s++)
        for (int i = 0; i < TAPS; i++) tree[s][i] <= '0;
    end else if (shift_en) begin
      for (int i = 0; i < TAPS; i++) dl[i] <= dl_next[i];
      // products (registered, tree level 0) of the POST-shift delay line
      for (int i = 0; i < TAPS; i++)
        tree[0][i] <= $signed(dl_next[i]) * $signed(coeff[i]);
      // balanced binary adder tree
      for (int s = 1; s < TREE; s++) begin
        for (int i = 0; i < TAPS; i++) begin
          if      (2*i+1 < nin_at(s)) tree[s][i] <= $signed(tree[s-1][2*i])   + $signed(tree[s-1][2*i+1]);
          else if (2*i   < nin_at(s)) tree[s][i] <= $signed(tree[s-1][2*i]);
          else                        tree[s][i] <= '0;
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Output requantisation: round (optional) then truncate, saturate (optional)
  //----------------------------------------------------------------------------
  logic signed [ACCW:0] acc_ext;      // sign-extended final accumulator
  logic signed [ACCW:0] rnd;          // accumulator + rounding bias (ACCW+1 bits)
  logic signed [ACCW:0] y_pre;        // post-shift value (floor / arithmetic shift)

  assign acc_ext = $signed(tree[TREE-1][0]);
  // 1 in bit (FD-1): round-half-up bias (ties round towards +infinity)
  function automatic [ACCW:0] round_bias(input int fd);
    round_bias = '0;
    if (fd > 0) round_bias[fd-1] = 1'b1;
  endfunction

  // Arithmetic (floor) right shift: matches Python's `>>` on signed ints, so
  // the RTL and the reference model agree for negative accumulators too.
  assign y_pre = rnd >>> FD;

  always_comb begin
    if (ROUND && FD > 0) rnd = acc_ext + $signed(round_bias(FD));
    else                 rnd = acc_ext;

    if (!SAT) begin
      // wraparound: keep the low DW bits (explicit two's-complement truncation)
      y_q = rnd[DW+FD-1 : FD];
    end else if (y_pre > $signed({{(ACCW+1-DW){1'b0}}, {(DW-1){1'b1}}})) begin
      // positive overflow: saturate to +max = 2^(DW-1)-1
      y_q = {1'b0, {(DW-1){1'b1}}};
    end else if (y_pre < $signed({1'b1, {(ACCW-DW){1'b1}}, {(DW-1){1'b0}}})) begin
      // negative overflow: saturate to -min = -2^(DW-1)
      y_q = {1'b1, {(DW-1){1'b0}}};
    end else begin
      // in range: low DW bits of the shifted value
      y_q = y_pre[DW-1:0];
    end
  end

endmodule
