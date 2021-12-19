module PositionEstimator(
  input        clock,
  input        reset,
  input        io_hall_u,
  input        io_hall_v,
  input        io_hall_w,
  input        io_qdec_inc,
  input        io_qdec_dec,
  output [8:0] io_theta_bits,
  output       io_theta_error,
  output       io_theta_uncertain
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
`endif // RANDOMIZE_REG_INIT
  reg  previousHall_u; // @[PositionEstimator.scala 31:31]
  reg  previousHall_v; // @[PositionEstimator.scala 31:31]
  reg  previousHall_w; // @[PositionEstimator.scala 31:31]
  wire  uvwTransition_hi_hi = previousHall_u ^ io_hall_u; // @[PositionEstimator.scala 32:44]
  wire  uvwTransition_hi_lo = previousHall_v ^ io_hall_v; // @[PositionEstimator.scala 32:72]
  wire  uvwTransition_lo = previousHall_w ^ io_hall_w; // @[PositionEstimator.scala 32:100]
  wire [2:0] uvwTransition = {uvwTransition_hi_hi,uvwTransition_hi_lo,uvwTransition_lo}; // @[Cat.scala 30:58]
  reg [8:0] bits; // @[PositionEstimator.scala 35:23]
  reg  error; // @[PositionEstimator.scala 36:24]
  reg  uncertain; // @[PositionEstimator.scala 37:28]
  wire  _T_9 = uvwTransition == 3'h4; // @[PositionEstimator.scala 47:29]
  wire  _T_11 = ~io_hall_w; // @[PositionEstimator.scala 47:59]
  wire  _T_14 = ~io_hall_v; // @[PositionEstimator.scala 50:52]
  wire  _T_17 = uvwTransition == 3'h2; // @[PositionEstimator.scala 53:35]
  wire  _T_22 = ~io_hall_u; // @[PositionEstimator.scala 56:52]
  wire  _T_25 = uvwTransition == 3'h1; // @[PositionEstimator.scala 59:35]
  wire [7:0] _GEN_0 = _T_25 & _T_22 & io_hall_v ? 8'h80 : 8'h0; // @[PositionEstimator.scala 62:77 PositionEstimator.scala 64:18 PositionEstimator.scala 66:18]
  wire  _GEN_1 = _T_25 & _T_22 & io_hall_v ? 1'h0 : 1'h1; // @[PositionEstimator.scala 62:77 PositionEstimator.scala 45:15 PositionEstimator.scala 67:19]
  wire [8:0] _GEN_2 = uvwTransition == 3'h1 & io_hall_u & _T_14 ? 9'h180 : {{1'd0}, _GEN_0}; // @[PositionEstimator.scala 59:77 PositionEstimator.scala 61:18]
  wire  _GEN_3 = uvwTransition == 3'h1 & io_hall_u & _T_14 ? 1'h0 : _GEN_1; // @[PositionEstimator.scala 59:77 PositionEstimator.scala 45:15]
  wire [8:0] _GEN_4 = _T_17 & ~io_hall_u & io_hall_w ? 9'hd5 : _GEN_2; // @[PositionEstimator.scala 56:77 PositionEstimator.scala 58:18]
  wire  _GEN_5 = _T_17 & ~io_hall_u & io_hall_w ? 1'h0 : _GEN_3; // @[PositionEstimator.scala 56:77 PositionEstimator.scala 45:15]
  wire [8:0] _GEN_6 = uvwTransition == 3'h2 & io_hall_u & _T_11 ? 9'h1d5 : _GEN_4; // @[PositionEstimator.scala 53:77 PositionEstimator.scala 55:18]
  wire  _GEN_7 = uvwTransition == 3'h2 & io_hall_u & _T_11 ? 1'h0 : _GEN_5; // @[PositionEstimator.scala 53:77 PositionEstimator.scala 45:15]
  wire [8:0] _GEN_8 = _T_9 & ~io_hall_v & io_hall_w ? 9'h12a : _GEN_6; // @[PositionEstimator.scala 50:77 PositionEstimator.scala 52:18]
  wire  _GEN_9 = _T_9 & ~io_hall_v & io_hall_w ? 1'h0 : _GEN_7; // @[PositionEstimator.scala 50:77 PositionEstimator.scala 45:15]
  wire  _GEN_11 = uvwTransition == 3'h4 & io_hall_v & ~io_hall_w ? 1'h0 : _GEN_9; // @[PositionEstimator.scala 47:71 PositionEstimator.scala 45:15]
  wire [2:0] _T_33 = {io_hall_u,io_hall_v,io_hall_w}; // @[Cat.scala 30:58]
  wire  _T_34 = 3'h0 == _T_33; // @[Conditional.scala 37:30]
  wire  _T_35 = 3'h1 == _T_33; // @[Conditional.scala 37:30]
  wire  _T_36 = 3'h2 == _T_33; // @[Conditional.scala 37:30]
  wire  _T_37 = 3'h3 == _T_33; // @[Conditional.scala 37:30]
  wire  _T_38 = 3'h4 == _T_33; // @[Conditional.scala 37:30]
  wire  _T_39 = 3'h5 == _T_33; // @[Conditional.scala 37:30]
  wire  _T_40 = 3'h6 == _T_33; // @[Conditional.scala 37:30]
  wire  _T_41 = 3'h7 == _T_33; // @[Conditional.scala 37:30]
  wire [8:0] _GEN_12 = _T_41 ? 9'h0 : bits; // @[Conditional.scala 39:67 PositionEstimator.scala 80:33 PositionEstimator.scala 35:23]
  wire [8:0] _GEN_13 = _T_40 ? 9'h0 : _GEN_12; // @[Conditional.scala 39:67 PositionEstimator.scala 79:33]
  wire [8:0] _GEN_14 = _T_39 ? 9'h155 : _GEN_13; // @[Conditional.scala 39:67 PositionEstimator.scala 78:33]
  wire [8:0] _GEN_15 = _T_38 ? 9'h1aa : _GEN_14; // @[Conditional.scala 39:67 PositionEstimator.scala 77:33]
  wire [8:0] _GEN_16 = _T_37 ? 9'haa : _GEN_15; // @[Conditional.scala 39:67 PositionEstimator.scala 76:33]
  wire [8:0] _GEN_17 = _T_36 ? 9'h55 : _GEN_16; // @[Conditional.scala 39:67 PositionEstimator.scala 75:33]
  wire [8:0] _GEN_18 = _T_35 ? 9'h100 : _GEN_17; // @[Conditional.scala 39:67 PositionEstimator.scala 74:33]
  wire [8:0] _GEN_19 = _T_34 ? 9'h0 : _GEN_18; // @[Conditional.scala 40:58 PositionEstimator.scala 73:33]
  wire [8:0] _bits_T_1 = bits + 9'h1; // @[PositionEstimator.scala 86:22]
  wire [8:0] _bits_T_3 = bits - 9'h1; // @[PositionEstimator.scala 89:22]
  wire [8:0] _GEN_20 = ~io_qdec_inc & io_qdec_dec ? _bits_T_3 : bits; // @[PositionEstimator.scala 87:45 PositionEstimator.scala 89:14 PositionEstimator.scala 35:23]
  wire [8:0] _GEN_21 = io_qdec_inc & ~io_qdec_dec ? _bits_T_1 : _GEN_20; // @[PositionEstimator.scala 84:45 PositionEstimator.scala 86:14]
  wire  _GEN_23 = uncertain ? 1'h0 : error; // @[PositionEstimator.scala 70:27 PositionEstimator.scala 82:15 PositionEstimator.scala 36:24]
  wire  _GEN_25 = |uvwTransition & ~error ? _GEN_11 : _GEN_23; // @[PositionEstimator.scala 43:45]
  wire  _GEN_26 = |uvwTransition & ~error ? _GEN_11 : uncertain; // @[PositionEstimator.scala 43:45]
  wire  _GEN_29 = io_hall_u & io_hall_v & io_hall_w | ~(io_hall_u | io_hall_v | io_hall_w) | _GEN_25; // @[PositionEstimator.scala 38:91 PositionEstimator.scala 41:15]
  wire  _GEN_30 = io_hall_u & io_hall_v & io_hall_w | ~(io_hall_u | io_hall_v | io_hall_w) | _GEN_26; // @[PositionEstimator.scala 38:91 PositionEstimator.scala 42:19]
  assign io_theta_bits = bits; // @[PositionEstimator.scala 91:19]
  assign io_theta_error = error; // @[PositionEstimator.scala 92:20]
  assign io_theta_uncertain = uncertain; // @[PositionEstimator.scala 93:24]
  always @(posedge clock) begin
    previousHall_u <= io_hall_u; // @[PositionEstimator.scala 31:31]
    previousHall_v <= io_hall_v; // @[PositionEstimator.scala 31:31]
    previousHall_w <= io_hall_w; // @[PositionEstimator.scala 31:31]
    if (reset) begin // @[PositionEstimator.scala 35:23]
      bits <= 9'h0; // @[PositionEstimator.scala 35:23]
    end else if (io_hall_u & io_hall_v & io_hall_w | ~(io_hall_u | io_hall_v | io_hall_w)) begin // @[PositionEstimator.scala 38:91]
      bits <= 9'h0; // @[PositionEstimator.scala 40:14]
    end else if (|uvwTransition & ~error) begin // @[PositionEstimator.scala 43:45]
      if (uvwTransition == 3'h4 & io_hall_v & ~io_hall_w) begin // @[PositionEstimator.scala 47:71]
        bits <= 9'h2a; // @[PositionEstimator.scala 49:18]
      end else begin
        bits <= _GEN_8;
      end
    end else if (uncertain) begin // @[PositionEstimator.scala 70:27]
      bits <= _GEN_19;
    end else begin
      bits <= _GEN_21;
    end
    error <= reset | _GEN_29; // @[PositionEstimator.scala 36:24 PositionEstimator.scala 36:24]
    uncertain <= reset | _GEN_30; // @[PositionEstimator.scala 37:28 PositionEstimator.scala 37:28]
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_REG_INIT
  _RAND_0 = {1{`RANDOM}};
  previousHall_u = _RAND_0[0:0];
  _RAND_1 = {1{`RANDOM}};
  previousHall_v = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  previousHall_w = _RAND_2[0:0];
  _RAND_3 = {1{`RANDOM}};
  bits = _RAND_3[8:0];
  _RAND_4 = {1{`RANDOM}};
  error = _RAND_4[0:0];
  _RAND_5 = {1{`RANDOM}};
  uncertain = _RAND_5[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
