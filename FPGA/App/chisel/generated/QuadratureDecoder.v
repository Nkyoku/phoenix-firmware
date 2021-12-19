module QuadratureDecoder(
  input         clock,
  input         reset,
  input         io_enc_a,
  input         io_enc_b,
  output        io_inc,
  output        io_dec,
  input         io_counter_ready,
  output        io_counter_valid,
  output [15:0] io_counter_bits
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
`endif // RANDOMIZE_REG_INIT
  reg  previousEncA; // @[QuadratureDecoder.scala 26:31]
  reg  previousEncB; // @[QuadratureDecoder.scala 27:31]
  wire [3:0] state = {previousEncB,previousEncA,io_enc_b,io_enc_a}; // @[Cat.scala 30:58]
  wire  _T = 4'h0 == state; // @[Conditional.scala 37:30]
  wire  _T_1 = 4'h1 == state; // @[Conditional.scala 37:30]
  wire  _T_2 = 4'h2 == state; // @[Conditional.scala 37:30]
  wire  _T_3 = 4'h3 == state; // @[Conditional.scala 37:30]
  wire  _T_4 = 4'h4 == state; // @[Conditional.scala 37:30]
  wire  _T_5 = 4'h5 == state; // @[Conditional.scala 37:30]
  wire  _T_6 = 4'h6 == state; // @[Conditional.scala 37:30]
  wire  _T_7 = 4'h7 == state; // @[Conditional.scala 37:30]
  wire  _T_8 = 4'h8 == state; // @[Conditional.scala 37:30]
  wire  _T_9 = 4'h9 == state; // @[Conditional.scala 37:30]
  wire  _T_10 = 4'ha == state; // @[Conditional.scala 37:30]
  wire  _T_11 = 4'hb == state; // @[Conditional.scala 37:30]
  wire  _T_12 = 4'hc == state; // @[Conditional.scala 37:30]
  wire  _T_13 = 4'hd == state; // @[Conditional.scala 37:30]
  wire  _T_14 = 4'he == state; // @[Conditional.scala 37:30]
  wire [1:0] _GEN_1 = _T_14 ? 2'h2 : 2'h0; // @[Conditional.scala 39:67 QuadratureDecoder.scala 45:33]
  wire [1:0] _GEN_2 = _T_13 ? 2'h1 : _GEN_1; // @[Conditional.scala 39:67 QuadratureDecoder.scala 44:33]
  wire [1:0] _GEN_3 = _T_12 ? 2'h3 : _GEN_2; // @[Conditional.scala 39:67 QuadratureDecoder.scala 43:33]
  wire [1:0] _GEN_4 = _T_11 ? 2'h1 : _GEN_3; // @[Conditional.scala 39:67 QuadratureDecoder.scala 42:33]
  wire [1:0] _GEN_5 = _T_10 ? 2'h0 : _GEN_4; // @[Conditional.scala 39:67 QuadratureDecoder.scala 41:33]
  wire [1:0] _GEN_6 = _T_9 ? 2'h3 : _GEN_5; // @[Conditional.scala 39:67 QuadratureDecoder.scala 40:33]
  wire [1:0] _GEN_7 = _T_8 ? 2'h2 : _GEN_6; // @[Conditional.scala 39:67 QuadratureDecoder.scala 39:33]
  wire [1:0] _GEN_8 = _T_7 ? 2'h2 : _GEN_7; // @[Conditional.scala 39:67 QuadratureDecoder.scala 38:33]
  wire [1:0] _GEN_9 = _T_6 ? 2'h3 : _GEN_8; // @[Conditional.scala 39:67 QuadratureDecoder.scala 37:33]
  wire [1:0] _GEN_10 = _T_5 ? 2'h0 : _GEN_9; // @[Conditional.scala 39:67 QuadratureDecoder.scala 36:33]
  wire [1:0] _GEN_11 = _T_4 ? 2'h1 : _GEN_10; // @[Conditional.scala 39:67 QuadratureDecoder.scala 35:33]
  wire [1:0] _GEN_12 = _T_3 ? 2'h3 : _GEN_11; // @[Conditional.scala 39:67 QuadratureDecoder.scala 34:33]
  wire [1:0] _GEN_13 = _T_2 ? 2'h1 : _GEN_12; // @[Conditional.scala 39:67 QuadratureDecoder.scala 33:33]
  wire [1:0] _GEN_14 = _T_1 ? 2'h2 : _GEN_13; // @[Conditional.scala 39:67 QuadratureDecoder.scala 32:33]
  wire [1:0] inc_dec = _T ? 2'h0 : _GEN_14; // @[Conditional.scala 40:58 QuadratureDecoder.scala 31:33]
  reg [15:0] internalCounter; // @[QuadratureDecoder.scala 52:34]
  wire [15:0] nextInternalCounter = io_counter_ready ? $signed(16'sh0) : $signed(internalCounter); // @[QuadratureDecoder.scala 53:34]
  reg [15:0] counter_bits; // @[QuadratureDecoder.scala 54:31]
  wire [15:0] _internalCounter_T_2 = $signed(nextInternalCounter) - 16'sh1; // @[QuadratureDecoder.scala 60:61]
  wire [15:0] _internalCounter_T_5 = $signed(nextInternalCounter) + 16'sh1; // @[QuadratureDecoder.scala 60:88]
  reg  io_counter_valid_REG; // @[QuadratureDecoder.scala 68:32]
  assign io_inc = inc_dec[1]; // @[QuadratureDecoder.scala 48:47]
  assign io_dec = inc_dec[0]; // @[QuadratureDecoder.scala 49:47]
  assign io_counter_valid = io_counter_valid_REG; // @[QuadratureDecoder.scala 68:22]
  assign io_counter_bits = counter_bits; // @[QuadratureDecoder.scala 69:21]
  always @(posedge clock) begin
    previousEncA <= io_enc_a; // @[QuadratureDecoder.scala 26:31]
    previousEncB <= io_enc_b; // @[QuadratureDecoder.scala 27:31]
    if (reset) begin // @[QuadratureDecoder.scala 52:34]
      internalCounter <= 16'sh0; // @[QuadratureDecoder.scala 52:34]
    end else if (inc_dec == 2'h2) begin // @[QuadratureDecoder.scala 58:31]
      internalCounter <= _internalCounter_T_5; // @[QuadratureDecoder.scala 60:25]
    end else if (inc_dec == 2'h1) begin // @[QuadratureDecoder.scala 61:37]
      internalCounter <= _internalCounter_T_2; // @[QuadratureDecoder.scala 63:25]
    end else if (io_counter_ready) begin // @[QuadratureDecoder.scala 53:34]
      internalCounter <= 16'sh0;
    end
    if (reset) begin // @[QuadratureDecoder.scala 54:31]
      counter_bits <= 16'sh0; // @[QuadratureDecoder.scala 54:31]
    end else if (io_counter_ready) begin // @[QuadratureDecoder.scala 55:28]
      counter_bits <= internalCounter; // @[QuadratureDecoder.scala 56:22]
    end
    io_counter_valid_REG <= io_counter_ready; // @[QuadratureDecoder.scala 68:32]
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
  previousEncA = _RAND_0[0:0];
  _RAND_1 = {1{`RANDOM}};
  previousEncB = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  internalCounter = _RAND_2[15:0];
  _RAND_3 = {1{`RANDOM}};
  counter_bits = _RAND_3[15:0];
  _RAND_4 = {1{`RANDOM}};
  io_counter_valid_REG = _RAND_4[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule
