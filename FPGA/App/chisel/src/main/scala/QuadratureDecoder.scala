// @file
//   QuadratureDecoder.scala
// @author
//   Fujii Naomichi
// @copyright
//   (c) 2021 Fujii Naomichi
// SPDX-License-Identifier: MIT

import chisel3._
import chisel3.util._

class EncoderSignals extends Bundle {
    val a = Bool()
    val b = Bool()
}

class QuadratureDecoder(counterWidth: Int, inverse: Bool) extends Module {
    val io = IO(new Bundle {
        val enc = Input(new EncoderSignals())
        val inc = Output(Bool())
        val dec = Output(Bool())
        val counter = Decoupled(SInt(counterWidth.W))
    })

    // Decoder
    val previousEncA = RegNext(io.enc.a)
    val previousEncB = RegNext(io.enc.b)
    val state = Cat(previousEncB, previousEncA, io.enc.b, io.enc.a)
    val inc_dec = WireDefault(0.U)
    switch(state) {
        is("b0000".U) { inc_dec := "b00".U }
        is("b0001".U) { inc_dec := "b10".U } // +1
        is("b0010".U) { inc_dec := "b01".U } // -1
        is("b0011".U) { inc_dec := "b11".U } // error
        is("b0100".U) { inc_dec := "b01".U } // -1
        is("b0101".U) { inc_dec := "b00".U }
        is("b0110".U) { inc_dec := "b11".U } // error
        is("b0111".U) { inc_dec := "b10".U } // +1
        is("b1000".U) { inc_dec := "b10".U } // +1
        is("b1001".U) { inc_dec := "b11".U } // error
        is("b1010".U) { inc_dec := "b00".U }
        is("b1011".U) { inc_dec := "b01".U } // -1
        is("b1100".U) { inc_dec := "b11".U } // error
        is("b1101".U) { inc_dec := "b01".U } // -1
        is("b1110".U) { inc_dec := "b10".U } // +1
        is("b1111".U) { inc_dec := "b00".U }
    }
    io.inc := Mux(inverse, inc_dec(0), inc_dec(1))
    io.dec := Mux(inverse, inc_dec(1), inc_dec(0))

    // Counter and latch
    val internalCounter = RegInit(0.S(counterWidth.W))
    val nextInternalCounter = Mux(io.counter.ready, 0.S, internalCounter)
    val counter_bits = RegInit(0.S(counterWidth.W))
    when(io.counter.ready) {
        counter_bits := internalCounter
    }
    when(inc_dec === "b10".U) {
        // Count up
        internalCounter := Mux(inverse, nextInternalCounter - 1.S, nextInternalCounter + 1.S)
    }.elsewhen(inc_dec === "b01".U) {
        // Count down
        internalCounter := Mux(inverse, nextInternalCounter + 1.S, nextInternalCounter - 1.S)
    }.otherwise {
        // No change
        internalCounter := nextInternalCounter
    }
    io.counter.valid := RegNext(io.counter.ready)
    io.counter.bits := counter_bits
}

object QuadratureDecoder extends App {
    (new chisel3.stage.ChiselStage).emitVerilog(new QuadratureDecoder(16, false.B), args)
}
