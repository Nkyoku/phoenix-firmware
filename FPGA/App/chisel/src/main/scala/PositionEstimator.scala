// @file
//   PositionEstimator.scala
// @author
//   Fujii Naomichi
// @copyright
//   (c) 2021 Fujii Naomichi
// SPDX-License-Identifier: MIT

import chisel3._
import chisel3.util._

class HallSignals extends Bundle {
    val u = Bool()
    val v = Bool()
    val w = Bool()
}

class PositionEstimator(thetaWidth: Int) extends Module {
    val io = IO(new Bundle {
        val hall = Input(new HallSignals())
        val qdec_inc = Input(Bool())
        val qdec_dec = Input(Bool())
        val theta_bits = Output(UInt(thetaWidth.W))
        val theta_error = Output(Bool())
        val theta_uncertain = Output(Bool())
    })

    val pulsePerRotation = 1 << thetaWidth

    // Transition detector of the hall signals
    val previousHall = RegNext(io.hall)
    val uvwTransition = Cat(previousHall.u ^ io.hall.u, previousHall.v ^ io.hall.v, previousHall.w ^ io.hall.w)

    // Estimator
    val bits = RegInit(0.U(thetaWidth.W))
    val error = RegInit(true.B)
    val uncertain = RegInit(true.B)
    when((io.hall.u && io.hall.v && io.hall.w) || !(io.hall.u || io.hall.v || io.hall.w)) {
        // Hall sensor error
        bits := 0.U
        error := true.B
        uncertain := true.B
    }.elsewhen(uvwTransition.orR && !error) {
        // Transition was occured
        // Set theta_bits to transition angle
        error := false.B
        uncertain := false.B
        when((uvwTransition === "b100".U) && io.hall.v && !io.hall.w) {
            // 30 deg
            bits := (pulsePerRotation / 12).U
        }.elsewhen((uvwTransition === "b100".U) && !io.hall.v && io.hall.w) {
            // 210 deg
            bits := (pulsePerRotation * 7 / 12).U
        }.elsewhen((uvwTransition === "b010".U) && io.hall.u && !io.hall.w) {
            // 330 deg
            bits := (pulsePerRotation * 11 / 12).U
        }.elsewhen((uvwTransition === "b010".U) && !io.hall.u && io.hall.w) {
            // 150 deg
            bits := (pulsePerRotation * 5 / 12).U
        }.elsewhen((uvwTransition === "b001".U) && io.hall.u && !io.hall.v) {
            // 270 deg
            bits := (pulsePerRotation * 3 / 4).U
        }.elsewhen((uvwTransition === "b001".U) && !io.hall.u && io.hall.v) {
            // 90 deg
            bits := (pulsePerRotation / 4).U
        }.otherwise {
            // Invalid transition
            bits := 0.U
            error := true.B
            uncertain := true.B
        }
    }.elsewhen(uncertain) {
        // Initial accurate angle is unknown
        // Estimate angle by only hall signals
        switch(Cat(io.hall.u, io.hall.v, io.hall.w)) {
            is("b000".U) { bits := 0.U } // error
            is("b001".U) { bits := (pulsePerRotation / 2).U } // 180 deg
            is("b010".U) { bits := (pulsePerRotation / 6).U } // 60 deg
            is("b011".U) { bits := (pulsePerRotation / 3).U } // 120 deg
            is("b100".U) { bits := (pulsePerRotation * 5 / 6).U } // 300 deg
            is("b101".U) { bits := (pulsePerRotation * 2 / 3).U } // 240 deg
            is("b110".U) { bits := 0.U } // 0 deg
            is("b111".U) { bits := 0.U } // error
        }
        error := false.B
        uncertain := true.B
    }.elsewhen(io.qdec_inc && !io.qdec_dec) {
        // Count up
        bits := bits + 1.U
    }.elsewhen(!io.qdec_inc && io.qdec_dec) {
        // Count down
        bits := bits - 1.U
    }
    io.theta_bits := bits
    io.theta_error := error
    io.theta_uncertain := uncertain
}

object PositionEstimator extends App {
    (new chisel3.stage.ChiselStage).emitVerilog(new PositionEstimator(9), args)
}
