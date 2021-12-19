// @file
//   QuadratureDecoderSpec.scala
// @author
//   Fujii Naomichi
// @copyright
//   (c) 2021 Fujii Naomichi
// SPDX-License-Identifier: MIT

import chisel3._
import chisel3.util._
import chisel3.iotesters._
import chiseltest._
import org.scalatest._
import org.scalatest.flatspec.AnyFlatSpec

class PulseGenerator(a: Bool, b: Bool) {
    def init() = {
        a.poke(false.B)
        b.poke(false.B)
    }

    // パルスを1つ前進させる
    def up() = {
        if (a.peek().litToBoolean == false) {
            if (b.peek().litToBoolean == false) {
                a.poke(true.B)
            } else {
                b.poke(false.B)
            }
        } else {
            if (b.peek().litToBoolean == false) {
                b.poke(true.B)
            } else {
                a.poke(false.B)
            }
        }
    }

    // パルスを1つ後退させる
    def down() = {
        if (a.peek().litToBoolean == false) {
            if (b.peek().litToBoolean == false) {
                b.poke(true.B)
            } else {
                a.poke(true.B)
            }
        } else {
            if (b.peek().litToBoolean == false) {
                a.poke(false.B)
            } else {
                b.poke(false.B)
            }
        }
    }
}

class QuadratureDecoderSpec extends AnyFlatSpec with ChiselScalatestTester {
    "QuadratureDecoderTest" should "pass" in {
        test(new QuadratureDecoder(16, false.B)) { c =>
            val sink = new DecoupledDriver(c.io.counter)
            val pulse_gen = new PulseGenerator(c.io.enc.a, c.io.enc.b)
            sink.setSinkClock(c.clock)
            sink.initSink();

            // Count Up Test
            c.clock.step()
            fork {
                pulse_gen.up()
                c.clock.step(3)
                pulse_gen.up()
                c.clock.step(3)
                pulse_gen.up()
                c.clock.step(3)
                pulse_gen.up()
            }.fork {
                c.clock.step(15)
                sink.expectDequeue(4.S)
            }.fork {
                var count = 0
                while (c.io.counter.valid.peek().litToBoolean == false) {
                    if (c.io.inc.peek().litToBoolean == true) {
                        count = count + 1
                    }
                    c.clock.step(1)
                }
                assert(count == 4)
            }.join()

            // Count Down Test
            c.clock.step()
            fork {
                pulse_gen.down()
                c.clock.step(3)
                pulse_gen.down()
                c.clock.step(3)
                pulse_gen.down()
                c.clock.step(3)
                pulse_gen.down()
            }.fork {
                c.clock.step(15)
                sink.expectDequeue(-4.S)
            }.fork {
                var count = 0
                while (c.io.counter.valid.peek().litToBoolean == false) {
                    if (c.io.dec.peek().litToBoolean == true) {
                        count = count + 1
                    }
                    c.clock.step(1)
                }
                assert(count == 4)
            }.join()

            // Error State Test
            c.clock.step()
            fork {
                c.io.enc.a.poke(false.B)
                c.io.enc.b.poke(false.B)
                c.clock.step(3)
                c.io.enc.a.poke(true.B) // error
                c.io.enc.b.poke(true.B)
                c.clock.step(3)
                c.io.enc.a.poke(false.B) // +1
                c.io.enc.b.poke(true.B)
                c.clock.step(3)
                c.io.enc.a.poke(true.B) // -1
                c.io.enc.b.poke(true.B)
                c.clock.step(3)
                c.io.enc.a.poke(false.B) // error
                c.io.enc.b.poke(false.B)
            }.fork {
                c.clock.step(20)
                sink.expectDequeue(0.S)
            }.join()

            // Conflict inc/dec and ready Test
            c.clock.step()
            fork {
                pulse_gen.down()
                c.clock.step(3)
                pulse_gen.down()
                c.clock.step(3)
                pulse_gen.down()
                c.clock.step(3)
                pulse_gen.down()
            }.fork {
                c.io.counter.ready.poke(true.B)
                var count = 0
                for (i <- 1 to 15) {
                    if (c.io.counter.valid.peek().litToBoolean == true) {
                        count = count + c.io.counter.bits.peek().litValue.toInt
                    }
                    c.clock.step(1)
                }
                c.io.counter.ready.poke(false.B)
                c.clock.step(1)
                assert(count == -4)
            }.join()
        }
    }
}
