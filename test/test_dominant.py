#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


# NOTE: This test must be run with the CAN bus clamped at the dominant level.
#
# Required hardware setup:
# - Force the bus to the dominant level, i.e. drive CANH high (toward VCC)
#   AND CANL low (toward GND) so the differential (CANH - CANL) stays above
#   the receiver's dominant threshold (~0.9 V). A bus-dominant forcing
#   circuit, or grounding a transceiver's TXD pin, achieves this.
#
# Do NOT use these as a dominant clamp (they do NOT produce a dominant level):
# - A short between CANH and CANL gives a differential of ~0 V, which the
#   transceiver reads as RECESSIVE, not dominant.
#
# Operational caveat (why Phase B uses a long sleep with a manual hand-off):
# - In practice, applying the dominant clamp BEFORE opening the channel is
#   unreliable. Two suspected causes (not yet pinned down):
#     (a) FDCAN seems to need a recessive->dominant edge after the channel
#         opens. With the bus already dominant at open time, the controller
#         may stay in its bus-integrity wait state and REC never increments.
#     (b) When the clamp is powered by a small source (e.g. a coin cell),
#         the dominant level cannot be sustained — the supply sags under
#         the transceiver's dominant current draw and only a momentary
#         dominant pulse reaches the bus.
# - The workaround used here: open the channel with the bus recessive, then
#   apply the dominant clamp by hand within the sleep window in Phase B.
class DominantTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_error_passive_under_dominant_bus(self):
        """Verify that opening the channel on a dominant-clamped CAN bus
        drives the node into error-passive state, with the expected F
        flags raised.

        Two phases:

        Phase A (internal loopback, sanity):
            Opens the channel in internal loopback (=). The DUT must
            report no error and node_sts=ER_ACTV. This guards against
            false-positive runs where the device is broken before the
            dominant bus is even applied.

        Phase B (external loopback, dominant bus):
            Opens the channel in external loopback (O) with the bus
            still RECESSIVE, then waits in time.sleep(5) so the
            operator can apply the dominant clamp by hand. The
            recessive->dominant edge after the channel is open seems
            to be required for FDCAN to leave its bus-integrity wait
            state (clamping before open has been observed not to work
            — see the class NOTE for the suspected causes). Once the
            dominant level is applied, continuous FORM errors on the
            receive side increment REC and the node enters
            error-passive after ~128 frame attempts.

            Expected F flags right after error-passive is reached:
                bit 2 (EI,  Error Warning)  = 0x04
                bit 5 (EPI, Error Passive)  = 0x20
                bit 7 (BEI, Bus Error)      = 0x80
                                              ------
                                              F = 0xA4

            After F is read once, the EI / BEI status flags clear and
            the next F read returns F00. The detailed f-command then
            reports node_sts=ER_PSSV with err_cnt_tx_rx=[0x00, 0x80]
            (REC=128).
        """
        #self.dut.print_on = True

        # --- Phase A: internal-loopback sanity check (no traffic on the bus) ---
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Sanity: device must start with F=00 before the dominant bus is applied")
        self.dut.send(b"f\r")
        status = self.dut.receive()
        self.assertIn(b"node_sts=ER_ACTV", status,
                      f"Sanity: DUT must be error-active before dominant bus, got: {status!r}")
        self.assertIn(b"err_cnt_tx_rx=[0x00, 0x00]", status,
                      f"Sanity: TEC and REC must both be 0 before dominant bus, got: {status!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- Phase B: external loopback over the dominant-clamped bus ---
        # Open the channel with the bus RECESSIVE. Within the sleep window
        # below, the operator must manually apply the dominant clamp so
        # FDCAN sees a clean recessive->dominant edge after open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # 5 s window: gives the operator time to apply the dominant clamp
        # by hand after O. The error-passive transition itself is fast
        # once the bus goes dominant — at 125 kbps each frame attempt is
        # ~1 ms, so REC=128 is reached in roughly 128 ms of continuous
        # FORM errors. The rest of the 5 s is purely manual-operation
        # margin (and tolerates a flaky clamp such as a sagging coin-cell
        # supply that may only hold dominant in bursts).
        time.sleep(5)

        # First F read: bits 2/5/7 -> 0x04 | 0x20 | 0x80 = 0xA4 (see docstring).
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r",
                         "Expected F=A4 (BEI|EPI|EI) under dominant bus; see docstring for bit breakdown")
        # Second F read: EI / BEI status flags are cleared by the first F read
        # (LAWICEL semantics); EPI is a state flag and stays implicit in f-command.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F=00 on the second read (status flags cleared by the first F)")
        # Detailed f-command must reflect the persistent error-passive state.
        self.dut.send(b"f\r")
        status = self.dut.receive()
        self.assertIn(b"node_sts=ER_PSSV", status,
                      f"Expected ER_PSSV (error-passive) under dominant bus, got: {status!r}")
        self.assertIn(b"err_cnt_tx_rx=[0x00, 0x80]", status,
                      f"Expected REC=0x80 (=128, error-passive threshold), got: {status!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_off_via_frame_send_under_dominant_bus(self):
        """Verify that attempting to transmit on a dominant-clamped
        bus drives the node into bus-off, with both error counters
        climbing in parallel.

        Loose counterpart to ShortTestCase.test_bus_off_and_recovery
        (CANH-CANL short -> stuck recessive -> BIT0 -> bus-off via TX
        side only). Here the bus is stuck dominant, and unlike the
        short case BOTH the RX path and the TX path see errors at
        once:
            - RX path: the continuous dominant level is interpreted as
              an incoming frame whose format keeps violating CAN rules,
              so REC climbs via FORM errors (capped at 128 once the node
              becomes error-passive on the receive side).
            - TX path: the queued frame's transmission attempts fail
              because the DUT cannot drive any recessive bit on top of
              the clamp, so TEC climbs by 8 per failed attempt.

        last_err_code ends up reporting FORM (not BIT1), because the
        RX-side FORM errors are continuous and overwrite the most
        recent error code register; the TX-side BIT1 errors still
        drive TEC, but their code is not the latest one seen.

        Two phases:

        Phase A (internal loopback, sanity):
            Opens the channel in internal loopback (=). The DUT must
            report no error and node_sts=ER_ACTV. Guards against a
            broken device before the dominant bus is even applied.

        Phase B (Normal mode, dominant bus, single frame send):
            Opens the channel in Normal mode (O) with the bus still
            RECESSIVE, then waits 5 s so the operator can apply the
            dominant clamp by hand (see class NOTE for why the clamp
            cannot be applied before O). With the clamp in place,
            sends a single classic frame (t0000). Auto-retransmission
            keeps trying the frame; TEC reaches the bus-off threshold
            well within the 200 ms wait that follows.

            Expected F flags right after bus-off is reached:
                bit 2 (EI,  Error Warning)  = 0x04
                bit 4 (BO,  Bus-Off)        = 0x10
                bit 5 (EPI, Error Passive)  = 0x20
                bit 7 (BEI, Bus Error)      = 0x80
                                              ------
                                              F = 0xB4

            Detailed f-command after the F reads is:
                node_sts=BUS_OFF
                last_err_code=FORM
                err_cnt_tx_rx=[0xF8, 0x80]   # TEC=248, REC=128
        """
        #self.dut.print_on = True

        # --- Phase A: internal-loopback sanity check (no traffic on the bus) ---
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Sanity: device must start with F=00 before the dominant bus is applied")
        self.dut.send(b"f\r")
        status = self.dut.receive()
        self.assertIn(b"node_sts=ER_ACTV", status,
                      f"Sanity: DUT must be error-active before dominant bus, got: {status!r}")
        self.assertIn(b"err_cnt_tx_rx=[0x00, 0x00]", status,
                      f"Sanity: TEC and REC must both be 0 before dominant bus, got: {status!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- Phase B: Normal mode over the dominant-clamped bus, single frame send ---
        # Open the channel with the bus RECESSIVE. Within the sleep window
        # below, the operator must manually apply the dominant clamp so
        # FDCAN sees a clean recessive->dominant edge after open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # 5 s window: gives the operator time to apply the dominant clamp
        # by hand after O (same as test_error_passive_under_dominant_bus).
        time.sleep(5)

        # Send one classic frame. With the bus clamped dominant, the DUT
        # cannot drive any recessive bit during transmission (BIT1 error,
        # TEC += 8) AND the bus itself looks like a malformed incoming
        # frame to the receiver path (FORM error, REC += 1). Auto-retransmit
        # keeps trying until TEC exceeds 255 -> bus-off.
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "Tx queue must accept the frame while the node is still error-active")

        # Wait for bus-off: TEC climbs in ~ms steps; 200 ms is well above
        # the time needed to cross 255.
        time.sleep(0.2)

        # First F read: bits 2/4/5/7 -> 0x04 | 0x10 | 0x20 | 0x80 = 0xB4.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FB4\r",
                         "Expected F=B4 (BEI|EPI|BO|EI) after bus-off via dominant-clamped tx")
        # Second F read: status flags cleared by the first F.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F=00 on the second read (status flags cleared by the first F)")
        # Detailed f-command must reflect bus-off. last_err_code is FORM,
        # not BIT1, because the RX-side FORM errors are continuous and
        # overwrite the latest-error-code register; the TX-side BIT1 errors
        # still drive TEC up to bus-off but their code is not the latest
        # one captured. err_cnt_tx_rx=[0xF8, 0x80] = TEC=248, REC=128 —
        # REC capped at the error-passive threshold, TEC at the bus-off
        # snapshot value.
        self.dut.send(b"f\r")
        status = self.dut.receive()
        self.assertIn(b"node_sts=BUS_OFF", status,
                      f"Expected BUS_OFF after dominant-clamped tx, got: {status!r}")
        self.assertIn(b"last_err_code=FORM", status,
                      f"Expected last_err_code=FORM (RX-side FORM errors dominate the latest-error "
                      f"register even while TX-side BIT1 errors drive TEC), got: {status!r}")
        self.assertIn(b"err_cnt_tx_rx=[0xF8, 0x80]", status,
                      f"Expected TEC=0xF8 (bus-off snapshot) and REC=0x80 (error-passive cap), "
                      f"got: {status!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
