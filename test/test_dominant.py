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
            Opens the channel in external loopback (O). The dominant
            bus causes continuous FORM errors on the receive side,
            incrementing REC. After ~128 frame attempts the node enters
            error-passive.

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
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Wait for error-passive: at 125 kbps default each frame attempt
        # is ~1 ms; REC reaches 128 (error-passive threshold) after roughly
        # 128 ms of continuous FORM errors. 500 ms is well above that and
        # gives margin for FW boot timing and USB jitter.
        time.sleep(0.5)

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


if __name__ == "__main__":
    unittest.main()
