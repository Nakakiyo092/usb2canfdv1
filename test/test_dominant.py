#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


# NOTE: This test must be run with the CAN bus clamped at the dominant level.
#
# Required hardware setup:
# - Fix the CAN bus at the dominant level before running (e.g. short CANH
#   to GND, or use a bus-dominant forcing circuit).
# - Optionally a short between CANH and CANL can be used instead.
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


# NOTE: This test needs to be done with CAN bus fixed at dominant level.
#
# Required hardware setup:
# - Fix the CAN bus at dominant level before running (e.g. connect CANH to GND
#   or use a bus-dominant forcing circuit).
# - Release the dominant condition when the test prompts you to do so.
# - Optionally short circuit between CANH and CANL can be used instead of
#   fixing to dominat level.
#
# How it works:
# - In external loopback mode (+), the dominant bus forces continuous FORM
#   errors on the receive side, incrementing REC to 128 within ~200 ms.
# - Once error-passive (REC >= 128), the FDCAN hardware forces ESI=1 in any
#   CAN-FD frame the node transmits.
# - After the dominant condition is released, the device self-acknowledges its
#   own transmissions (external loopback), generating Tx events.
# - The first successful frame is transmitted while REC >= 128 -> ESI=1.
# - After that frame, REC drops from >= 128 to 127 (per CAN spec) -> error-active.
# - The second frame is transmitted in error-active state -> ESI=0.
class EsiTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_esi_bit_value(self):
        """Verify ESI bit value in CAN-FD Tx event reports:
        - ESI = 1 while the DUT is error-passive (REC >= 128)
        - ESI = 0 after the DUT recovers to error-active (REC < 128)

        Phase 1 (bus dominant):
            Open in external loopback mode (+). The dominant bus causes
            continuous FORM errors on the receive side, incrementing REC to
            128 (error-passive). No frames need to be sent during this phase.

        Phase 2 (bus released, error-passive):
            The device self-ACKs its own transmissions in external loopback
            mode. The first successful frame is transmitted while REC >= 128,
            so the FDCAN hardware forces ESI=1 in the frame, which is
            reflected in the Tx event report.

        Phase 3 (recovery to error-active):
            After the first successful reception, REC drops from >= 128 to 127
            (per CAN spec), making the node error-active. The next frame has
            ESI=0.
        """
        #self.dut.print_on = True

        # Enable Tx event reporting with ESI (no timestamp, Rx frame reporting off).
        # z0012: n=0 (no timestamp), x=0 (reserved), yy=0x12 (Tx event | ESI)
        self.dut.send(b"z0012\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Phase 1: open in external loopback mode.
        # The dominant bus causes continuous FORM errors on the receive side,
        # incrementing REC to 128 (error-passive) within ~200 ms.
        # The channel must stay open throughout so REC is preserved.
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")
        time.sleep(0.5)     # wait for error passive (> 1ms * 128)

        self.dut.send(b"f\r")
        status = self.dut.receive()
        self.assertIn(b"ER_PSSV", status,
                      "DUT must enter error-passive on a dominant bus within 500ms")

        # Phase 2: bus released; device self-ACKs in external loopback mode.
        # The channel remains open so REC (>= 128) is preserved across this
        # prompt — no C command is issued here.
        input("\nRelease the dominant bus level, then press Enter...")

        # Send one CAN-FD frame. The node is error-passive (REC >= 128) at the
        # moment of transmission, so the hardware forces ESI=1 in the frame.
        # Tx event format (z0012, no timestamp): '\r' + 'zd03F01\r'
        self.dut.send(b"d03F0\r")
        rx_data = self.dut.receive()
        self.assertIn(b"zd03F01\r", rx_data,
                      f"Expected CAN-FD Tx event with ESI=1 while error-passive, got: {rx_data!r}")

        # Phase 3: after the first successful self-ACKed frame, REC drops from
        # >= 128 to 127 (per CAN spec), making the node error-active.
        # Confirm ESI = 0 in the next frame.
        # Tx event format (z0012, no timestamp): '\r' + 'zd03F00\r'
        self.dut.send(b"d03F0\r")
        rx_data = self.dut.receive()
        self.assertIn(b"zd03F00\r", rx_data,
                      f"Expected CAN-FD Tx event with ESI=0 after recovery to error-active, got: {rx_data!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
