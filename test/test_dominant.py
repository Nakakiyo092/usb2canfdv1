#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


# NOTE: This test needs to be done with CAN bus fixed at dominant level.
class DominantTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_dominant(self):
        #self.dut.print_on = True

        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check no error in internal loopback mode
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(),
                         b"f: node_sts=ER_ACTV, last_err_code=NONE, err_cnt_tx_rx=[0x00, 0x00], th_bus_load_percent=00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check error passive after "receiving" fixed signal at dominant level
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        time.sleep(0.5)     # wait for error passive ( > 1ms * 128 / 1)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r")  # BEI + EPI + EI
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")  # check error clear
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(),
                         b"f: node_sts=ER_PSSV, last_err_code=FORM, err_cnt_tx_rx=[0x00, 0x80], th_bus_load_percent=00\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


# NOTE: This test verifies ESI (Error State Indicator) bit values in CAN-FD
# frames while the DUT transitions through error states.
#
# Required hardware setup:
# - A partner CAN node must be connected to the same bus so that it can
#   acknowledge the DUT's transmissions.
# - Before running, fix the CAN bus at dominant level (e.g. short CANH to GND
#   or use a bus-dominant forcing circuit).
# - Release the dominant condition when the test prompts you to do so.
#
# Why a partner node is required:
# - The FDCAN peripheral is fully reset (TEC/REC cleared) by the C command, so
#   the error state cannot be carried across a channel close/reopen cycle.
# - Tx event reports are only generated on *successful* transmissions (ACK
#   received); failed DAR frames produce no Tx event.
# - Therefore ESI can only be observed while the same channel session is open
#   and a partner node provides the ACK after the bus dominant force is removed.
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
        - ESI = 1 while the DUT is error-passive (TEC >= 128)
        - ESI = 0 after the DUT recovers to error-active (TEC < 128)

        Phase 1 (bus dominant, DAR mode):
            Each failed CAN-FD transmission on the dominant bus increments TEC.
            The loop exits when 'f' reports ER_PSSV.

        Phase 2 (bus released, still error-passive):
            The partner node acknowledges transmissions.  The FDCAN hardware
            forces ESI = 1 in transmitted frames when TEC >= 128, regardless
            of the descriptor's ESI field.  The Tx event report reflects this.

        Phase 3 (recovery to error-active):
            Each acknowledged transmission decrements TEC by 1.  The loop
            exits when 'f' reports ER_ACTV, and ESI = 0 is then confirmed.
        """
        #self.dut.print_on = True

        # Enable Tx event reporting with ESI (no timestamp, Rx frame reporting off).
        # z0012: n=0 (no timestamp), x=0 (reserved), yy=0x12 (Tx event | ESI)
        self.dut.send(b"z0012\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Phase 1: accumulate TEC on dominant bus using DAR (no-retransmit) mode.
        # The channel must stay open throughout so TEC is preserved.
        self.dut.send(b"-\r")           # external CAN, DAR (no retransmit)
        self.assertEqual(self.dut.receive(), b"\r")

        status = b""
        for _ in range(30):
            self.dut.send(b"d03F0\r")       # CAN-FD base frame, DLC = 0
            self.dut.receive()              # buffer-save '\r'; no Tx event on failure
            # Allow the FDCAN error counter to be updated before polling status.
            time.sleep(0.05)
            self.dut.send(b"f\r")
            status = self.dut.receive()
            if b"ER_PSSV" in status:
                break

        self.assertIn(b"ER_PSSV", status,
                      "DUT must enter error-passive after DAR frames on a dominant bus")

        # Phase 2: bus released; partner node will acknowledge transmissions.
        # The channel remains open so TEC (>= 128) is preserved across this
        # prompt — no C command is issued here.
        input("\nRelease the dominant bus level, then press Enter...")

        # Send CAN-FD frames; each acknowledged frame produces a Tx event.
        # ESI = 1 is expected while TEC >= 128.
        # Tx event format (z0012, no timestamp): '\r' + 'zd03F01\r'
        esi1_found = False
        for _ in range(200):
            self.dut.send(b"d03F0\r")
            rx_data = self.dut.receive()
            if b"zd03F01\r" in rx_data:
                esi1_found = True
                break
            # Stop early if the node has already recovered unexpectedly.
            self.dut.send(b"f\r")
            if b"ER_ACTV" in self.dut.receive():
                break

        self.assertTrue(esi1_found,
                        "Expected at least one CAN-FD Tx event with ESI=1 "
                        "while the DUT is in error-passive state")

        # Phase 3: wait for TEC to drop below 128 (error-active recovery).
        # Each successful DAR transmission decrements TEC by 1.
        for _ in range(300):
            self.dut.send(b"f\r")
            status = self.dut.receive()
            if b"ER_ACTV" in status:
                break
            self.dut.send(b"d03F0\r")
            self.dut.receive()

        self.assertIn(b"ER_ACTV", status,
                      "DUT must recover to error-active after sufficient "
                      "successful transmissions")

        # Confirm ESI = 0 in error-active state.
        # Tx event format (z0012, no timestamp): '\r' + 'zd03F00\r'
        self.dut.send(b"d03F0\r")
        rx_data = self.dut.receive()
        self.assertIn(b"zd03F00\r", rx_data,
                      f"Expected ESI=0 in error-active state, got: {rx_data!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
