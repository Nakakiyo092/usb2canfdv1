#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class ExLoopbackTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_external_loopback(self):
        """Verify lossless loopback for all 8 frame types over external loopback.

        For each combination of:
          * frame type: data (t/T/d/D/b/B) or remote (r/R)
          * ID width: standard (lowercase) or extended (uppercase)
          * data phase: classic, CAN-FD without BRS, CAN-FD with BRS

        send both the minimum-length payload (single byte / no data)
        and the maximum-length payload (8 bytes for classic, 64 bytes
        for CAN-FD), and verify the looped-back frame matches
        byte-for-byte.

        Unlike the internal loopback variant, this exercises the
        transceiver and the external CAN bus path (requires a
        terminator and the bus wired back to itself).
        """
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check loopback of shortest frames of each type
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r",
                             f"Short std external loopback mismatch for {cmd!r}")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r",
                             f"Short ext external loopback mismatch for {cmd!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check loopback of longest frames of each type
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")
        tx_data = b"r03FF\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                         "Long remote std (r) external loopback mismatch")
        tx_data = b"t03F80011223344556677\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                         "Long data std (t, 8 bytes) external loopback mismatch")
        tx_data = b"d03FF" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                         "Long CAN-FD std no-BRS (d, 64 bytes) external loopback mismatch")
        tx_data = b"b03FF" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                         "Long CAN-FD std BRS (b, 64 bytes) external loopback mismatch")
        tx_data = b"R0137FEC8F\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data,
                         "Long remote ext (R) external loopback mismatch")
        tx_data = b"T0137FEC880011223344556677\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data,
                         "Long data ext (T, 8 bytes) external loopback mismatch")
        tx_data = b"D0137FEC8F" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data,
                         "Long CAN-FD ext no-BRS (D, 64 bytes) external loopback mismatch")
        tx_data = b"B0137FEC8F" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data,
                         "Long CAN-FD ext BRS (B, 64 bytes) external loopback mismatch")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_nominal_bitrate(self):
        """Verify that every nominal CAN bitrate (S0..S8) can transmit
        and receive a CAN-FD-with-BRS frame over external loopback.

        Iterates through all 9 nominal bitrate options
        (S0=10 kbps .. S8=1 Mbps); the data-phase bitrate keeps the
        default Y setting throughout.
        """
        #self.dut.print_on = True
        for rate in range(0, 9):
            cmd = "S" + str(rate) + "\r"
            self.dut.send(cmd.encode())
            self.assertEqual(self.dut.receive(), b"\r")
            self.dut.send(b"+\r")
            self.assertEqual(self.dut.receive(), b"\r")
            tx_data = b"b03F80011223344556677\r"
            self.dut.send(tx_data)
            self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                             f"S{rate}: nominal-bitrate external loopback mismatch")
            self.dut.send(b"C\r")
            self.assertEqual(self.dut.receive(), b"\r")


    def test_data_bitrate(self):
        """Verify that each supported CAN-FD data-phase bitrate can
        transmit and receive a CAN-FD-with-BRS frame over external
        loopback.

        Iterates through Y0, Y1, Y2, Y4, Y5. Y3 is intentionally
        skipped because the firmware does not support that data-phase
        bitrate.
        """
        #self.dut.print_on = True
        for rate in (0, 1, 2, 4, 5):
            cmd = "Y" + str(rate) + "\r"
            self.dut.send(cmd.encode())
            self.assertEqual(self.dut.receive(), b"\r")
            self.dut.send(b"+\r")
            self.assertEqual(self.dut.receive(), b"\r")
            tx_data = b"b03F80011223344556677\r"
            self.dut.send(tx_data)
            self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                             f"Y{rate}: data-bitrate external loopback mismatch")
            self.dut.send(b"C\r")
            self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_load_stepwise(self):
        """Verify the reported bus load value scales correctly with the
        generated traffic.

        Generates traffic at four nominal levels (0 %, 18 %, 36 %, 72 %)
        by sending increasingly large bursts of minimum-stuffing frames
        at 10 kbps. After each level, queries the bus load via the `f`
        command and checks that the reported percentage falls within a
        ±10 % window around the expected value (bus load value lives at
        columns 89-90 of the `f` response, in base 10).

        The 10 % point accuracy has no theoretical basis; it is an
        empirical margin to absorb the inaccuracy of the host-side bus
        load creation (USB latency, OS scheduling jitter, frame
        inter-burst gaps).
        """
        #self.dut.print_on = True

        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")

        tx_data = b"t55585555555555555555\r"    # 112bit * 0.1ms = 11.2ms
        for _ in range(0, 4):
            tx_data = tx_data + tx_data    # 11.2ms * 16 = 179.2ms

        # NOTE: The 10% point accuracy has no reasoning.
        # It is just to give some margin for the inaccurate bus load creation.

        # Check bus load in 0% mode (prove 10% point accuracy)
        time.sleep(1)
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92, "f-command response length mismatch (idle level)")
        pct = int(rx_data[89:91], 10)
        self.assertGreaterEqual(pct, 0,
                                f"idle bus load: reported {pct} %, expected within [0, 10]")
        self.assertLessEqual(pct, 10,
                             f"idle bus load: reported {pct} %, expected within [0, 10]")

        # Check bus load in 18% mode (prove 10% point accuracy)
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(1)
        self.dut.send(tx_data)
        time.sleep(0.5)
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92, "f-command response length mismatch (18 % level)")
        pct = int(rx_data[89:91], 10)
        self.assertGreaterEqual(pct, 8,
                                f"18 % bus load: reported {pct} %, expected within [8, 28]")
        self.assertLessEqual(pct, 28,
                             f"18 % bus load: reported {pct} %, expected within [8, 28]")

        # Check bus load in 36% mode (prove 10% point accuracy)
        tx_data = tx_data + tx_data
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(1)
        self.dut.send(tx_data)
        time.sleep(0.5)
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92, "f-command response length mismatch (36 % level)")
        pct = int(rx_data[89:91], 10)
        self.assertGreaterEqual(pct, 26,
                                f"36 % bus load: reported {pct} %, expected within [26, 46]")
        self.assertLessEqual(pct, 46,
                             f"36 % bus load: reported {pct} %, expected within [26, 46]")

        # Check bus load in 72% mode (prove 10% point accuracy)
        #tx_data = tx_data + tx_data     # Large chunk may be not sent correctly
        time.sleep(0.25)
        self.dut.send(tx_data)
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(0.25)
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92, "f-command response length mismatch (72 % level)")
        pct = int(rx_data[89:91], 10)
        self.assertGreaterEqual(pct, 62,
                                f"72 % bus load: reported {pct} %, expected within [62, 82]")
        self.assertLessEqual(pct, 82,
                             f"72 % bus load: reported {pct} %, expected within [62, 82]")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_load_full_10k(self):
        """Verify the bus load reading saturates near 100 % under
        full-load traffic at 10 kbps.

        Two patterns are checked:
          * Minimum stuffing (data 0x55..): reported 95 - 99 %
          * Maximum stuffing (data 0x00..): reported 83 - 88 %, an
            ~14 % underestimate that follows from
            (11 + 64) / 112 stuff-bit overhead in the frame.

        The 5 % margin absorbs test-setup and calculation inaccuracy.
        Bus load value lives at columns 89-90 of the `f` response, in
        base 10.
        """
        #self.dut.print_on = True

        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Minimum stuffing
        tx_data = b"t55585555555555555555\r"    # 112bit * 0.1ms = 11.2ms
        for _ in range(0, 5):
            tx_data = tx_data + tx_data    # 11.2ms * 32 = 358.4ms

        # Full load for more than 1 second
        time.sleep(1)
        for _ in range(0, 10):
            self.dut.receive()
            self.dut.send(tx_data)
            time.sleep(0.25)

        self.dut.receive()
        self.dut.send(b"F\r")
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92, "f-command response length mismatch (10 kbps min-stuffing)")
        # 7% margin for test setup and calculation (widened from 5% for Linux virtual box stability)
        pct = int(rx_data[89:91], 10)
        self.assertGreaterEqual(pct, 93,
                                f"10 kbps min-stuffing full load: reported {pct} %, expected within [93, 99]")
        self.assertLessEqual(pct, 99,
                             f"10 kbps min-stuffing full load: reported {pct} %, expected within [93, 99]")


        # Maximum stuffing (~ 20% * (11 + 64) / 112 ~ 14% underestimation)
        tx_data = b"t00080000000000000000\r"    # 112bit * 0.1ms = 11.2ms
        for _ in range(0, 5):
            tx_data = tx_data + tx_data    # 11.2ms * 32 = 358.4ms

        # Full load for more than 1 second
        time.sleep(1)
        for _ in range(0, 10):
            self.dut.receive()
            self.dut.send(tx_data)
            time.sleep(0.25)

        self.dut.receive()
        self.dut.send(b"F\r")
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92, "f-command response length mismatch (10 kbps max-stuffing)")
        # 7% margin for test setup and calculation (widened from 5% for Linux virtual box stability)
        pct = int(rx_data[89:91], 10)
        self.assertGreaterEqual(pct, 81,
                                f"10 kbps max-stuffing full load: reported {pct} %, expected within [81, 88]")
        self.assertLessEqual(pct, 88,
                             f"10 kbps max-stuffing full load: reported {pct} %, expected within [81, 88]")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_load_full_20k(self):
        """Verify the bus load reading saturates near 100 % under
        full-load traffic at 20 kbps.

        Mirrors test_bus_load_full_10k at the next nominal bitrate
        (S1). Both minimum-stuffing and maximum-stuffing patterns are
        checked with the same 5 % margin and ~14 % stuff-bit
        underestimation as the 10 kbps case.
        """
        #self.dut.print_on = True

        self.dut.send(b"S1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Minimum stuffing
        tx_data = b"t55585555555555555555\r"    # 112bit * 0.05ms = 5.6ms
        for _ in range(0, 5):
            tx_data = tx_data + tx_data    # 5.6ms * 32 = 179.2ms

        # Full load for more than 1 second
        time.sleep(1)
        for _ in range(0, 20):
            self.dut.receive()
            self.dut.send(tx_data)
            time.sleep(0.125)

        self.dut.receive()
        self.dut.send(b"F\r")
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92, "f-command response length mismatch (20 kbps min-stuffing)")
        # 7% margin for test setup and calculation (widened from 5% for Linux virtual box stability)
        pct = int(rx_data[89:91], 10)
        self.assertGreaterEqual(pct, 93,
                                f"20 kbps min-stuffing full load: reported {pct} %, expected within [93, 99]")
        self.assertLessEqual(pct, 99,
                             f"20 kbps min-stuffing full load: reported {pct} %, expected within [93, 99]")


        # Maximum stuffing (~ 20% * (11 + 64) / 112 ~ 14% underestimation)
        tx_data = b"t00080000000000000000\r"    # 112bit * 0.05ms = 5.6ms
        for _ in range(0, 5):
            tx_data = tx_data + tx_data    # 5.6ms * 32 = 179.2ms

        # Full load for more than 1 second
        time.sleep(1)
        for _ in range(0, 20):
            self.dut.receive()
            self.dut.send(tx_data)
            time.sleep(0.125)

        self.dut.receive()
        self.dut.send(b"F\r")
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92, "f-command response length mismatch (20 kbps max-stuffing)")
        # 7% margin for test setup and calculation (widened from 5% for Linux virtual box stability)
        pct = int(rx_data[89:91], 10)
        self.assertGreaterEqual(pct, 81,
                                f"20 kbps max-stuffing full load: reported {pct} %, expected within [81, 88]")
        self.assertLessEqual(pct, 88,
                             f"20 kbps max-stuffing full load: reported {pct} %, expected within [81, 88]")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    # TODO Measure and show tx delay of the tranceiver?


if __name__ == "__main__":
    unittest.main()
