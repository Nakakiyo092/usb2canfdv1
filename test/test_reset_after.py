#!/usr/bin/env python3
"""Tests verifying persisted settings are restored after a device reset.

Paired with test_reset_before.py.

Note: setUp deliberately does NOT call self.dut.setup(); the whole point
is to verify the device auto-starts correctly from non-volatile memory.
"""

import unittest

import time
from device_under_test import DeviceUnderTest


class ResetAfterTestCase(unittest.TestCase):
    """Verify auto-startup behavior and persisted settings after reset."""

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        # Do NOT call self.dut.setup(); the test verifies auto-startup.
        self.dut = DeviceUnderTest()
        self.dut.open()


    def tearDown(self):
        self.dut.close()


    def test_serial(self):
        """Serial number persists across reset (N writes NVM directly)."""
        self.dut.send(b"N\r")
        self.assertEqual(self.dut.receive(), b"NA123\r",
                         "Serial number was not preserved across reset")


    def test_timestamp(self):
        """Q1-saved report mode (z2011) is restored; z0000+Z1 RAM overrides are lost.

        Frames must carry an 8-char us timestamp with ESI on FD frames.
        If the RAM-only overrides had survived, the timestamp would be
        4 chars (ms, from Z1) and ESI would be absent (Z resets flags
        to Rx-only) -- both caught as a length mismatch.
        """
        cmd_send_std = (b"r", b"t", b"d", b"b")

        #self.dut.print_on = True

        # Don't assert on the reply: only the first test sees the auto-startup-opened
        # channel; subsequent tests start with the channel already closed.
        self.dut.send(b"C\r")
        self.dut.receive()
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            rx_data = self.dut.receive()
            if cmd in (b"r", b"t"):
                self.assertEqual(
                    len(rx_data), len(b"z\r" + cmd + b"03F0TTTTTTTT\r"),
                    f"Expected us timestamp for cmd {cmd!r}")
                self.assertEqual(
                    rx_data[:len(b"z\r" + cmd + b"03F0")],
                    b"z\r" + cmd + b"03F0")
            else:
                self.assertEqual(
                    len(rx_data), len(b"z\r" + cmd + b"03F0TTTTTTTTE\r"),
                    f"Expected us timestamp + ESI for FD cmd {cmd!r}")
                self.assertEqual(
                    rx_data[:len(b"z\r" + cmd + b"03F0")],
                    b"z\r" + cmd + b"03F0")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bitrate(self):
        """Persisted nominal + data bitrates (S0 + Y0) are restored.

        Sends two back-to-back FD BRS frames with a long data portion
        in internal loopback.  The long data portion ensures back-to-back
        transmission even with host-side latency, and makes the data
        bitrate (Y) contribute to the inter-frame timing, so both S and
        Y persistence are verified by the single measurement.
        """
        #self.dut.print_on = True

        # Verify auto-startup opened the channel.  Relies on unittest's default
        # alphabetical method ordering: this is the first channel-touching test
        # in this suite, so it is the only one that observes the auto-startup state.
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Channel should be open at start; "
                         "auto-startup did not open the channel after reset")
        # us timestamp is already active (Z2 restored from NVM by auto-startup).
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # FD BRS frame: ID=0x03F (passes filter; the 0...01...1 pattern
        # forces 2 stuff bits in the arbitration phase, unavoidable),
        # DLC=F (64 bytes data), alternating bit pattern (no dynamic stuff
        # in the data phase).
        data = b"55" * 64
        tx_frame = b"b03FF" + data + b"\r"
        self.dut.send(tx_frame + tx_frame)
        time.sleep(0.1)
        rx_data = self.dut.receive()

        reply_prefix = b"b03FF" + data
        per_frame_reply_len = len(b"z\r" + reply_prefix + b"TTTTTTTTE\r")
        self.assertEqual(len(rx_data), 2 * per_frame_reply_len,
                         "Unexpected total reply length for two loopback frames")

        # The firmware sends both tx receipts ("z\r") first, then both
        # loopback frame echoes back-to-back: z\r z\r b03FF<data>ts E\r b03FF<data>ts E\r
        pos1 = 2 * len(b"z\r") + len(reply_prefix)
        ts_1st = int(rx_data[pos1:pos1 + 8], 16)
        pos2 = pos1 + len(b"TTTTTTTTE\r") + len(reply_prefix)
        ts_2nd = int(rx_data[pos2:pos2 + 8], 16)

        diff_us = ts_2nd - ts_1st
        if diff_us < 0:
            diff_us += 3600000000  # us timestamp wraps at 60 minutes

        # Expected ~4300 us at S0 (10 kbps) + Y0 (500 kbps).  Any RAM-leftover
        # leak scenario produces a much shorter interval:
        # - Y2 default leak (S persisted, Y not): ~3470 us
        # - Y4 RAM leak (S persisted, Y RAM): ~3340 us
        # - S4 RAM leak (S RAM, Y persisted): ~1350 us
        # - Both RAM leak (S RAM, Y RAM): ~390 us
        # Tolerance 4000-5200 us cleanly distinguishes correct from any leak.
        self.assertGreater(diff_us, 4000,
                           f"Inter-frame interval {diff_us} us too short; "
                           f"expected ~4300 us at S0+Y0 (NVM-restored)")
        self.assertLess(diff_us, 5200,
                        f"Inter-frame interval {diff_us} us too long; "
                        f"expected ~4300 us at S0+Y0 (NVM-restored)")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_filter(self):
        """Persisted 0x03F filter rejects other IDs after reset.

        With M0000003F / mFFFFF800, only frames whose lower 11 bits of
        the CAN ID equal 0x03F pass (IDE bit is don't-care).
        """
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        #self.dut.print_on = True

        # Don't assert on the reply: only the first test sees the auto-startup-opened
        # channel; subsequent tests start with the channel already closed.
        self.dut.send(b"C\r")
        self.dut.receive()
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Standard IDs
        for cmd in cmd_send_std:
            # Pass: std ID 0x03F matches the filter.
            self.dut.send(cmd + b"03F0\r")
            rx_data = self.dut.receive()
            if cmd in (b"r", b"t"):
                self.assertEqual(
                    len(rx_data), len(b"z\r" + cmd + b"03F0TTTTTTTT\r"),
                    f"Std ID 0x03F should pass for cmd {cmd!r}")
                self.assertEqual(rx_data[0:-9], b"z\r" + cmd + b"03F0")
            else:
                self.assertEqual(
                    len(rx_data), len(b"z\r" + cmd + b"03F0TTTTTTTTE\r"),
                    f"Std ID 0x03F should pass for FD cmd {cmd!r}")
                self.assertEqual(rx_data[0:-10], b"z\r" + cmd + b"03F0")

            # Reject: std IDs other than 0x03F.
            for reject_id in (b"7C00", b"43F0", b"03E0"):
                self.dut.send(cmd + reject_id + b"\r")
                self.assertEqual(
                    self.dut.receive(), b"z\r",
                    f"Filter should reject {(cmd + reject_id)!r}")

        # Extended IDs
        for cmd in cmd_send_ext:
            # Pass: ext ID whose lower 11 bits == 0x03F.
            self.dut.send(cmd + b"0000003F0\r")
            rx_data = self.dut.receive()
            if cmd in (b"R", b"T"):
                self.assertEqual(
                    len(rx_data), len(b"Z\r" + cmd + b"0000003F0TTTTTTTT\r"),
                    f"Ext ID 0x0000003F should pass for cmd {cmd!r}")
                self.assertEqual(rx_data[0:-9], b"Z\r" + cmd + b"0000003F0")
            else:
                self.assertEqual(
                    len(rx_data), len(b"Z\r" + cmd + b"0000003F0TTTTTTTTE\r"),
                    f"Ext ID 0x0000003F should pass for FD cmd {cmd!r}")
                self.assertEqual(rx_data[0:-10], b"Z\r" + cmd + b"0000003F0")

            # Reject: ext IDs whose lower 11 bits != 0x03F.
            for reject_id in (b"000007C00", b"0137FEC80", b"1EC801370"):
                self.dut.send(cmd + reject_id + b"\r")
                self.assertEqual(
                    self.dut.receive(), b"Z\r",
                    f"Filter should reject {(cmd + reject_id)!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
