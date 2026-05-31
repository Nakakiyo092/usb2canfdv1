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
        """Q1-saved report mode (z1011) is restored; z0000+Z2 RAM overrides are lost.

        Frames must carry a 4-char ms timestamp.  If the RAM-only
        overrides had survived, either no reply would be received
        (Rx report off from z0000) or the timestamp would be 8 chars
        (us, from Z2) -- both caught as a length mismatch.
        """
        cmd_send_std = (b"r", b"t", b"d", b"b")

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
                    len(rx_data), len(b"z\r" + cmd + b"03F0TTTT\r"),
                    f"Expected ms timestamp for cmd {cmd!r}")
                self.assertEqual(
                    rx_data[:len(b"z\r" + cmd + b"03F0")],
                    b"z\r" + cmd + b"03F0")
            else:
                self.assertEqual(
                    len(rx_data), len(b"z\r" + cmd + b"03F0TTTTE\r"),
                    f"Expected ms timestamp + ESI for FD cmd {cmd!r}")
                self.assertEqual(
                    rx_data[:len(b"z\r" + cmd + b"03F0")],
                    b"z\r" + cmd + b"03F0")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bitrate(self):
        """Persisted nominal bitrate S0 (10 kbps) is restored.

        Sends two back-to-back FD BRS frames with a long data portion
        in internal loopback.  The long data portion ensures back-to-back
        transmission even with host-side latency, and makes the data
        bitrate (Y) contribute to the inter-frame timing as well.
        """
        # Verify auto-startup opened the channel.  Relies on unittest's default
        # alphabetical method ordering: this is the first channel-touching test
        # in this suite, so it is the only one that observes the auto-startup state.
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Channel should be open at start; "
                         "auto-startup did not open the channel after reset")
        self.dut.send(b"Z2\r")  # us timestamp for precise measurement
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # FD BRS frame: ID=0x03F (passes filter), DLC=F (64 bytes data),
        # alternating bit pattern.
        data = b"55" * 64
        tx_frame = b"b03FF" + data + b"\r"
        self.dut.send(tx_frame + tx_frame)
        time.sleep(0.1)
        rx_data = self.dut.receive()

        reply_prefix = b"b03FF" + data
        per_frame_reply_len = len(b"z\r" + reply_prefix + b"TTTTTTTTE\r")
        self.assertEqual(len(rx_data), 2 * per_frame_reply_len,
                         "Unexpected total reply length for two loopback frames")

        pos1 = len(b"z\r") + len(reply_prefix)
        ts_1st = int(rx_data[pos1:pos1 + 8], 16)
        pos2 = pos1 + len(b"TTTTTTTTE\r") + len(b"z\r") + len(reply_prefix)
        ts_2nd = int(rx_data[pos2:pos2 + 8], 16)

        diff_us = ts_2nd - ts_1st
        if diff_us < 0:
            diff_us += 3600000000  # us timestamp wraps at 60 minutes

        # At S0 (nominal 10 kbps) + default Y2 (data 2 Mbps), expected ~3500 us.
        # At S4 default (125 kbps) the same frame would take ~400 us.
        self.assertGreater(diff_us, 2500,
                           f"Inter-frame interval {diff_us} us too short; "
                           f"expected ~3500 us at S0")
        self.assertLess(diff_us, 4500,
                        f"Inter-frame interval {diff_us} us too long; "
                        f"expected ~3500 us at S0")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_filter(self):
        """Persisted 0x03F filter rejects other IDs after reset.

        With M0000003F / mFFFFF800, only frames whose lower 11 bits of
        the CAN ID equal 0x03F pass (IDE bit is don't-care).
        """
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

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
                    len(rx_data), len(b"z\r" + cmd + b"03F0TTTT\r"),
                    f"Std ID 0x03F should pass for cmd {cmd!r}")
                self.assertEqual(rx_data[0:-5], b"z\r" + cmd + b"03F0")
            else:
                self.assertEqual(
                    len(rx_data), len(b"z\r" + cmd + b"03F0TTTTE\r"),
                    f"Std ID 0x03F should pass for FD cmd {cmd!r}")
                self.assertEqual(rx_data[0:-6], b"z\r" + cmd + b"03F0")

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
                    len(rx_data), len(b"Z\r" + cmd + b"0000003F0TTTT\r"),
                    f"Ext ID 0x0000003F should pass for cmd {cmd!r}")
                self.assertEqual(rx_data[0:-5], b"Z\r" + cmd + b"0000003F0")
            else:
                self.assertEqual(
                    len(rx_data), len(b"Z\r" + cmd + b"0000003F0TTTTE\r"),
                    f"Ext ID 0x0000003F should pass for FD cmd {cmd!r}")
                self.assertEqual(rx_data[0:-6], b"Z\r" + cmd + b"0000003F0")

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
