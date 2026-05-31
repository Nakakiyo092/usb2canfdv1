#!/usr/bin/env python3
"""Tests that arrange the device state to be verified after a manual reset.

Paired with test_reset_after.py.

Run order:
    1. python -m unittest test_reset_before
    2. Manually reset the device (power cycle or hardware reset)
    3. python -m unittest test_reset_after
"""

import unittest

from device_under_test import DeviceUnderTest


class ResetBeforeTestCase(unittest.TestCase):
    """Write the persisted baseline and the RAM-only overrides.

    All settings written here are intended to be verified after a manual
    device reset.
    """

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_persist_via_q1(self):
        """Write persisted settings via Q1, then apply RAM-only overrides.

        Persisted (expected to survive reset):
            serial number, report mode z1011, filter mode W2 + 0x03F
            filter, nominal bitrate S0, data bitrate Y0.

        RAM-only (expected to be lost on reset):
            z0000 + Z2 (report flags off, us timestamp),
            S4 + Y4 (bitrate overrides),
            W0 (dual filter mode), all-pass filter.
        """
        # Persisted: serial number (N writes NVM directly, no Q required).
        self.dut.send(b"NA123\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Persisted: report mode (ms timestamp + Rx report + Tx event + ESI).
        self.dut.send(b"z1011\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Persisted: filter mode and 0x03F pass filter.
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M0000003F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mFFFFF800\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Persisted: nominal bitrate S0 (10 kbps), data bitrate Y0 (500 kbps).
        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y0\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Open and commit to non-volatile memory via Q1 (auto-startup ON).
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Q1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # RAM-only: report flags off (z0000), us timestamp (Z2),
        # bitrate overrides (S4 + Y4), dual filter mode (W0), all-pass filter.
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"S4\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y4\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"W0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mFFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
