#!/usr/bin/env python3

import unittest

from device_under_test import DeviceUnderTest


class ResetBeforeTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_setup(self):
        # Set serial number and other settings and activate auto startup mode
        self.dut.send(b"NA123\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"z1011\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"M0000003F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mFFFFF800\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Set S0 (10 kbps) to verify nominal bitrate is saved and restored by Q1.
        # Distinguishable from the S4 default (125 kbps) via timestamp difference.
        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"Q1\r")  # Auto startup enable
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Update setting in RAM to check they are overwritten
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Set Z2 (microsecond timestamp) in RAM without Q.
        # This should NOT persist after reset; the Q1-saved z1011 should be restored.
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mFFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
