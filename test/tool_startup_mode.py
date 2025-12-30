#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest

# Tool to set up auto startup mode

class ToolStartupModeTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_startup_mode(self):
        self.dut.print_on = True

        # Report
        self.dut.send(b"z0001\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Filter
        self.dut.send(b"W0\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"mFFFFFFFF\r")       # mFFFFFFFF -> Pass all
        self.assertEqual(self.dut.receive(), b"\r")

        # Open port
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Mode
        self.dut.send(b"Q1\r")
        time.sleep(0.1)         # Extra wait for flash update
        self.assertEqual(self.dut.receive(), b"\r")

        # Close port
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
