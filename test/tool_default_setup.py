#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest

# Tool to get the device back to default setup

class ToolDefaultTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()
        

    def test_default(self):
        #self.dut.print_on = True

        # Bit rate
        self.dut.send(b"S4\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y2\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Timestamp
        self.dut.send(b"Z0\r")
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

        # Startup mode
        self.dut.send(b"Q0\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Close port
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
