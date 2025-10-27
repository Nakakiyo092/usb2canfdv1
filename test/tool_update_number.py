#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest

# Tool to update serial number

class ToolUpdateNumberTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_serial_number(self):
        self.dut.print_on = True
        # Update serial number
        #self.dut.send(b"NEC01\r")
        self.dut.send(b"N3C01\r")
        #self.dut.send(b"NA11E\r")
        time.sleep(0.1)         # Extra wait for flash update
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
