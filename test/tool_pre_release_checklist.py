#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest

# Tool to show pre-release checklist

class ToolPreReleaseChecklistTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_pre_release_checklist(self):
        self.dut.print_on = True
        self.dut.send(b"V\r")
        self.dut.receive()
        self.dut.send(b"v\r")
        self.dut.receive()
        self.dut.send(b"I\r")
        self.dut.receive()
        self.dut.send(b"i\r")
        self.dut.receive()
        self.dut.send(b"N\r")
        self.dut.receive()


if __name__ == "__main__":
    unittest.main()
