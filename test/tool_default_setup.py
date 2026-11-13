#!/usr/bin/env python3

import re
import unittest

from device_under_test import DeviceUnderTest

# Tool to get the device back to default setup

# Serial number (N) of each known device, keyed by the unique ID (uid in the n reply).
# Tests overwrite the serial number, so this tool writes it back.
# A device not listed here gets NA123 only if no serial number is stored.
DEVICE_SERIALS = {
    # "0x0123456789ABCDEF01234567": b"3C01",
}

class ToolDefaultTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_default(self):
        #self.dut.print_on = True

        # Serial number: restore the one of a known device,
        # otherwise write NA123 if not already stored
        self.dut.send(b"n\r")
        match = re.search(rb"uid=(0x[0-9A-F]+)", self.dut.receive())
        uid = match.group(1).decode() if match else None
        self.dut.send(b"N\r")
        serial = self.dut.receive()
        if uid in DEVICE_SERIALS:
            if serial != b"N" + DEVICE_SERIALS[uid] + b"\r":
                self.dut.send(b"N" + DEVICE_SERIALS[uid] + b"\r")
                self.assertEqual(self.dut.receive(), b"\r")
        else:
            print(f"NOTE: Unique ID {uid} is not listed in DEVICE_SERIALS")
            if serial == b"\a":
                self.dut.send(b"NA123\r")
                self.assertEqual(self.dut.receive(), b"\r")

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
