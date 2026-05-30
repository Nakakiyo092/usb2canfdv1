#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class ResetAfterTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        # DO NOT SETUP! CHECK AUTO SETUP!


    def tearDown(self):
        self.dut.close()


    def test_serial(self):
        # Check serial number is stored after reset
        self.dut.send(b"N\r")
        self.assertEqual(self.dut.receive(), b"NA123\r")


    def test_timestamp(self):
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        #cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check timestamp is still on after reset
        self.dut.send(b"C\r")
        self.dut.receive()
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            rx_data = self.dut.receive()
            if cmd in (b"r", b"t"):
                self.assertEqual(len(rx_data), len(b"z\r" + cmd + b"03F0TTTT\r"))
                self.assertEqual(rx_data[:len(b"z\r" + cmd + b"03F0")], b"z\r" + cmd + b"03F0")
            else:
                self.assertEqual(len(rx_data), len(b"z\r" + cmd + b"03F0TTTTE\r"))
                self.assertEqual(rx_data[:len(b"z\r" + cmd + b"03F0")], b"z\r" + cmd + b"03F0")

        # Not work due to filter
        #for cmd in cmd_send_ext:
        #    self.dut.send(cmd + b"0137FEC80\r")
        #    rx_data = self.dut.receive()
        #    self.assertEqual(len(rx_data), len(b"Z\r" + cmd + b"0137FEC80TTTT\r"))
        #    self.assertEqual(rx_data[:len(b"Z\r" + cmd + b"0137FEC80")], b"Z\r" + cmd + b"0137FEC80")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bitrate(self):
        """Check that nominal bitrate (S) is stored and restored after reset.

        S0 (10 kbps) was saved via Q1 in test_reset_before.  After reset the
        device auto-starts at 10 kbps.  Two back-to-back frames are sent in
        internal loopback mode; the microsecond timestamp difference between
        their start-of-frame events must be consistent with a 10 kbps bitrate
        (approx. 4700 us), not the 125 kbps default (~376 us).
        """
        #self.dut.print_on = True
        self.dut.send(b"C\r")
        self.dut.receive()
        # Enable microsecond timestamp in RAM (not saved) for precise measurement
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Send two frames back-to-back and read both loopback reports
        tx_frame = b"t03F0\r"
        self.dut.send(tx_frame + tx_frame)
        time.sleep(0.1)
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 2 * (len(b"z\r") + len(b"t03F0") + len(b"TTTTTTTT\r")))

        # Extract start-of-frame timestamps (us timestamp, 8 hex chars each)
        pos1 = len(b"z\r") + len(b"t03F0")
        ts_1st = int(rx_data[pos1 : pos1 + 8], 16)

        pos2 = pos1 + len(b"TTTTTTTT\r") + len(b"z\r") + len(b"t03F0")
        ts_2nd = int(rx_data[pos2 : pos2 + 8], 16)

        diff_us = ts_2nd - ts_1st
        if diff_us < 0:
            diff_us += 3600000000  # wrap-around

        # Standard CAN frame (44 bits + 3 IFS) at 10 kbps ≈ 4700 us.
        # At S4 (125 kbps default) the same frame takes only ~376 us.
        # Allow ±50% tolerance to handle stuffing bits and clock accuracy.
        self.assertGreater(diff_us, 3000,
                           f"Bitrate too fast for saved S0 (10 kbps): diff={diff_us} us")
        self.assertLess(diff_us, 7000,
                        f"Bitrate too slow for saved S0 (10 kbps): diff={diff_us} us")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_Z_without_Q_no_persist(self):
        """Check that Z command without Q does not write non-volatile memory.

        Z2 (microsecond timestamp) was set in RAM in test_reset_before without
        a subsequent Q call, so it must NOT survive the reset.  The Q1-saved
        setting (z1011) uses millisecond timestamp; after reset frames should
        carry 4-char timestamps, not the 8-char microsecond ones.
        """
        #self.dut.print_on = True
        self.dut.send(b"C\r")
        self.dut.receive()
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # ms timestamp (z1011): z[CR] + t03F0 + TTTT + [CR]  = 12 bytes
        # us timestamp (Z2)   : z[CR] + t03F0 + TTTTTTTT + [CR] = 16 bytes
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"z\rt03F0TTTT\r"),
                         "Z2 without Q must not persist: expected 4-char ms timestamp")
        self.assertEqual(rx_data[:len(b"z\rt03F0")], b"z\rt03F0")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_filter(self):
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check pass 0x03F filter is still active after reset
        self.dut.send(b"C\r")
        self.dut.receive()
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            rx_data = self.dut.receive()
            if cmd in (b"r", b"t"):
                self.assertEqual(len(rx_data), len(b"z\r" + cmd + b"03F0TTTT\r"))
                self.assertEqual(rx_data[0:-5], b"z\r" + cmd + b"03F0")
            else:
                self.assertEqual(len(rx_data), len(b"z\r" + cmd + b"03F0TTTTE\r"))
                self.assertEqual(rx_data[0:-6], b"z\r" + cmd + b"03F0")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"43F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"03E0\r")
            self.assertEqual(self.dut.receive(), b"z\r")

        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            rx_data = self.dut.receive()
            if cmd in (b"R", b"T"):
                self.assertEqual(len(rx_data), len(b"Z\r" + cmd + b"0000003F0TTTT\r"))
                self.assertEqual(rx_data[0:-5], b"Z\r" + cmd + b"0000003F0")
            else:
                self.assertEqual(len(rx_data), len(b"Z\r" + cmd + b"0000003F0TTTTE\r"))
                self.assertEqual(rx_data[0:-6], b"Z\r" + cmd + b"0000003F0")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
