#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class ExLoopbackTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_external_loopback(self):
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check loopback of shortest frames of each type
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check loopback of longest frames of each type
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")
        tx_data = b"r03FF\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data)
        tx_data = b"t03F80011223344556677\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data)
        tx_data = b"d03FF" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data)
        tx_data = b"b03FF" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data)
        tx_data = b"R0137FEC8F\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data)
        tx_data = b"T0137FEC880011223344556677\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data)
        tx_data = b"D0137FEC8F" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data)
        tx_data = b"B0137FEC8F" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data)
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_nominal_bitrate(self):
        """Check loopback of a frame in every nominal bitrates"""
        #self.dut.print_on = True
        for rate in range(0, 9):
            cmd = "S" + str(rate) + "\r"
            self.dut.send(cmd.encode())
            self.assertEqual(self.dut.receive(), b"\r")
            self.dut.send(b"+\r")
            self.assertEqual(self.dut.receive(), b"\r")
            tx_data = b"b03F80011223344556677\r"
            self.dut.send(tx_data)
            self.assertEqual(self.dut.receive(), b"z\r" + tx_data)
            self.dut.send(b"C\r")
            self.assertEqual(self.dut.receive(), b"\r")


    def test_data_bitrate(self):
        """Check loopback of a frame in every data bitrates"""
        #self.dut.print_on = True
        for rate in (0, 1, 2, 4, 5):
            cmd = "Y" + str(rate) + "\r"
            self.dut.send(cmd.encode())
            self.assertEqual(self.dut.receive(), b"\r")
            self.dut.send(b"+\r")
            self.assertEqual(self.dut.receive(), b"\r")
            tx_data = b"b03F80011223344556677\r"
            self.dut.send(tx_data)
            self.assertEqual(self.dut.receive(), b"z\r" + tx_data)
            self.dut.send(b"C\r")
            self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_load_stepwise(self):
        """Check bus load with stepwise increasing load"""
        #self.dut.print_on = True

        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")

        tx_data = b"t55585555555555555555\r"    # 112bit * 0.1ms = 11.2ms
        for _ in range(0, 4):
            tx_data = tx_data + tx_data    # 11.2ms * 16 = 179.2ms

        # NOTE: The 10% point accuracy has no reasoning.
        # It is just to give some margin for the inaccurate bus load creation.

        # Check bus load in 0% mode (prove 10% point accuracy)
        time.sleep(1)
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92)
        self.assertGreaterEqual(int(rx_data[89:91], 10), 0)
        self.assertLessEqual(int(rx_data[89:91], 10), 10)

        # Check bus load in 18% mode (prove 10% point accuracy)
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(1)
        self.dut.send(tx_data)
        time.sleep(0.5)
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92)
        self.assertGreaterEqual(int(rx_data[89:91], 10), 8)
        self.assertLessEqual(int(rx_data[89:91], 10), 28)

        # Check bus load in 36% mode (prove 10% point accuracy)
        tx_data = tx_data + tx_data
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(1)
        self.dut.send(tx_data)
        time.sleep(0.5)
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92)
        self.assertGreaterEqual(int(rx_data[89:91], 10), 26)
        self.assertLessEqual(int(rx_data[89:91], 10), 46)

        # Check bus load in 72% mode (prove 10% point accuracy)
        #tx_data = tx_data + tx_data     # Large chunk may be not sent correctly
        time.sleep(0.25)
        self.dut.send(tx_data)
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(0.5)
        self.dut.send(tx_data)
        time.sleep(0.25)
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92)
        self.assertGreaterEqual(int(rx_data[89:91], 10), 62)
        self.assertLessEqual(int(rx_data[89:91], 10), 82)

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_load_full_10k(self):
        """Check bus load with full load at 10kbps"""
        #self.dut.print_on = True

        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Minimum stuffing
        tx_data = b"t55585555555555555555\r"    # 112bit * 0.1ms = 11.2ms
        for _ in range(0, 5):
            tx_data = tx_data + tx_data    # 11.2ms * 32 = 358.4ms

        # Full load for more than 1 second
        time.sleep(1)
        for _ in range(0, 10):
            self.dut.receive()
            self.dut.send(tx_data)
            time.sleep(0.25)

        self.dut.receive()
        self.dut.send(b"F\r")
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92)
        # 5% margin for test setup and calculation
        self.assertGreaterEqual(int(rx_data[89:91], 10), 95)
        self.assertLessEqual(int(rx_data[89:91], 10), 99)


        # Maximum stuffing (~ 20% * (11 + 64) / 112 ~ 14% underestimation)
        tx_data = b"t00080000000000000000\r"    # 112bit * 0.1ms = 11.2ms
        for _ in range(0, 5):
            tx_data = tx_data + tx_data    # 11.2ms * 32 = 358.4ms

        # Full load for more than 1 second
        time.sleep(1)
        for _ in range(0, 10):
            self.dut.receive()
            self.dut.send(tx_data)
            time.sleep(0.25)

        self.dut.receive()
        self.dut.send(b"F\r")
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92)
        # 5% margin for test setup and calculation
        self.assertGreaterEqual(int(rx_data[89:91], 10), 83)
        self.assertLessEqual(int(rx_data[89:91], 10), 88)

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_load_full_20k(self):
        """Check bus load with full load at 20kbps"""
        #self.dut.print_on = True

        self.dut.send(b"S1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"+\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Minimum stuffing
        tx_data = b"t55585555555555555555\r"    # 112bit * 0.05ms = 5.6ms
        for _ in range(0, 5):
            tx_data = tx_data + tx_data    # 5.6ms * 32 = 179.2ms

        # Full load for more than 1 second
        time.sleep(1)
        for _ in range(0, 20):
            self.dut.receive()
            self.dut.send(tx_data)
            time.sleep(0.125)

        self.dut.receive()
        self.dut.send(b"F\r")
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92)
        # 5% margin for test setup and calculation
        self.assertGreaterEqual(int(rx_data[89:91], 10), 95)
        self.assertLessEqual(int(rx_data[89:91], 10), 99)


        # Maximum stuffing (~ 20% * (11 + 64) / 112 ~ 14% underestimation)
        tx_data = b"t00080000000000000000\r"    # 112bit * 0.05ms = 5.6ms
        for _ in range(0, 5):
            tx_data = tx_data + tx_data    # 5.6ms * 32 = 179.2ms

        # Full load for more than 1 second
        time.sleep(1)
        for _ in range(0, 20):
            self.dut.receive()
            self.dut.send(tx_data)
            time.sleep(0.125)

        self.dut.receive()
        self.dut.send(b"F\r")
        rx_data = self.dut.receive()
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 92)
        # 5% margin for test setup and calculation
        self.assertGreaterEqual(int(rx_data[89:91], 10), 83)
        self.assertLessEqual(int(rx_data[89:91], 10), 88)

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    # TODO Measure and show tx delay of the tranceiver?


if __name__ == "__main__":
    unittest.main()
