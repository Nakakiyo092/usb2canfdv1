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
        tx_data = b"d03FF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data)
        tx_data = b"b03FF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data)
        tx_data = b"R0137FEC8F\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data)
        tx_data = b"T0137FEC880011223344556677\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data)
        tx_data = b"D0137FEC8F00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data)
        tx_data = b"B0137FEC8F00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF00112233445566778899AABBCCDDEEFF\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data)
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_nominal_bitrate(self):
        #self.dut.print_on = True
        # Check loopback of a frame in every nominal bitrates
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
        #self.dut.print_on = True
        # Check loopback of a frame in every data bitrates
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


    # TODO Measure and show tx delay of the tranceiver?


if __name__ == "__main__":
    unittest.main()
