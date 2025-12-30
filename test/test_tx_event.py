#!/usr/bin/env python3

import unittest

import time
import random
from device_under_test import DeviceUnderTest


# NOTE: This test requires another device with the default setup on CAN bus.
class TxEventTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        # close serial
        self.dut.close()


    def test_normal(self):
        #self.dut.print_on = True
        random.seed(92)
        rx_data = b""
        rx_data_exp = b""
        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        #self.dut.send(b"-1\r")
        #self.assertEqual(self.dut.receive(), b"\r")
        #self.dut.send(b"O\r")
        #self.assertEqual(self.dut.receive(), b"\r")

        for i in range(0x000, 0x800):
            if random.random() < 0.1:
                tx_data = b"b"  # +8 tx err cnt by b, -9 by t
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            else:
                tx_data = b"t"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            self.dut.send(tx_data)
            if i % 180 == 0:
                # the buffer can store as least 180 messages (4096 / 22)
                rx_data += self.dut.receive()
            time.sleep(0.001)

        # check all reply
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\r", b"")   # [CR] and tx event may swap
        rx_data_exp = rx_data_exp.replace(b"\r", b"")
        self.assertEqual(rx_data, rx_data_exp)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_no_retransmit(self):
        #self.dut.print_on = True
        random.seed(92)
        rx_data = b""
        rx_data_exp = b""
        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"-\r")
        self.assertEqual(self.dut.receive(), b"\r")
        #self.dut.send(b"-0\r")
        #self.assertEqual(self.dut.receive(), b"\r")
        #self.dut.send(b"O\r")
        #self.assertEqual(self.dut.receive(), b"\r")

        for i in range(0x000, 0x800):
            if random.random() < 0.1:
                tx_data = b"b"  # +8 tx err cnt by b, -9 by t
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            else:
                tx_data = b"t"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            self.dut.send(tx_data)
            if i % 180 == 0:
                # the buffer can store as least 180 messages (4096 / 22)
                rx_data += self.dut.receive()
            time.sleep(0.001)

        # check all reply
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\r", b"")   # [CR] and tx event may swap
        rx_data_exp = rx_data_exp.replace(b"\r", b"")
        self.assertEqual(rx_data, rx_data_exp)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_selective_error(self):
        #self.dut.print_on = True
        random.seed(92)
        rx_data = b""
        rx_data_exp = b""
        # Setup sampling point so that CBFF is OK but FBFF with BRS is not. See the link for details.
        # TODO Just a unmatched data bit rate is not enough?
        # https://github.com/Nakakiyo092/canable2-fw/discussions/72#discussioncomment-14331610
        #self.dut.send(b"s10420D0C\r")   # For CANable2
        self.dut.send(b"s08420D0C\r")   # For WeActStudio   TODO use VXXXX
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"-\r")
        self.assertEqual(self.dut.receive(), b"\r")
        #self.dut.send(b"-0\r")
        #self.assertEqual(self.dut.receive(), b"\r")
        #self.dut.send(b"O\r")
        #self.assertEqual(self.dut.receive(), b"\r")

        for i in range(0x000, 0x800):
            if random.random() < 0.1:
                tx_data = b"b"  # +8 tx err cnt by b, -9 by t
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
            else:
                tx_data = b"t"
                tx_data += format(i, "03X").encode() + b"2" + format(i, "04X").encode() + b"\r"
                rx_data_exp += b"\r" + b"z" + tx_data
            self.dut.send(tx_data)
            if i % 180 == 0:
                # the buffer can store as least 180 messages (4096 / 22)
                rx_data += self.dut.receive()
            time.sleep(0.001)

        # check all reply
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\r", b"")   # [CR] and tx event may swap
        rx_data_exp = rx_data_exp.replace(b"\r", b"")
        self.assertEqual(rx_data, rx_data_exp)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F80\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
