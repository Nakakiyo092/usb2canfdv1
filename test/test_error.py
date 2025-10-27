#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class ErrorTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_error_active(self):
        #self.dut.print_on = True

        # Check error active in normal operation
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"f: node_sts=ER_ACTV, last_err_code=NONE, err_cnt_tx_rx=[0x00, 0x00], th_bus_load_percent=00\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\rt0000\r")
        time.sleep(0.2)     # wait for a while ( > 1ms * 1)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"f: node_sts=ER_ACTV, last_err_code=NONE, err_cnt_tx_rx=[0x00, 0x00], th_bus_load_percent=00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_error(self):
        #self.dut.print_on = True

        # Check no ack error is reported
        self.dut.send(b"-\r")   # No retransmit mode
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # wait for a while ( > 1ms * 1)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F80\r")  # BEI
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")  # check error clear
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"f: node_sts=ER_ACTV, last_err_code=_ACK, err_cnt_tx_rx=[0x08, 0x00], th_bus_load_percent=00\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")
        

    def test_error_warning(self):
        #self.dut.print_on = True

        # Check error warning level is reported
        self.dut.send(b"-\r")   # No retransmit mode
        self.assertEqual(self.dut.receive(), b"\r")
        for i in range(0, 12):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # wait for a while ( > 1ms * 12)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F84\r")  # BEI + EI
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")  # check error clear
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"f: node_sts=ER_ACTV, last_err_code=_ACK, err_cnt_tx_rx=[0x60, 0x00], th_bus_load_percent=00\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_error_passive(self):
        #self.dut.print_on = True

        # Check error passive status is reported
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"f: node_sts=ER_ACTV, last_err_code=NONE, err_cnt_tx_rx=[0x00, 0x00], th_bus_load_percent=00\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # wait for error passive ( > 1ms * 128)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r")  # BEI & EPI & EI
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")  # check error clear
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"f: node_sts=ER_PSSV, last_err_code=_ACK, err_cnt_tx_rx=[0x80, 0x00], th_bus_load_percent=00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_error_passive_clear(self):
        #self.dut.print_on = True

        # Check error passive status is reported after status flags are cleared
        # Check error warning level is not reported after exceeding error warning level
        self.dut.send(b"-\r")   # No retransmit mode
        self.assertEqual(self.dut.receive(), b"\r")
        
        for i in range(0, 12):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # wait for a while ( > 1ms * 12)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F84\r")  # BEI + EI
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")  # check error clear
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"f: node_sts=ER_ACTV, last_err_code=_ACK, err_cnt_tx_rx=[0x60, 0x00], th_bus_load_percent=00\r")

        for i in range(0, 4):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # wait for a while ( > 1ms * 4)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA0\r")  # EPI + EI
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")  # check error clear
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"f: node_sts=ER_PSSV, last_err_code=_ACK, err_cnt_tx_rx=[0x80, 0x00], th_bus_load_percent=00\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_cdc_tx_overflow(self):
        #self.dut.print_on = True

        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Confirm no error at initial state
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        #  send a lot of command without receiving data (amount depends on PC env.)
        for i in range(0, 400):
            self.dut.send(b"v\r")
            time.sleep(0.001)

        #  recieve all reply
        rx_data = self.dut.receive()

        # Check CAN Rx Full error is reported for CDC Tx overflow
        self.dut.send(b"F\r")
        #  CAN Rx Full (because this type of overflow is caused typically by too many can frame)
        self.assertEqual(self.dut.receive(), b"F01\r")

        # Check error clear
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_can_rx_overflow(self):
        #self.dut.print_on = True

        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Confirm no error at initial state
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        #  the buffer can store as least 180 messages (4096 / 24)
        for i in range(0, 180):
            self.dut.send(b"t03F80011223344556677\r")
            time.sleep(0.001)

        #  recieve all reply
        rx_data = self.dut.receive()
        rx_data = self.dut.receive()    # just to make sure

        # Confirm no error
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        # TODO: This process creates overflow in CDC Tx buffer not in CAN Rx buffer
        #  the buffer can not store 800 messages (amount depends on PC env.)
        for i in range(0, 800):
            self.dut.send(b"t03F80011223344556677\r")
            time.sleep(0.001)

        #  recieve all reply
        rx_data = self.dut.receive()
        rx_data = self.dut.receive()    # just to make sure

        # Check CAN Rx Full error is reported for CDC Tx overflow
        self.dut.send(b"F\r")
        #  CAN Rx Full (because this type of overflow is caused typically by too many can frame)
        self.assertEqual(self.dut.receive(), b"F01\r")

        # Check error clear
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_can_tx_overflow(self):
        #self.dut.print_on = True

        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Confirm no error at initial state
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        #  the buffer can store as least 64 messages
        for i in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")

        # Confirm no overflow
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r")  # BEI & EPI & EI

        #  the buffer can not store additional 64 messages
        for i in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.dut.receive()

        # Check CAN Tx Full error is reported for CAN Tx overflow
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F02\r")

        # Check a [BELL] is sent for each transmitted frame when CAN Tx overflow occurs
        for i in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.assertEqual(self.dut.receive(), b"\a")

        # Check error
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F02\r")

        # Check error clear
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_no_retransmit(self):
        #self.dut.print_on = True

        self.dut.send(b"-\r")
        self.assertEqual(self.dut.receive(), b"\r")
        #self.dut.send(b"-0\r")
        #self.assertEqual(self.dut.receive(), b"\r")
        #self.dut.send(b"O\r")
        #self.assertEqual(self.dut.receive(), b"\r")

        # Confirm no error at initial state
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        #  will be discarded by no ack
        for i in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")

        # Confirm no overflow error
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r")  # BEI & EPI & EI
        self.dut.send(b"f\r")
        self.dut.receive()

        #  will be discarded by no ack
        for i in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.dut.receive()

        # Confirm no overflow error
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")  # TEC saturates by 16 frames

        #  will be discarded by no ack
        for i in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")

        # Confirm no overflow error
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")  # TEC saturates by 16 frames

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
