#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class InLoopbackTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_internal_loopback(self):
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check loopback of shortest frames of each type
        self.dut.send(b"=\r")
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
        self.dut.send(b"=\r")
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


    def test_timestamp_milli(self):
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check no timestamp when timestamp is off
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check millisec timestamp when timestamp is on
        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            rx_data = self.dut.receive()
            self.assertEqual(len(rx_data), len(b"z\r" + cmd + b"03F0TTTT\r"))
            self.assertEqual(rx_data[:len(b"z\r" + cmd + b"03F0")], b"z\r" + cmd + b"03F0")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            rx_data = self.dut.receive()
            self.assertEqual(len(rx_data), len(b"Z\r" + cmd + b"0137FEC80TTTT\r"))
            self.assertEqual(rx_data[:len(b"Z\r" + cmd + b"0137FEC80")], b"Z\r" + cmd + b"0137FEC80")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check no timestamp when timestamp is back to off
        self.dut.send(b"Z0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check timestamp accuracy
        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        
        #  send first frame and get timestamp
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"z\r" + b"t03F0TTTT\r"))
        self.assertEqual(rx_data[:len(b"z\r" + b"t03F0")], b"z\r" + b"t03F0")
        last_timestamp = rx_data[len(b"z\r" + b"t03F0"):len(b"z\r" + b"t03F0") + 4]
        last_time_ms = int(last_timestamp.decode(), 16)

        #  sleep for a while
        sleep_time_ms = 30 * 1000
        time.sleep(sleep_time_ms / 1000.0)

        #  send second frame and get timestamp
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"z\r" + b"t03F0TTTT\r"))
        self.assertEqual(rx_data[:len(b"z\r" + b"t03F0")], b"z\r" + b"t03F0")
        crnt_timestamp = rx_data[len(b"z\r" + b"t03F0"):len(b"z\r" + b"t03F0") + 4]
        crnt_time_ms = int(crnt_timestamp.decode(), 16)
        if crnt_time_ms > last_time_ms:
            diff_time_ms = crnt_time_ms - last_time_ms
        else:
            diff_time_ms = (60000 + crnt_time_ms) - last_time_ms

        # Proving 2% accuracy. 600ms should be enough for USB latency.
        self.assertLess(abs(sleep_time_ms - diff_time_ms), 600)
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_micro(self):
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check no timestamp when timestamp is off
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check microsec timestamp when timestamp is on
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            rx_data = self.dut.receive()
            self.assertEqual(len(rx_data), len(b"z\r" + cmd + b"03F0TTTTTTTT\r"))
            self.assertEqual(rx_data[:len(b"z\r" + cmd + b"03F0")], b"z\r" + cmd + b"03F0")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            rx_data = self.dut.receive()
            self.assertEqual(len(rx_data), len(b"Z\r" + cmd + b"0137FEC80TTTTTTTT\r"))
            self.assertEqual(rx_data[:len(b"Z\r" + cmd + b"0137FEC80")], b"Z\r" + cmd + b"0137FEC80")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check no timestamp when timestamp is back to off
        self.dut.send(b"Z0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check timestamp accuracy
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        #  send first frame and get timestamp
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"z\r" + b"t03F0TTTTTTTT\r"))
        self.assertEqual(rx_data[:len(b"z\r" + b"t03F0")], b"z\r" + b"t03F0")
        last_timestamp = rx_data[len(b"z\r" + b"t03F0"):len(b"z\r" + b"t03F0") + 8]
        last_time_us = int(last_timestamp.decode(), 16)

        #  sleep for a while
        sleep_time_us = 30 * 1000 * 1000
        time.sleep(sleep_time_us / 1000.0 / 1000.0)

        #  send second frame and get timestamp
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"z\r" + b"t03F0TTTTTTTT\r"))
        self.assertEqual(rx_data[:len(b"z\r" + b"t03F0")], b"z\r" + b"t03F0")
        crnt_timestamp = rx_data[len(b"z\r" + b"t03F0"):len(b"z\r" + b"t03F0") + 8]
        crnt_time_us = int(crnt_timestamp.decode(), 16)
        if crnt_time_us > last_time_us:
            diff_time_us = crnt_time_us - last_time_us
        else:
            diff_time_us = (3600000000 + crnt_time_us) - last_time_us

        # Proving 2% accuracy. 600ms should be enough for USB latency.
        self.assertLess(abs(sleep_time_us - diff_time_us), 600 * 1000)
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_same_stamp(self):
        #self.dut.print_on = True

        # Check Rx frame and Tx event have the same timestamp in CAN loopback mode
        self.dut.send(b"z2003\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"\r" + b"zt03F0TTTTTTTT\r" + b"t03F0TTTTTTTT\r"))
        if rx_data[1] == b"z"[0]:
            tx_timestamp = rx_data[len(b"\rzt03F0"):len(b"\rzt03F0") + 8]
        else:
            tx_timestamp = rx_data[len(b"\rt03F0TTTTTTTT\rzt03F0"):len(b"\rt03F0TTTTTTTT\rzt03F0") + 8]
        if rx_data[1] == b"z"[0]:
            rx_timestamp = rx_data[len(b"\rzt03F0TTTTTTTT\rt03F0"):len(b"\rzt03F0TTTTTTTT\rt03F0") + 8]
        else:
            rx_timestamp = rx_data[len(b"\rt03F0"):len(b"\rt03F0") + 8]
        self.assertEqual(tx_timestamp, rx_timestamp)
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_consistency(self):
        #self.dut.print_on = True

        # Compare timestamp for a Z[CR] command and for a received CAN frame
        self.dut.send(b"S8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z2001\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z\rt03F0\r")
        rx_data = self.dut.receive() + self.dut.receive()
        last_timestamp = rx_data[len(b"Z2"):len(b"Z2") + 8]
        last_time_us = int(last_timestamp.decode(), 16)
        crnt_timestamp = rx_data[len(b"Z2XXXXXXXX\rz\rt03F0"):len(b"Z2XXXXXXXX\rz\rt03F0") + 8]
        crnt_time_us = int(crnt_timestamp.decode(), 16)
        if crnt_time_us > last_time_us:
            diff_time_us = crnt_time_us - last_time_us
        else:
            diff_time_us = (3600000000 + crnt_time_us) - last_time_us

        # Difference should be less than the length of the frame (~50us) + device main loop cycle (~100us)
        self.assertLess(diff_time_us, 200)
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_accuracy_milli(self):
        #self.dut.print_on = True

        # Configure a CAN bus with the slowest bitrates
        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check timestamp difference for two frames sent consecutively
        tx_frame = b"t55585555555555555555" # Minimize stuffing bits
        self.dut.send(tx_frame + b"\r" + tx_frame + b"\r")
        time.sleep(0.1)
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 2 * (len(b"z\r") + len(tx_frame) + len(b"TTTT\r")))

        pos = 2 * len(b"z\r") + len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 4]

        pos = 2 * len(b"z\r") + len(tx_frame) + len(b"TTTT\r") + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 4]

        time_exp_us = (int(timestamp_1st, 16) * 1000 + (47 + 8 * 8 + 1) * 100) % 60000000   # 1 stuff bit?
        self.assertLess(abs(time_exp_us - int(timestamp_2nd, 16) * 1000), 1000)

        # Check timestamp difference for two frames with BRS sent consecutively
        tx_frame = b"B1555555585555555555555555" # Minimize stuffing bits
        self.dut.send(tx_frame + b"\r" + tx_frame + b"\r")
        time.sleep(0.1)
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 2 * (len(b"Z\r") + len(tx_frame) + len(b"TTTT\r")))

        pos = 2 * len(b"Z\r") + len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 4]

        pos = 2 * len(b"Z\r") + len(tx_frame) + len(b"TTTT\r") + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 4]

        time_exp_us = (int(timestamp_1st, 16) * 1000 + 49 * 100 + 8 * 8 + 26 + 7) % 60000000   # 7 stuff bits?
        self.assertLess(abs(time_exp_us - int(timestamp_2nd, 16) * 1000), 1000)

        # Close port
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_accuracy_micro(self):
        #self.dut.print_on = True

        # Configure a CAN bus with the slowest bitrates
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check timestamp difference for two frames sent consecutively
        tx_frame = b"t55585555555555555555" # Minimize stuffing bits
        self.dut.send(tx_frame + b"\r" + tx_frame + b"\r")
        time.sleep(0.1)
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 2 * (len(b"z\r") + len(tx_frame) + len(b"TTTTTTTT\r")))

        pos = 2 * len(b"z\r") + len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 8]

        pos = 2 * len(b"z\r") + len(tx_frame) + len(b"TTTTTTTT\r") + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 8]

        time_exp_us = (int(timestamp_1st, 16) + (47 + 8 * 8 + 1) * 100) % 3600000000   # 1 stuff bit?
        self.assertEqual(time_exp_us, int(timestamp_2nd, 16))

        # Check timestamp difference for two frames with BRS sent consecutively
        tx_frame = b"B1555555585555555555555555" # Minimize stuffing bits
        self.dut.send(tx_frame + b"\r" + tx_frame + b"\r")
        time.sleep(0.1)
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 2 * (len(b"Z\r") + len(tx_frame) + len(b"TTTTTTTT\r")))

        pos = 2 * len(b"Z\r") + len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 8]

        pos = 2 * len(b"Z\r") + len(tx_frame) + len(b"TTTTTTTT\r") + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 8]

        time_exp_us = (int(timestamp_1st, 16) + 49 * 100 + 8 * 8 + 26 + 7) % 3600000000   # 7 stuff bits?
        self.assertEqual(time_exp_us, int(timestamp_2nd, 16))

        # Check timestamp difference for 20 frames sent consecutively
        tx_frame = b"t55585555555555555555"
        for i in range(0, 20):
            self.dut.send(tx_frame + b"\r")
        time.sleep(0.5)
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 20 * (len(b"z\r") + len(tx_frame) + len(b"TTTTTTTT\r")))

        rx_data = rx_data.replace(b"z\r", b"")

        pos = len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 8]

        pos = 19 * (len(tx_frame) + len(b"TTTTTTTT\r")) + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 8]

        time_exp_us = (int(timestamp_1st, 16) + 19 * (47 + 8 * 8 + 1) * 100) % 3600000000   # 1 stuff bit?
        self.assertEqual(time_exp_us, int(timestamp_2nd, 16))

        # Check timestamp difference for 20 frames with BRS sent consecutively
        tx_frame = b"B1555555585555555555555555"
        for i in range(0, 20):
            self.dut.send(tx_frame + b"\r")
        time.sleep(0.5)
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), 20 * (len(b"Z\r") + len(tx_frame) + len(b"TTTTTTTT\r")))

        rx_data = rx_data.replace(b"Z\r", b"")

        pos = len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 8]

        pos = 19 * (len(tx_frame) + len(b"TTTTTTTT\r")) + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 8]

        time_exp_us = (int(timestamp_1st, 16) + 19 * (49 * 100 + 8 * 8 + 26 + 7)) % 3600000000   # 7 stuff bits?
        self.assertEqual(time_exp_us, int(timestamp_2nd, 16))

        # Close port
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_tx_off_rx_on(self):
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check tx event is disabled and rx frame is enabled
        self.dut.send(b"z0001\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_tx_on_rx_off(self):
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check tx event is enabled and rx frame is disabled
        self.dut.send(b"z0002\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"\r" + b"z" + cmd + b"03F0\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"\r" + b"Z" + cmd + b"0137FEC80\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_esi_on(self):
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check CC frames are reported without ESI and FD frames with ESI (Rx frame and Tx event)
        # TODO: check ESI bit 0 for error active and 1 for error passive
        self.dut.send(b"z0013\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            if cmd == b"r" or cmd == b"t":
                rx_data = self.dut.receive()
                self.assertEqual(len(rx_data), len(b"\r" + b"z" + cmd + b"03F0\r" + cmd + b"03F0\r"))
                if rx_data[1] == b"z"[0]:
                    self.assertEqual(rx_data, b"\r" + b"z" + cmd + b"03F0\r" + cmd + b"03F0\r")
                else:
                    self.assertEqual(rx_data, b"\r" + cmd + b"03F0\r" + b"z" + cmd + b"03F0\r")
            else:
                rx_data = self.dut.receive()
                self.assertEqual(len(rx_data), len(b"\r" + b"z" + cmd + b"03F00\r" + cmd + b"03F00\r"))
                if rx_data[1] == b"z"[0]:
                    self.assertEqual(rx_data, b"\r" + b"z" + cmd + b"03F00\r" + cmd + b"03F00\r")
                else:
                    self.assertEqual(rx_data, b"\r" + cmd + b"03F00\r" + b"z" + cmd + b"03F00\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            if cmd == b"R" or cmd == b"T":
                rx_data = self.dut.receive()
                self.assertEqual(len(rx_data), len(b"\r" + b"Z" + cmd + b"0137FEC80\r" + cmd + b"0137FEC80\r"))
                if rx_data[1] == b"Z"[0]:
                    self.assertEqual(rx_data, b"\r" + b"Z" + cmd + b"0137FEC80\r" + cmd + b"0137FEC80\r")
                else:
                    self.assertEqual(rx_data, b"\r" + cmd + b"0137FEC80\r" + b"Z" + cmd + b"0137FEC80\r")
            else:
                rx_data = self.dut.receive()
                self.assertEqual(len(rx_data), len(b"\r" + b"Z" + cmd + b"0137FEC800\r" + cmd + b"0137FEC800\r"))
                if rx_data[1] == b"Z"[0]:
                    self.assertEqual(rx_data, b"\r" + b"Z" + cmd + b"0137FEC800\r" + cmd + b"0137FEC800\r")
                else:
                    self.assertEqual(rx_data, b"\r" + cmd + b"0137FEC800\r" + b"Z" + cmd + b"0137FEC800\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_dual_filter_basic(self):
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        #self.dut.print_on = True

        # Check pass all filter (default)
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"7C00\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0000003F0\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"000007C00\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"1EC801370\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_simple_filter_basic(self):
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        #self.dut.print_on = True

        # Check pass all filter (default)
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"7C00\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0000003F0\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"000007C00\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"1EC801370\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass 0x03F and 0x0000003F filter
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M0000003F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mFFFFF800\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0000003F0\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass 0x6C8 and 0x0137FEC8 filter
        self.dut.send(b"M0137FEC8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"mE0000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"6C80\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"6C80\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass STD ID 0x03F filter
        self.dut.send(b"M8000003F\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass EXT ID 0x0137FEC8 filter
        self.dut.send(b"M0137FEC8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"7C00\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            self.dut.send(cmd + b"6C80\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0000003F0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"000007C00\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r")
            self.dut.send(cmd + b"1EC801370\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")
        

    def test_simple_filter_every_bits(self):
        # Check pass STD ID 0x000 filter comparing every bit in CAN ID
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t0000\r")
        for idx in range(0, 11):
            self.dut.send(b"t" + (f'{(1 << idx):03X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r")
        for idx in range(0, 29):
            self.dut.send(b"T" + (f'{(1 << idx):08X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass EXT ID 0x00000000 filter comparing every bit in CAN ID
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        for idx in range(0, 11):
            self.dut.send(b"t" + (f'{(1 << idx):03X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r" + b"T000000000\r")
        for idx in range(0, 29):
            self.dut.send(b"T" + (f'{(1 << idx):08X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check pass 0x000 and 0x00000000 filter comparing every bit in CAN ID
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t0000\r")
        for idx in range(0, 11):
            self.dut.send(b"t" + (f'{(1 << idx):03X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r" + b"T000000000\r")
        for idx in range(0, 29):
            self.dut.send(b"T" + (f'{(1 << idx):08X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check consistency by setting pass STD ID 0x000 filter again
        self.dut.send(b"m80000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r" + b"t0000\r")
        for idx in range(0, 11):
            self.dut.send(b"t" + (f'{(1 << idx):03X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        self.dut.send(b"T000000000\r")
        self.assertEqual(self.dut.receive(), b"Z\r" + b"T000000000\r")
        for idx in range(0, 29):
            self.dut.send(b"T" + (f'{(1 << idx):08X}').encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"Z\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    # TODO: implement more detailed test
    def test_bus_load(self):
        #self.dut.print_on = True

        self.dut.send(b"S0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        tx_data = b"t55585555555555555555\r"    # 112bit * 0.1ms = 11.2ms
        for i in range(0, 4):
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


if __name__ == "__main__":
    unittest.main()
