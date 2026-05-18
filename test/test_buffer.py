#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class BufferTestCase(unittest.TestCase):

    print_on: bool
    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    # Check response to a command longer than the cdc rx buffer itself
    def test_too_long_data_in_cdc_rx_buffer(self):
        # CDC Rx buffer size: 64 * 8 = 512
        for i in range(999):
            self.dut.send(b"F")
        self.assertEqual(self.dut.receive(), b"")
        self.dut.send(b"\r")
        self.assertEqual(self.dut.receive(), b"\a")


    def test_message_loss_in_cdc_rx_buffer(self):
        """
        Check no corruption of data in cdc rx buffer when it is full

        Method:
        Send data within which all message is valid command
        but turns into an invalid command if some of the data is lost.
        Send such data repeatedly until the rx buffer of the device is full.

        Criteria:
        The device should not respond to the false invalid command
        which is created by data loss.
        """
        #self.dut.print_on = True
        version = b""
        tx_data = b""
        rx_data = b""

        self.dut.send(b"V\r")
        version = self.dut.receive()

        self.dut.send(b"O\r")   # Need to use F
        self.assertEqual(self.dut.receive(), b"\r")

        # Rx buffer size: 8 * 64
        for _ in range(2500):
            tx_data += b"V\r\r" # 2 char loss will create VV\r. V\r version is in the tx test.
        for _ in range(10):
            self.dut.send(tx_data)
            rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(version, b"")
        rx_data = rx_data.replace(b"\r", b"")
        self.assertEqual(rx_data, b"")  # Confirm no \a
        time.sleep(0.1)
        self.dut.send(b"\r")    # Flush the buffer
        self.dut.receive()
        self.dut.send(b"F\r")
        self.assertIn(self.dut.receive(), [b"F03\r", b"F01\r"])


    @unittest.skip("This test does not create rx buffer overflow")
    def test_message_loss_in_cdc_rx_buffer_fail(self):
        """
        Check no corruption of data in cdc rx buffer when it is full

        Method:
        Send data within which all message is invalid command
        but turns into a valid command if some of the data is lost.
        Send such data repeatedly until the rx buffer of the device is full.

        Criteria:
        The device should not respond to the false valid command
        which is created by data loss.
        """
        #self.dut.print_on = True

        self.dut.send(b"O\r")   # Need to use F
        self.assertEqual(self.dut.receive(), b"\r")

        # Catch char-level loss: 1, 4, 7, 10, 13, 16, 19 ...
        tx_data = b""
        rx_data = b""
        for _ in range(900):
            tx_data += b"VV\r"
        for _ in range(10):
            if self.dut.ser.write(tx_data) != len(tx_data):
                print("Failed to write all data to the device")
            rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\a", b"")
        rx_data = rx_data.replace(b"\r", b"")
        self.assertEqual(rx_data, b"")
        time.sleep(0.1)
        self.dut.send(b"\r")    # Flush the buffer
        self.dut.receive()
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F03\r")  # Or F01

        # Catch char-level loss: 1, 2, 5, 6, 9, 10, 13, 14, 17, 18 ...
        tx_data = b""
        rx_data = b""
        for _ in range(800):
            tx_data += b"VV\r\r"
        for _ in range(10):
            if self.dut.ser.write(tx_data) != len(tx_data):
                print("Failed to write all data to the device")
            rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\a", b"")
        rx_data = rx_data.replace(b"\r", b"")
        self.assertEqual(rx_data, b"")
        time.sleep(0.1)
        self.dut.send(b"\r")    # Flush the buffer
        self.dut.receive()
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F03\r")

        # Catch dchar-level loss: 1, 2, 3, 6, 7, 8, 11, 12, 13, 16, 17, 18 ...
        tx_data = b""
        rx_data = b""
        for _ in range(700):
            tx_data += b"VV\r\r\r"
        for _ in range(10):
            if self.dut.ser.write(tx_data) != len(tx_data):
                print("Failed to write all data to the device")
            rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(b"\a", b"")
        rx_data = rx_data.replace(b"\r", b"")
        self.assertEqual(rx_data, b"")
        time.sleep(0.1)
        self.dut.send(b"\r")    # Flush the buffer
        self.dut.receive()
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F03\r")


    def test_rx_frame_in_cdc_tx_buffer(self):
        """Check stored frames in CDC Tx buffer are not altered in order or content"""
        #self.dut.print_on = True
        rx_data_exp = b""

        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # the buffer can store as least 180 messages (4096 / 22)
        for i in range(0, 180):
            tx_data = b"t" + format(i, "03X").encode() + b"8" + format(i, "016X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += b"z\r" + tx_data
            time.sleep(0.001)

        # Check all reply
        rx_data = self.dut.receive()
        self.assertEqual(rx_data, rx_data_exp)

        # Check no message loss
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_tx_frame_in_cdc_tx_buffer(self):
        """Check stored frames in CDC Tx buffer are not altered in order or content"""
        #self.dut.print_on = True
        rx_data_exp = b""

        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The buffer can store as least 180 messages (4096 / 22)
        for i in range(0, 180):
            tx_data = b"t" + format(i, "03X").encode() + b"8" + format(i, "016X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += b"\r" + b"z" + tx_data
            time.sleep(0.001)

        # Check all reply
        rx_data = self.dut.receive()
        self.assertEqual(rx_data, rx_data_exp)

        # Check no message loss
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_message_loss_in_cdc_tx_buffer(self):
        """
        Check no corruption of data in cdc tx buffer when it is full

        Method:
        Send many commands to the device and occasionally receive responses
        until the tx buffer of the device is full.

        Criteria:
        Data loss is acceptable if the whole message is lost,
        but the device should not send any corrupted data.
        """
        #self.dut.print_on = True
        version = b""
        tx_data = b""
        rx_data = b""

        self.dut.send(b"V\r")
        version = self.dut.receive()

        self.dut.send(b"O\r")   # Need to use F
        self.assertEqual(self.dut.receive(), b"\r")

        # Tx buffer size: 3 * 4096
        for _ in range(2500):   # * 6 bytes of reply
            tx_data += b"V\r"
        for _ in range(10):
            self.dut.send(tx_data)
            rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data += self.dut.receive()
        rx_data = rx_data.replace(version, b"")
        self.assertEqual(rx_data, b"")  # Confirm no \a as a rx test
        time.sleep(0.1)
        self.dut.send(b"\r")    # Flush the buffer
        self.dut.receive()
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F03\r")  # Or F02


    # Check stored frames in CAN Rx buffer are not altered in order or content
    def test_can_rx_buffer(self):
        """Check stored frames in CAN Rx buffer are not altered in order or content"""
        #self.dut.print_on = True

        chunk = 30  # "stun" the device by sending too many frames at once

        self.dut.send(b"S8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The cdc buffer can store as least 180 messages (4096 / 22)
        # Do not cause irrelevant buffer overflow
        rx_data_exp = b""
        for i in range(0, int(180 / chunk)):
            tx_data = b""
            for j in range(0, chunk):
                nbr = int(i * chunk + j)
                frame = b"t" + format(nbr, "03X").encode() + b"1" + format(nbr, "02X").encode() + b"\r"
                tx_data += frame
                rx_data_exp += frame    # except ack
            self.dut.send(tx_data)
            time.sleep(0.001)

        rx_data = self.dut.receive()

        # Check number of acks (this is not mandatory)
        rx_msgs = rx_data.split(b"\r")
        self.assertEqual(rx_msgs.count(b"z"), int(180 / chunk) * chunk)
        rx_msgs = [msg for msg in rx_msgs if msg != b"z"]   # remove acks

        # Check rx frames are as expected (except frame loss)
        ex_msgs = rx_data_exp.split(b"\r")
        for msg in ex_msgs:
            if rx_msgs == []:
                break
            if msg == rx_msgs[0]:
                rx_msgs.remove(rx_msgs[0])
        self.assertEqual(rx_msgs, [])

        # Check message loss in the HAL buffer to confrim a frame stack
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F08\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_can_tx_buffer(self):
        """Check stored frames in CAN Tx buffer are not altered in order or content"""
        #self.dut.print_on = True
        rx_data_exp = b""

        self.dut.send(b"S0\r")  # take ~10ms to send one frame
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The buffer can store as least 64 messages
        # TODO: Cause buffer overflow and prove that no swapping happens?
        for i in range(0, 64):
            tx_data = b"t" + format(i, "03X").encode() + b"8" + format(i, "016X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += tx_data
            time.sleep(0.001)

        rx_data = self.dut.receive()
        rx_data += self.dut.receive()    # just to make sure (need time to tx all)
        rx_data = rx_data.replace(b"z\r", b"")
        self.assertEqual(rx_data, rx_data_exp)

        # Check no buffer overflow
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_can_tx_event_buffer(self):
        """Check stored frames in Tx event buffer are not altered in order or content"""
        #self.dut.print_on = True

        chunk = 30  # "stun" the device by sending too many frames at once

        self.dut.send(b"S8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The cdc buffer can store as least 180 messages (4096 / 22) (Do not cause irrelevant buffer overflow)
        rx_data_exp = b""
        for i in range(0, int(180 / chunk)):
            tx_data = b""
            for j in range(0, chunk):
                nbr = int(i * chunk + j)
                frame = b"t" + format(nbr, "03X").encode() + b"1" + format(nbr, "02X").encode() + b"\r"
                tx_data += frame
                rx_data_exp += b"z" + frame    # except ack
            self.dut.send(tx_data)
            time.sleep(0.001)

        rx_data = self.dut.receive()

        # Check number of acks (this is not mandatory)
        rx_msgs = rx_data.split(b"\r")
        self.assertEqual(rx_msgs.count(b""), int(180 / chunk) * chunk + 1)  # +1 by the last tx event
        rx_msgs = [msg for msg in rx_msgs if msg != b""]   # remove acks

        # Check rx frames are as expected (except frame loss)
        ex_msgs = rx_data_exp.split(b"\r")
        for msg in ex_msgs:
            if rx_msgs == []:
                break
            if msg == rx_msgs[0]:
                rx_msgs.remove(rx_msgs[0])
        self.assertEqual(rx_msgs, [])

        # Check message loss in the HAL buffer to confrim a frame stack
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F08\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_high_rx_frame_rate(self):
        #self.dut.print_on = True
        rx_data_exp = b""

        # Check stored frames in buffer are not altered in order or content in high rx frame rate
        self.dut.send(b"S8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0001\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        #  the buffer can store as least 180 messages (4096 / 22)
        for i in range(0, 180):
            tx_data = b"t" + format(i, "03X").encode() + b"1" + format(i, "02X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += b"z\r" + tx_data
            time.sleep(0.001)   # TODO ? Prevent stuck on host side

        rx_data = self.dut.receive()
        self.assertEqual(rx_data, rx_data_exp)

        # Check no buffer overflow
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_high_tx_frame_rate(self):
        #self.dut.print_on = True
        rx_data_exp = b""

        # Check stored frames in buffer are not altered in order or content in high tx frame rate
        self.dut.send(b"S8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0002\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        #  the buffer can store as least 180 messages (4096 / 22)
        for i in range(0, 180):
            tx_data = b"t" + format(i, "03X").encode() + b"1" + format(i, "02X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += b"\r" + b"z" + tx_data
            time.sleep(0.001)   # TODO ? Prevent stuck on host side

        rx_data = self.dut.receive()
        self.assertEqual(rx_data, rx_data_exp)

        # Check no buffer overflow
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
