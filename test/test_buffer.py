#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class BufferTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_too_long_data_in_cdc_rx_buffer(self):
        """Verify that a command longer than the CDC Rx buffer is rejected
        with [BELL].

        Sends 999 bytes of 'F' without a terminating [CR] (well above the
        512-byte CDC Rx buffer = 8 packets x 64 bytes, and the LNBUF /
        SLCAN_MTU limit). On the following [CR], the firmware must reject
        the assembled command with [BELL] instead of treating it as a
        valid F command.
        """
        for i in range(999):
            self.dut.send(b"F")
        self.assertEqual(self.dut.receive(), b"",
                         "Device should not respond before a terminating [CR] is received")
        self.dut.send(b"\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Over-length command should be rejected with [BELL]")


    @unittest.skip("This test occasionally fails probably due to host performance limit")
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
            if self.dut.ser.write(tx_data) != len(tx_data):
                print("Failed to write all data to the device")
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
        """Verify CDC Tx buffer preserves order and content of stored Rx
        frame reports under sustained load.

        Internal-loopback mode: sends 180 short data frames (sized to
        fit in the 4096-byte CDC Tx slot at ~22 bytes per reply). Expects
        all `z[CR]` acks and looped-back Rx frame reports in order, with
        no loss reported via the F command.
        """
        #self.dut.print_on = True
        rx_data_exp = b""

        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The buffer can store at least 180 messages (4096 / 22)
        for i in range(0, 180):
            tx_data = b"t" + format(i, "03X").encode() + b"8" + format(i, "016X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += b"z\r" + tx_data

        # Check all reply
        rx_data = self.dut.receive()
        self.assertEqual(rx_data, rx_data_exp,
                         "CDC Tx buffer altered the order or content of stored Rx frame reports")

        # Check no message loss
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F00 (no loss) after 180-frame Rx burst within CDC Tx capacity")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_tx_frame_in_cdc_tx_buffer(self):
        """Verify CDC Tx buffer preserves order and content of stored Tx
        event reports under sustained load.

        Reporting mode z0002 (Tx event ON, Rx OFF). Sends 180 short data
        frames over internal loopback (sized to fit in the 4096-byte CDC
        Tx slot at ~22 bytes per Tx event) and confirms each Tx event
        report is delivered in order with no loss.
        """
        #self.dut.print_on = True
        rx_data_exp = b""

        self.dut.send(b"z0002\r")  # no rx, tx event only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The buffer can store at least 180 messages (4096 / 22)
        for i in range(0, 180):
            tx_data = b"t" + format(i, "03X").encode() + b"8" + format(i, "016X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += b"\r" + b"z" + tx_data

        # Check all reply
        rx_data = self.dut.receive()
        self.assertEqual(rx_data, rx_data_exp,
                         "CDC Tx buffer altered the order or content of stored Tx event reports")

        # Check no message loss
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F00 (no loss) after 180-frame Tx-event burst within CDC Tx capacity")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    @unittest.skip("This test occasionally fails probably due to host performance limit")
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
            if self.dut.ser.write(tx_data) != len(tx_data):
                print("Failed to write all data to the device")
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


    def test_can_rx_buffer(self):
        """Verify CAN Rx buffer preserves order of frames received in bursts.

        Sends frames in chunks of 30 (well above the HAL Rx FIFO depth of
        3) at S8/Y5 (1 Mbps / 5 Mbps) to force frame loss. Verifies that
        the frames that DO reach the host are in the correct order, and
        that the F command reports F08 (DATA_OVERRUN) indicating the
        dropped frames (HAL Rx FIFO message lost).
        """
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

        rx_data = self.dut.receive()

        # Check number of acks (this is not mandatory)
        rx_msgs = rx_data.split(b"\r")
        self.assertEqual(rx_msgs.count(b"z"), int(180 / chunk) * chunk,
                         "Unexpected number of z[CR] acks for the sent Tx commands")
        rx_msgs = [msg for msg in rx_msgs if msg != b"z"]   # remove acks

        # Check rx frames are as expected (except frame loss)
        ex_msgs = rx_data_exp.split(b"\r")
        for msg in ex_msgs:
            if rx_msgs == []:
                break
            if msg == rx_msgs[0]:
                rx_msgs.remove(rx_msgs[0])
        self.assertEqual(rx_msgs, [],
                         "Received Rx frames are out of order (some leftovers did not match the expected sequence)")

        # Check message loss in the HAL buffer to confirm a frame stack
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F08\r",
                         "Expected F08 (DATA_OVERRUN) from HAL Rx FIFO loss after burst beyond FIFO depth")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_can_tx_buffer(self):
        """Verify CAN Tx buffer preserves order and content of frames
        within its capacity (APP Tx FIFO = 64 slots).

        Sends exactly 64 short data frames at S0 (10 kbps, ~10 ms per
        frame) so the APP Tx FIFO fills but does not overflow. Internal
        loopback then echoes each frame back; expects all 64 to arrive
        in order with no loss (F00) at the end.
        """
        #self.dut.print_on = True
        rx_data_exp = b""

        self.dut.send(b"S0\r")  # take ~10ms to send one frame
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The buffer can store at least 64 messages
        # TODO: Cause buffer overflow and prove that no swapping happens?
        for i in range(0, 64):
            tx_data = b"t" + format(i, "03X").encode() + b"8" + format(i, "016X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += tx_data

        rx_data = self.dut.receive()
        rx_data += self.dut.receive()    # just to make sure (need time to tx all)
        rx_data = rx_data.replace(b"z\r", b"")
        self.assertEqual(rx_data, rx_data_exp,
                         "CAN Tx buffer altered the order or content of stored frames within capacity")

        # Check no buffer overflow
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F00 (no loss) after sending exactly the APP Tx FIFO capacity (64 frames)")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_can_tx_event_buffer(self):
        """Verify HAL Tx Event FIFO preserves order of events during burst
        transmission, and that overflow is reported via F08.

        Sends frames in chunks of 30 (well above the HAL Tx Event FIFO
        depth of 3) at S8/Y5 (1 Mbps / 5 Mbps) with z0002 (Tx event ON,
        Rx OFF) to force event drops. Verifies that the events that DO
        reach the host are in the correct order, and that the F command
        reports F08 (DATA_OVERRUN). See the inline comment below for
        why F08 specifically proves Tx Event FIFO loss here.
        """
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

        rx_data = self.dut.receive()

        # Check number of acks (this is not mandatory)
        rx_msgs = rx_data.split(b"\r")
        self.assertEqual(rx_msgs.count(b""), int(180 / chunk) * chunk + 1,
                         "Unexpected number of bare [CR] acks for the sent Tx commands (z0002 mode)")  # +1 by the last tx event
        rx_msgs = [msg for msg in rx_msgs if msg != b""]   # remove acks

        # Check rx frames are as expected (except frame loss)
        ex_msgs = rx_data_exp.split(b"\r")
        for msg in ex_msgs:
            if rx_msgs == []:
                break
            if msg == rx_msgs[0]:
                rx_msgs.remove(rx_msgs[0])
        self.assertEqual(rx_msgs, [],
                         "Received Tx events are out of order (some leftovers did not match the expected sequence)")

        # Check F bit 3 (DATA_OVERRUN) is raised.
        # F bit 3 has three possible sources (HAL Rx FIFO lost, HAL Tx Frame FIFO write fail,
        # HAL Tx Event FIFO element lost), but only the last applies here:
        #   - Rx is disabled by z0002, so no Rx FIFO loss can occur.
        #   - Tx Frame FIFO writes are guarded by HAL_FDCAN_GetTxFifoFreeLevel() > 0,
        #     so write failures do not occur under normal test conditions.
        # Therefore F bit 3 here specifically proves HAL Tx Event FIFO element loss (frame stack).
        # See also: https://github.com/Nakakiyo092/usb2canfdv1/issues/49
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F08\r",
                         "Expected F08 (DATA_OVERRUN) from HAL Tx Event FIFO loss; see comment above for why this is specific")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_high_rx_frame_rate(self):
        """Verify Rx-side buffers preserve order and content at the
        maximum sustainable frame rate.

        S8/Y5 (1 Mbps / 5 Mbps) with z0001 (Rx ON, Tx event OFF). Sends
        180 short frames back-to-back over internal loopback (sized to
        fit in the 4096-byte CDC Tx slot at ~22 bytes per reply) and
        checks that every frame arrives in order with no loss (F00).
        """
        #self.dut.print_on = True
        rx_data_exp = b""

        self.dut.send(b"S8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0001\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The buffer can store at least 180 messages (4096 / 22)
        for i in range(0, 180):
            tx_data = b"t" + format(i, "03X").encode() + b"1" + format(i, "02X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += b"z\r" + tx_data

        rx_data = self.dut.receive()
        self.assertEqual(rx_data, rx_data_exp,
                         "Rx buffer altered the order or content of frames under high Rx frame rate")

        # Check no buffer overflow
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F00 (no loss) after 180-frame burst at high Rx rate")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_high_tx_frame_rate(self):
        """Verify Tx-side buffers preserve order and content at the
        maximum sustainable Tx event rate.

        S8/Y5 (1 Mbps / 5 Mbps) with z0002 (Tx event ON, Rx OFF). Sends
        180 short frames back-to-back over internal loopback (sized to
        fit in the 4096-byte CDC Tx slot at ~22 bytes per Tx event) and
        checks that every Tx event arrives in order with no loss (F00).
        """
        #self.dut.print_on = True
        rx_data_exp = b""

        self.dut.send(b"S8\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z0002\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # The buffer can store at least 180 messages (4096 / 22)
        for i in range(0, 180):
            tx_data = b"t" + format(i, "03X").encode() + b"1" + format(i, "02X").encode() + b"\r"
            self.dut.send(tx_data)
            rx_data_exp += b"\r" + b"z" + tx_data

        rx_data = self.dut.receive()
        self.assertEqual(rx_data, rx_data_exp,
                         "Tx buffer altered the order or content of events under high Tx frame rate")

        # Check no buffer overflow
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F00 (no loss) after 180-frame burst at high Tx rate")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
