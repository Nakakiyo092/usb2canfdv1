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


    def test_message_loss_in_cdc_rx_buffer(self):
        """Verify the firmware's CDC Rx buffer overflow defence:
        - the torn-prefix dropping in buf_process() prevents data loss from
          producing a fabricated valid command, AND
        - the overflow is reported via F bit 1 (SLCAN_STS_CAN_TX_FIFO_FULL,
          mapped to CDC Rx side per doc/2.-Command-List.md).

        Method:
        Probe the device once with `I\\r` to capture this hardware's
        expected reply (it varies by chip / clock_mhz, so we cannot
        hard-code `I3050\\r`). Then stall the main loop with
        ~<HHHH>[CR] (DEBUG-only) and flood the device with `II\\r` during
        the stall. `II\\r` is a 3-byte invalid command whose length is
        coprime with the 64-byte USB CDC packet size, so any non-trivial
        byte loss (single byte, packet-aligned, or multi-packet) shifts
        the command boundary. If the torn-prefix logic fails to discard
        the garbled bytes, a stray `I\\r` slice would produce an I reply
        or a bare `\\r` OK reply.

        Expected:
        rx contains only the deferred stall ACK (`\\r`) and a sequence of
        `\\a` (BEL) replies. F bit 1 is set.
        """
        # Open internal loopback so the F command is available afterwards.
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Capture this hardware's I reply for the corruption check.
        self.dut.send(b"I\r")
        expected_reply = self.dut.receive()
        self.assertTrue(expected_reply.startswith(b"I") and expected_reply.endswith(b"\r"),
                        f"Unexpected shape for the I reply: {expected_reply!r}")

        # Stall 1000 ms (0x03E8) and flood `II\r` during the stall.
        # 1500 * 3 = 4500 bytes >> CDC Rx ring (BUF_CDC_RX_NUM_BUFS * 64 ~= 512 B).
        self.dut.send(b"~03E8\r")
        self.dut.ser.write(b"II\r" * 1500)
        time.sleep(1.5)

        rx_data = b""
        for _ in range(20):
            chunk = self.dut.receive()
            if not chunk:
                break
            rx_data += chunk

        # The deferred stall ACK arrives as a single leading `\r`. Strip it,
        # then every remaining byte must be `\a` (BEL). Any I reply leakage
        # or extra `\r` indicates the torn-prefix defence failed.
        self.assertTrue(rx_data.startswith(b"\r"),
                        f"Expected the stall ACK as the first byte, got: {rx_data[:16]!r}")
        residue = rx_data[1:]
        self.assertNotIn(expected_reply, residue,
                         f"I reply leaked: torn data parsed as a valid I command, got: {rx_data!r}")
        self.assertEqual(residue.replace(b"\a", b""), b"",
                         f"Non-BEL bytes detected after the stall ACK, got: {rx_data!r}")

        # Send a bare [CR] (empty command) before F. Without this nudge the
        # F reply is delayed until the device has fully digested the flooded
        # buffers; the empty command flushes that pending state and lets the
        # subsequent F return immediately.
        self.dut.send(b"\r")
        self.dut.receive()

        self.dut.send(b"F\r")
        f_reply = self.dut.receive()
        self.assertEqual(len(f_reply), len(b"Fxx\r"),
                         f"Unexpected F reply length: {f_reply!r}")
        flags = int(f_reply[1:3], 16)
        self.assertTrue(flags & 0x02,
                        f"F bit 1 (CDC Rx overflow) is not raised, F={f_reply!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


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
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)

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
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)

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


    def test_message_loss_in_cdc_tx_buffer(self):
        """Verify the firmware's CDC Tx buffer overflow behaviour:
        - whole replies may be lost when the buffer fills, BUT
        - no reply is partially written (truncated / corrupted), AND
        - the overflow is reported via F bit 0 (SLCAN_STS_CAN_RX_FIFO_FULL,
          mapped to CDC Tx side per doc/2.-Command-List.md).

        Method:
        Probe the device once with `I\\r` to capture this hardware's
        expected reply (the reply varies by chip / clock_mhz, so we cannot
        hard-code `I3050\\r`). Then flood the device with `I\\r` in
        small bursts separated by short sleeps. The bursts are sized so
        the CDC Rx buffer never overflows (which would corrupt the
        request stream and skew the test). The host does NOT drain the
        Tx buffer during the bursts, so replies accumulate on the device
        side until the CDC Tx buffer (BUF_CDC_TX_NUM_BUFS * 4096 B ~= 12 kB
        => ~2k replies) overflows.

        Expected:
        - The drained stream consists solely of intact expected_reply
          repeats - no truncated or corrupted fragments.
        - The number of received replies is strictly less than the number
          of commands sent (proves the CDC Tx buffer actually overflowed).
        - F bit 0 is set.
        """
        # Open internal loopback so the F command is available afterwards.
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Capture this hardware's I reply (e.g. b"I3050\r" on STM32G0B1,
        # b"I30A0\r" on STM32G431). The test is then hardware-agnostic.
        self.dut.send(b"I\r")
        expected_reply = self.dut.receive()
        self.assertTrue(expected_reply.startswith(b"I") and expected_reply.endswith(b"\r"),
                        f"Unexpected shape for the I reply: {expected_reply!r}")

        # Burst flood, throttled to keep the CDC Rx buffer below overflow
        # so the request stream is not torn (which would skew the Tx test).
        # 50 requests * 2 B = 100 B per burst, well under the ~512 B Rx ring.
        # Burst-to-burst sleep lets the main loop drain Rx faster than we
        # write while replies pile up in Tx since the host does not read.
        BURST = 50
        BURST_DELAY = 0.05      # 50 ms between bursts
        N_REPLIES = 3000        # > 2k slots in CDC Tx ring -> guaranteed overflow
        for _ in range(N_REPLIES // BURST):
            self.dut.ser.write(b"I\r" * BURST)
            time.sleep(BURST_DELAY)

        # Drain the Tx buffer.
        time.sleep(0.5)
        rx_data = b""
        for _ in range(30):
            chunk = self.dut.receive()
            if not chunk:
                break
            rx_data += chunk

        # Every byte received must belong to a complete reply (no truncation).
        self.assertEqual(rx_data.replace(expected_reply, b""), b"",
                         f"Non-{expected_reply!r} residue detected (truncated/corrupt reply): {rx_data!r}")

        # Replies are lost as whole units; received count must be < N_REPLIES.
        received = rx_data.count(expected_reply)
        self.assertLess(received, N_REPLIES,
                        f"Expected reply loss but received all {received}/{N_REPLIES}; no overflow pressure")

        # Flush any pending device-side state with a bare [CR] before F
        # (same rationale as the Rx test).
        self.dut.send(b"\r")
        self.dut.receive()

        self.dut.send(b"F\r")
        f_reply = self.dut.receive()
        self.assertEqual(len(f_reply), len(b"Fxx\r"),
                         f"Unexpected F reply length: {f_reply!r}")
        flags = int(f_reply[1:3], 16)
        self.assertTrue(flags & 0x01,
                        f"F bit 0 (CDC Tx overflow) is not raised, F={f_reply!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


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
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)

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
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)

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
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)

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
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)

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
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)

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
