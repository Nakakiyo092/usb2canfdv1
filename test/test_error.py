#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest

# Detailed status replies expected at the various points in the tests below.
# Kept as constants to keep assertions readable.
F_DETAIL_CLEAN          = b"f: node_sts=ER_ACTV, last_err_code=NONE, err_cnt_tx_rx=[0x00, 0x00], th_bus_load_percent=00\r"
F_DETAIL_ACK_TX_08      = b"f: node_sts=ER_ACTV, last_err_code=_ACK, err_cnt_tx_rx=[0x08, 0x00], th_bus_load_percent=00\r"
F_DETAIL_ACK_TX_60      = b"f: node_sts=ER_ACTV, last_err_code=_ACK, err_cnt_tx_rx=[0x60, 0x00], th_bus_load_percent=00\r"
F_DETAIL_PSSV_TX_80     = b"f: node_sts=ER_PSSV, last_err_code=_ACK, err_cnt_tx_rx=[0x80, 0x00], th_bus_load_percent=00\r"


class ErrorTestCase(unittest.TestCase):


    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_error_active(self):
        """In normal operation the device reports the error-free state
        (ER_ACTV / NONE / err_cnt=[0,0]) both at idle and after a successful
        internal-loopback transmission."""
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Idle state - no error should be present.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Idle device should report F00 (no error)")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_CLEAN,
                         "Detailed status at idle should be ER_ACTV/NONE/zero err_cnt")

        # After a successful loopback send - still clean.
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\rt0000\r")
        time.sleep(0.05)    # wait for the report to settle ( > 1ms * 1)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "No error should accumulate after a clean loopback send")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_CLEAN,
                         "Detailed status after a clean send should still be ER_ACTV/NONE/zero err_cnt")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_bus_error(self):
        """A single un-acknowledged transmission (no-retransmit mode) raises
        the BUS_ERROR flag (F bit 7) and increments TEC by 8. last_err_code
        becomes _ACK."""
        self.dut.send(b"-\r")   # No retransmit mode
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.05)    # wait for the bus error to be observed ( > 1ms * 1)

        # First F read sees the bus error flag and clears it.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F80\r",
                         "Expected F80 (BUS_ERROR, bit 7) after one no-ACK transmission")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should self-clear on read; second F should be F00")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_ACK_TX_08,
                         "Detailed status should report last_err=_ACK and TEC=0x08 after one no-ACK send")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_error_warning(self):
        """Twelve un-acknowledged transmissions push TEC to 0x60 and raise
        ERROR_WARNING (F bit 2), combined with BUS_ERROR (bit 7) -> F84."""
        self.dut.send(b"-\r")   # No retransmit mode
        self.assertEqual(self.dut.receive(), b"\r")
        for _ in range(0, 12):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.05)    # wait for the warning level to be observed ( > 1ms * 12)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F84\r",
                         "Expected F84 (BUS_ERROR | ERROR_WARNING) after 12 no-ACK transmissions")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should self-clear on read; second F should be F00")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_ACK_TX_60,
                         "Detailed status should report TEC=0x60 (12 * 8) after 12 no-ACK sends")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_error_passive(self):
        """Sustained no-ACK retransmissions in normal mode drive TEC to >=128
        (0x80), which switches the node to ER_PSSV and raises ERROR_PASSIVE
        (F bit 5). Combined with BUS_ERROR (bit 7) and ERROR_WARNING (bit 2),
        F becomes FA4."""
        # Open in normal mode (not test mode) so auto-retransmit applies.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_CLEAN)

        # One frame, retransmitted automatically; TEC saturates the warning,
        # passive and bus-error bits over ~128 ms.
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # wait for error passive ( > 1ms * 128)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r",
                         "Expected FA4 (BUS_ERROR | ERROR_PASSIVE | ERROR_WARNING) after sustained no-ACK retransmit")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should self-clear on read; second F should be F00")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_PSSV_TX_80,
                         "Detailed status should report ER_PSSV and TEC=0x80 after entering passive state")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_error_passive_clear(self):
        """Once F bits are read and cleared, only currently-active conditions
        re-appear on the next read. After reaching ERROR_WARNING then clearing,
        further no-ACK transmissions raise ERROR_PASSIVE (bit 5) but not the
        latch-style BUS_ERROR (bit 7), giving FA0 rather than FA4."""
        self.dut.send(b"-\r")   # No retransmit mode
        self.assertEqual(self.dut.receive(), b"\r")

        # 12 no-ACK sends -> warning level (BUS_ERROR | ERROR_WARNING).
        for _ in range(0, 12):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.05)    # wait for the warning level ( > 1ms * 12)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F84\r",
                         "Expected F84 (BUS_ERROR | ERROR_WARNING) after 12 no-ACK transmissions")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_ACK_TX_60,
                         "Detailed status after clear should still show TEC=0x60 (clearing F does not reset err_cnt)")

        # 4 more no-ACK sends push TEC across 128 -> passive level.
        for _ in range(0, 4):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.05)    # wait for the passive level ( > 1ms * 4)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA0\r",
                         "Expected FA0 (ERROR_PASSIVE | ERROR_WARNING) after crossing TEC=128; "
                         "BUS_ERROR (bit 7) was already consumed by the previous F read")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_PSSV_TX_80,
                         "Detailed status should report ER_PSSV and TEC=0x80 after entering passive state")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_cdc_rx_overflow(self):
        """Flooding the CDC Rx buffer faster than the main loop can drain it
        raises F bit 1 (SLCAN_STS_CAN_TX_FIFO_FULL, shared by CDC Rx side
        per doc/2). The DEBUG-only stall command ~<HHHH>[CR] is used to
        guarantee the overflow by blocking the main loop while the host
        floods.

        Note: The corruption-defence behaviour of the same overflow path
        is verified in test_buffer.test_message_loss_in_cdc_rx_buffer;
        this test focuses on the error flag side."""
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Sanity check at idle.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        # Stall 1000 ms (0x03E8) and flood `II\r` during the stall;
        # 1500 * 3 = 4500 bytes >> ~512-byte CDC Rx ring.
        self.dut.send(b"~03E8\r")
        self.dut.ser.write(b"II\r" * 1500)
        time.sleep(1.5)
        for _ in range(20):
            if not self.dut.receive():
                break

        # Empty command flushes the device's pending state so F responds
        # promptly (see test_buffer.test_message_loss_in_cdc_rx_buffer).
        self.dut.send(b"\r")
        self.dut.receive()

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F02\r",
                         "Expected F02 (CDC Rx overflow, reported via bit 1) after sustained host write burst")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Drain any residual output from the flood so the next test starts
        # from a clean stream.
        for _ in range(5):
            if not self.dut.receive():
                break


    def test_cdc_tx_overflow(self):
        """Sending many `v\\r` commands without draining the responses fills
        the device's CDC Tx buffer. Overflow is reported as F bit 0
        (SLCAN_STS_CAN_RX_FIFO_FULL, shared by CDC Tx side per doc/2)."""
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Sanity check at idle.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        # Flood `v` commands without reading any reply. The response is the
        # long version string, so 400 fills the CDC Tx ring comfortably.
        # (Exact count depends on PC USB Rx scheduling.)
        for _ in range(0, 400):
            self.dut.send(b"v\r")
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)

        # Drain whatever made it out before checking the flag.
        self.dut.receive()

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F01\r",
                         "Expected F01 (CDC Tx overflow, reported via bit 0)")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_can_rx_overflow(self):
        """Drive a sustained loopback frame burst large enough to spill the
        CDC Tx buffer. Overflow is reported as F bit 0.

        TODO: Despite the test name, this currently exercises the CDC Tx
        buffer overflow path, not the CAN Rx FIFO. Reworking it to actually
        target the CAN Rx FIFO requires a faster producer than the host can
        provide via internal loopback."""
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Sanity check at idle.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        # 180 messages fit in the CDC Tx ring (4096 / 24 bytes per reply).
        for _ in range(0, 180):
            self.dut.send(b"t03F80011223344556677\r")
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)
        self.dut.receive()
        self.dut.receive()  # just to make sure

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "180 frames should fit without overflow (CDC Tx capacity)")

        # 2000 messages do not fit; the exact threshold depends on host OS
        # buffering, so the count is generous.
        for _ in range(0, 2000):
            self.dut.send(b"t03F80011223344556677\r")
            # Avoid main-loop starvation (STUN).
            # See https://github.com/Nakakiyo092/usb2canfdv1/discussions/152
            time.sleep(0.001)
        self.dut.receive()
        self.dut.receive()  # just to make sure

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F01\r",
                         "Expected F01 (CDC Tx overflow, reported via bit 0) after 2000-frame burst")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_can_tx_overflow(self):
        """The APP-level CAN Tx FIFO holds 64 frames. Filling it raises the
        bus-error/passive/warning chain (FA4) because frames cannot reach the
        bus. Subsequent Tx command bytes that find the FIFO already full
        report F02 (CAN Tx FIFO full, F bit 1) and individual `t` commands
        are rejected with [BEL]."""
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Sanity check at idle.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        # 64 frames just fit; all accepted.
        for _ in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")

        # CAN bus is not acknowledged in normal mode without a peer, so the
        # node escalates through warning -> passive -> bus error.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r",
                         "Expected FA4 (BUS_ERROR | ERROR_PASSIVE | ERROR_WARNING) once the 64-slot Tx FIFO is full")

        # 64 more frames cannot be enqueued; drain whatever comes back.
        for _ in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.dut.receive()

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F02\r",
                         "Expected F02 (CAN Tx FIFO full, bit 1) after the second 64-frame burst")

        # Each subsequent Tx command is rejected with [BEL] (no slot to queue).
        for _ in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             "Tx command should be rejected with [BEL] while the Tx FIFO is full")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F02\r",
                         "F02 should be raised again by the rejection burst")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_no_retransmit(self):
        """In no-retransmit mode a frame that fails to be acknowledged is
        dropped instead of being retransmitted. The TEC saturates after the
        first ~16 frames and stays put; further sends do not raise additional
        BUS_ERROR/PASSIVE flags."""
        self.dut.send(b"-\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Sanity check at idle.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r")

        # 64 frames, each dropped because no peer ACKs.
        for _ in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r",
                         "Expected FA4 (BUS_ERROR | ERROR_PASSIVE | ERROR_WARNING) after 64 no-ACK sends")
        self.dut.send(b"f\r")
        self.dut.receive()

        # Another 64 frames; TEC has already saturated, so nothing new arises.
        for _ in range(0, 64):
            self.dut.send(b"t03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "TEC saturates by ~16 frames; further no-ACK sends should not re-raise the error flags")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
