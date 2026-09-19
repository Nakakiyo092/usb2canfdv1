#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


# Detailed status replies expected at the various points in the tests below.
# Kept as constants to keep assertions readable.
F_DETAIL_CLEAN              = b"f: node_sts=ER_ACTV, last_err_code=NONE, err_cnt_tx_rx=[0x00, 0x00], th_bus_load_percent=00\r"
F_DETAIL_BUS_OFF_BIT0_F8    = b"f: node_sts=BUS_OFF, last_err_code=BIT0, err_cnt_tx_rx=[0xF8, 0x00], th_bus_load_percent=00\r"
F_DETAIL_PSSV_BIT0_TX_88    = b"f: node_sts=ER_PSSV, last_err_code=BIT0, err_cnt_tx_rx=[0x88, 0x00], th_bus_load_percent=00\r"
F_DETAIL_PSSV_BIT0_TX_F8    = b"f: node_sts=ER_PSSV, last_err_code=BIT0, err_cnt_tx_rx=[0xF8, 0x00], th_bus_load_percent=00\r"


class ShortTestCase(unittest.TestCase):
    """Tests that exercise the bus-off path.

    NOTE: These tests require the CAN bus to be physically shorted (CAN-H
    and CAN-L connected together). Each transmitted frame on a shorted
    bus generates a BIT0 protocol error, allowing the node to be driven
    through error-passive into bus-off in a deterministic way."""

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_bus_off_and_recovery(self):
        """End-to-end bus-off behaviour:
        - Internal loopback runs without errors despite the physical short.
        - Opening in normal mode and sending one frame drives the node
          straight into BUS_OFF (BIT0 errors, TEC saturates at 0xF8).
        - During BUS_OFF every Tx command is rejected with [BEL].
        - A C -> O sequence (under loopback) recovers the node to a clean
          state, and frames are echoed normally again."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Internal loopback is unaffected by the short (no real bus arbitration).
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Internal loopback open should leave the node error-free")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_CLEAN,
                         "Detailed status should be ER_ACTV/NONE/zero err_cnt under internal loopback")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Open in normal mode and send one frame; the short forces BUS_OFF.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Normal-mode open should leave the node error-free until the first send")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_CLEAN,
                         "Detailed status should be ER_ACTV/NONE/zero err_cnt right after open")
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.1)     # let TEC saturate ( > 1ms * 255 / 8)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FB4\r",
                         "Expected FB4 (BUS_ERROR | BUS_OFF | ERROR_PASSIVE | ERROR_WARNING) after one no-ACK send on a shorted bus")
        time.sleep(0.1)
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_BUS_OFF_BIT0_F8,
                         "Detailed status should report BUS_OFF with last_err=BIT0 and TEC=0xF8")

        # During BUS_OFF every Tx command must be rejected with [BEL].
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"STD command {cmd!r} should be rejected with [BEL] while BUS_OFF")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"EXT command {cmd!r} should be rejected with [BEL] while BUS_OFF")

        # C -> O (here, C -> internal loopback) recovers the node from BUS_OFF.
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Frames are echoed normally after recovery.
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r",
                             f"After BUS_OFF recovery, STD frame {cmd!r}03F0 should loopback normally")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r",
                             f"After BUS_OFF recovery, EXT frame {cmd!r}0137FEC8 should loopback normally")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_error_state_progression(self):
        """Track the error-state progression frame-by-frame in no-retransmit
        mode on a shorted bus. After F bits are read and cleared, only
        currently-active conditions reappear on the next read.

        Steps:
        1. One no-ACK send -> ER_PSSV (BIT0 jumps REC=0 straight to passive,
           TEC=0x88). F = FA4 (BUS_ERROR | ERROR_PASSIVE | ERROR_WARNING).
        2. Clear F, send 14 more -> TEC saturates at 0xF8 while still in
           passive. F = F80 (BUS_ERROR only; passive/warning are latched
           and already consumed).
        3. Clear F, send one more -> node enters BUS_OFF.
           F = F90 (BUS_ERROR | BUS_OFF).

        TODO: Step 1's REC=0 -> passive jump is HAL behaviour and not yet
        confirmed against the data sheet."""
        self.dut.send(b"-\r")   # No retransmit mode
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "No-retransmit open should leave the node error-free")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_CLEAN,
                         "Detailed status should be ER_ACTV/NONE/zero err_cnt at start")

        # Step 1: one no-ACK send -> straight to ER_PSSV (TEC=0x88).
        for _ in range(0, 1):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # let the error state settle ( > 1ms * 1)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"FA4\r",
                         "Expected FA4 (BUS_ERROR | ERROR_PASSIVE | ERROR_WARNING) after the first no-ACK send")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_PSSV_BIT0_TX_88,
                         "Detailed status should report ER_PSSV with last_err=BIT0 and TEC=0x88")

        # Step 2: 14 more no-ACK sends -> TEC saturates at 0xF8, still passive.
        # PASSIVE and WARNING were already consumed by the previous F read,
        # so only BUS_ERROR comes back.
        for _ in range(0, 14):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # let TEC saturate ( > 1ms * 14)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F80\r",
                         "Expected F80 (BUS_ERROR only) - PASSIVE/WARNING already consumed")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_PSSV_BIT0_TX_F8,
                         "Detailed status should report ER_PSSV with TEC saturated at 0xF8")

        # Step 3: one more no-ACK send -> BUS_OFF.
        for _ in range(0, 1):
            self.dut.send(b"t0000\r")
            self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.2)     # let the bus-off transition settle ( > 1ms * 1)

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F90\r",
                         "Expected F90 (BUS_ERROR | BUS_OFF) after crossing into bus-off")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F bits should clear after read")
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), F_DETAIL_BUS_OFF_BIT0_F8,
                         "Detailed status should report BUS_OFF with last_err=BIT0 and TEC=0xF8")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
