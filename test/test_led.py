#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class LedTestCase(unittest.TestCase):
    """Visual verification of LED blink behaviour.

    NOTE: This test requires a human operator. Both Rx and Tx LEDs should
    light up 16 times in total over the run; this aspect cannot be
    automated and must be confirmed by eye.
    """

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_led_on(self):
        """Send 16 short frames over internal loopback so the operator can
        confirm each frame produces a single Rx/Tx LED blink.

        The acceptance filter is configured to block all received frames
        (W2 + M00000000 + m00000000 -> STD 0x000 only), so any observed
        LED activity comes from the parse / transmit path and not from a
        post-filter Rx report. A 0.5 s gap between frames keeps individual
        blinks distinguishable to the human eye."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Simple filter + block-all so the LEDs are driven by parser/Tx only.
        self.dut.send(b"z0000\r")    # disable Tx event and Rx frame reporting
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"W2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")    # pass STD 0x000 only
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        print("")
        print("The both LED should lit 16 times.")
        print("")

        time.sleep(2)

        # 16 frames in four groups (STD/EXT x two ID values), paced for
        # human observation. Each ACK confirms the parser accepted the
        # command - which is what drives the LED blink.
        for cmd in cmd_send_std:
            time.sleep(0.5)
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD frame {cmd!r}03F0 should be ACKed (and trigger an LED blink)")

        for cmd in cmd_send_ext:
            time.sleep(0.5)
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT frame {cmd!r}0137FEC8 should be ACKed (and trigger an LED blink)")

        for cmd in cmd_send_std:
            time.sleep(0.5)
            self.dut.send(cmd + b"0000\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD frame {cmd!r}000 should be ACKed (and trigger an LED blink)")

        for cmd in cmd_send_ext:
            time.sleep(0.5)
            self.dut.send(cmd + b"000000000\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT frame {cmd!r}00000000 should be ACKed (and trigger an LED blink)")

        time.sleep(2)

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
