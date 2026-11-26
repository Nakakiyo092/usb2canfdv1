#!/usr/bin/env python3
"""Manual check of the LED behavior.

Walks the operator through the LED rules in doc/5.-Hardware.md. Each step
tells what will happen and how the LEDs should behave, waits for Enter,
creates the situation, and asks the operator to confirm by eye.

This test is not part of test_std_case. Run it when the LED behavior should
be checked, e.g. before a release.

Setup:
- Only the DUT on the CAN bus, with the termination. Do not open any other
  node, since the error steps rely on the missing ACK.
- Steps 2 and 4 need a DEBUG build, and are skipped otherwise.
- Step 6 asks to short CAN high and low, and step 7 to reset the device.
  Step 7 is the last one, as the reset breaks the USB connection.

A single step can be run, e.g.: python test_led.py LedTestCase.test_6_bus_off
"""

import unittest

import time
from device_under_test import DeviceUnderTest


class LedTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        # Step 7 resets the device and breaks the USB connection, so ignore errors here.
        try:
            self.dut.send(b"C\r")
            self.dut.receive()
            self.dut.close()
        except Exception:
            pass


    def announce(self, title: str, action: str, expected: tuple):
        """Tell the operator what will happen and how the LEDs should behave,
        then wait for Enter."""
        print("")
        print(f"=== {title} ===")
        print(f"Action:   {action}")
        for i, line in enumerate(expected):
            print(f"{'Expected: ' if i == 0 else '          '}{line}")
        input("Press Enter to start...")


    def confirm(self):
        """Ask the operator to confirm by eye; fail on a non-y answer."""
        response = input("Did the LEDs behave as described? (y/n): ")
        self.assertEqual(response.strip().lower(), "y", "LED behavior not confirmed by the operator")


    def test_1_bus_closed(self):
        """CAN bus closed: TX on, RX blinks by a command reception."""
        self.announce("CAN bus closed",
                      "Send 5 commands, 0.5 s apart.",
                      ("TX: constant ON",
                       "RX: blinks once per command (5 times)"))
        for _ in range(0, 5):
            self.dut.send(b"V\r")
            self.dut.receive()
            time.sleep(0.5)
        self.confirm()


    def test_2_bus_open(self):
        """CAN bus open: RX and TX blink by a CAN frame reception and transmission."""
        if not self.dut.debug_build:
            self.skipTest("Requires a DEBUG build (internal loopback)")
        self.dut.send(b"z0000\r")       # no reports, the LEDs do not depend on them
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.announce("CAN bus open",
                      "Send 8 frames in internal loopback, 0.5 s apart.",
                      ("RX & TX: blink once per frame (8 times)",
                       "RX & TX: OFF between the frames"))
        for i in range(0, 8):
            self.dut.send(b"t" + format(i, "03X").encode() + b"0\r")
            self.assertEqual(self.dut.receive(), b"z\r")
            time.sleep(0.5)
        self.confirm()


    def test_3_data_loss(self):
        """Data loss: RX on and TX off for a short time after any data is discarded."""
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.announce("Data loss",
                      "Flood the device with commands without reading the replies (about 6 s),"
                      " so that the replies are discarded.",
                      ("RX & TX: OFF until the buffers are full",
                       "RX: constant ON, TX: OFF while the replies are discarded",
                       "RX & TX: OFF again a moment after the flood"))
        # Same flood as BufferTestCase.test_message_loss_in_cdc_tx_buffer
        for _ in range(0, 6000 // 50):
            self.dut.ser.write(b"I\r" * 50)
            time.sleep(0.05)
        time.sleep(1)
        self.confirm()

        # Drain the replies, then check that data was discarded indeed
        for _ in range(0, 30):
            if not self.dut.receive():
                break
        self.dut.send(b"F\r")
        f_reply = self.dut.receive()
        self.assertTrue(f_reply.startswith(b"F") and int(f_reply[1:3], 16) & 0x01,
                        f"No data was discarded (F bit 0), so the step was not checked: {f_reply!r}")


    def test_4_protocol_error(self):
        """Error: RX and TX on for a short time after a CAN protocol error."""
        if not self.dut.debug_build:
            self.skipTest("Requires a DEBUG build (no retransmission)")
        self.dut.send(b"-\r")           # no retransmission: the frame fails only once
        self.assertEqual(self.dut.receive(), b"\r")

        self.announce("CAN protocol error",
                      "Send one frame, which fails once without an ACK (error active, TEC 8).",
                      ("RX & TX: ON for a moment",
                       "RX & TX: OFF after that"))
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(2)
        self.confirm()

        self.dut.send(b"F\r")
        f_reply = self.dut.receive()
        self.assertTrue(f_reply.startswith(b"F") and int(f_reply[1:3], 16) & 0x80,
                        f"No protocol error (F bit 7), so the step was not checked: {f_reply!r}")


    def test_5_error_passive(self):
        """Error: RX and TX on while the node is error passive. Reading F does not change it."""
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.announce("Error passive",
                      "Send one frame, which is retransmitted without an ACK until error passive;"
                      " then read F after 2 s, and close the channel after 2 s more.",
                      ("RX & TX: constant ON, also after F",
                       "TX: ON, RX: OFF after the close (CAN bus closed)"))
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(2)
        self.dut.send(b"F\r")
        self.dut.receive()
        time.sleep(2)
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")
        time.sleep(1)
        self.confirm()


    def test_6_bus_off(self):
        """Bus off: RX and TX blink together until the channel is closed."""
        input("\nShort CAN high and low, then press Enter...")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.announce("Bus off",
                      "Send one frame on the shorted bus, which drives the node into bus off;"
                      " close the channel after 5 s.",
                      ("RX & TX: blink together",
                       "TX: ON, RX: OFF after the close (CAN bus closed)"))
        self.dut.send(b"t0000\r")
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(5)
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")
        time.sleep(1)
        self.confirm()
        input("Remove the short, then press Enter...")


    def test_7_startup(self):
        """Startup: RX and TX blink 5 times in turn after a reset."""
        self.announce("Startup",
                      "Power-cycle or reset the device after pressing Enter.",
                      ("RDY: constant ON",
                       "RX & TX: blink 5 times in turn",
                       "TX: ON, RX: OFF after that (CAN bus closed)"))
        input("Press Enter once the device has finished its startup...")
        self.confirm()


if __name__ == "__main__":
    unittest.main()
