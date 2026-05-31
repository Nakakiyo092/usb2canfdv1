#!/usr/bin/env python3
"""Pre-release checklist tool.

Walks the operator through a sequence of manual checks that require
visual (LED) and content (printed output) confirmation.  Each step
prompts for y/n and fails on the first unconfirmed item.

Run order is significant: the startup LED check resets the device
(breaking the USB connection) and is therefore the last step.
"""

import unittest

import time
from device_under_test import DeviceUnderTest


class ToolPreReleaseChecklistTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()

    def tearDown(self):
        # The startup LED check resets the device, breaking the USB
        # connection; ignore any close error to preserve the test result.
        try:
            self.dut.close()
        except Exception:
            pass

    def _confirm(self, prompt: str):
        """Prompt the operator for y/n; fail the test on a non-y answer."""
        response = input(f"\n{prompt} (y/n): ")
        self.assertEqual(response.strip().lower(), "y",
                         f"Checklist item not confirmed: {prompt}")

    def test_checklist(self):
        self.dut.print_on = True

        # --- Version (V, v) ---
        print("\n=== Version ===")
        for cmd in (b"V\r", b"v\r"):
            self.dut.send(cmd)
            self.dut.receive()
        self._confirm("Does the version output look correct?")

        # --- Controller info (I, i) ---
        print("\n=== Controller info ===")
        for cmd in (b"I\r", b"i\r"):
            self.dut.send(cmd)
            self.dut.receive()
        self._confirm("Does the controller info output look correct?")

        # --- Serial number (N) ---
        print("\n=== Serial number ===")
        self.dut.send(b"N\r")
        self.dut.receive()
        self._confirm("Does the serial number look correct?")

        # --- CAN bus closed LED ---
        # Channel is closed by setup().  Expected per doc/5.-Hardware.md:
        #   RDY constant ON, TX constant ON, RX blinks on each command.
        print("\n=== CAN bus closed LED ===")
        print("Expected: RDY constant ON, TX constant ON,")
        print("          RX blinks on each command reception.")
        print("Sending 5 short commands with 0.5 s spacing...")
        for _ in range(5):
            self.dut.send(b"V\r")
            self.dut.receive()
            time.sleep(0.5)
        self._confirm("Did the LEDs behave as described?")

        # --- Error-stored LED ---
        # Trigger a bus error by sending to a quiet bus with no retransmit
        # so a stored flag gets set.  Per doc/5.-Hardware.md and the F
        # command note, any stored flag turns RX & TX to constant ON.
        print("\n=== Error-stored LED ===")
        print("Expected: RDY constant ON, RX & TX both constant ON.")
        self.dut.send(b"-\r")             # disable retransmit
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t0000\r")         # transmit -> NACK on quiet bus
        self.assertEqual(self.dut.receive(), b"z\r")
        time.sleep(0.5)                    # let the error flag settle
        self._confirm("Did the LEDs behave as described?")
        # Cleanup: F reads and clears the stored flag; then close channel.
        self.dut.send(b"F\r")
        self.dut.receive()
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- Startup LED (must be last; the device is reset here) ---
        print("\n=== Startup LED (device reset required) ===")
        print("Power-cycle or hardware-reset the device now.")
        print("Expected during startup:")
        print("  RDY: constant ON")
        print("  RX & TX: blink 5 times in total")
        input("\nPress Enter once the device has finished its startup...")
        self._confirm("Did the startup LEDs behave as described?")


if __name__ == "__main__":
    unittest.main()
