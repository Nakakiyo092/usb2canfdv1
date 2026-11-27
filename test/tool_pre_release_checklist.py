#!/usr/bin/env python3
"""Pre-release checklist tool.

Walks the operator through a sequence of manual checks that require
content (printed output) confirmation.  Each step prompts for y/n and
fails on the first unconfirmed item.

The LED behavior is checked separately with test_led.py.
"""

import unittest

from device_under_test import DeviceUnderTest


class ToolPreReleaseChecklistTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()

    def tearDown(self):
        self.dut.close()

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

        print("\nCheck the LED behavior with test_led.py as well.")


if __name__ == "__main__":
    unittest.main()
