#!/usr/bin/env python3

import unittest

from device_under_test import DeviceUnderTest


class SlcanTestCase(unittest.TestCase):
    """Per-command syntax tests for the SLCAN protocol.

    Each method below covers one command character (or a small group):
    valid forms that produce a reply or ACK, state-gated behaviour
    (closed / normal / silent), and invalid forms that are rejected
    with [BELL]."""

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_blank_command(self):
        """A bare [CR] is a no-op and is acknowledged with [CR]."""
        self.dut.send(b"\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Blank command ([CR]) should be acknowledged with [CR]")


    def test_error_command(self):
        """A [BELL] in the stream is buffered until the next [CR];
        the buffered command is then rejected with [BELL]."""
        self.dut.send(b"\a")
        # No reply yet - the command is incomplete without a terminating [CR].
        self.assertEqual(self.dut.receive(), b"",
                         "Device should not respond before a terminating [CR]")
        self.dut.send(b"\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Buffered [BELL] should be rejected with [BELL] after [CR]")


    def test_too_long_command(self):
        """Commands longer than the MTU (~165 bytes) are rejected with [BELL]."""
        # 1 + 138 + 8 + 1 + 1 + 16 = 165 bytes is the MTU (incl. [CR] and 16-byte margin).
        for length in (163, 164, 165):
            for _ in range(length):
                self.dut.send(b"F")
            self.assertEqual(self.dut.receive(), b"",
                             f"Device should not respond before [CR] (length={length})")
            self.dut.send(b"\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Over-MTU command (length={length}) should be rejected with [BELL]")


    def test_V_command(self):
        """V[CR]: returns the short hardware/software version (Vxxxx[CR])."""
        self.dut.send(b"V\r")
        rx_data = self.dut.receive()
        self.assertGreaterEqual(len(rx_data), len(b"V1013\r"),
                                f"V reply too short: {rx_data!r}")
        self.assertEqual(rx_data[0], b"V"[0],
                         f"V reply should start with 'V': {rx_data!r}")
        # Any trailing characters before [CR] are invalid.
        self.dut.send(b"V0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "V with trailing characters should be rejected")


    def test_v_command(self):
        """v[CR]: returns the detailed version string."""
        self.dut.send(b"v\r")
        rx_data = self.dut.receive()
        self.assertGreaterEqual(len(rx_data), len(b"v\r"),
                                f"v reply too short: {rx_data!r}")
        self.assertEqual(rx_data[0], b"v"[0],
                         f"v reply should start with 'v': {rx_data!r}")
        self.dut.send(b"v0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "v with trailing characters should be rejected")


    def test_I_command(self):
        """I[CR]: returns the short CAN controller info (Ixxxx[CR])."""
        self.dut.send(b"I\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"I20A0\r"),
                         f"I reply has unexpected length: {rx_data!r}")
        self.assertEqual(rx_data[0], b"I"[0],
                         f"I reply should start with 'I': {rx_data!r}")
        self.dut.send(b"I0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "I with trailing characters should be rejected")


    def test_i_command(self):
        """i[CR]: returns the detailed CAN controller info string."""
        self.dut.send(b"i\r")
        rx_data = self.dut.receive()
        self.assertGreaterEqual(len(rx_data), len(b"i\r"),
                                f"i reply too short: {rx_data!r}")
        self.assertEqual(rx_data[0], b"i"[0],
                         f"i reply should start with 'i': {rx_data!r}")
        self.dut.send(b"i0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "i with trailing characters should be rejected")


    def test_N_command(self):
        """N[CR]: read serial number. NA123[CR] writes one. Other forms rejected."""
        # Read current serial.
        self.dut.send(b"N\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"NA123\r"),
                         f"N reply has unexpected length: {rx_data!r}")
        self.assertEqual(rx_data[0], b"N"[0],
                         f"N reply should start with 'N': {rx_data!r}")
        # Write the same serial back.
        self.dut.send(b"NA123\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Writing a valid serial should be ACKed")
        # Wrong-length variants are rejected.
        self.dut.send(b"N0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "N with 1 char should be rejected")
        self.dut.send(b"NA12\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "N with 4 chars should be rejected")
        self.dut.send(b"NA1230\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "N with 6 chars should be rejected")


    def test_open_close_command(self):
        """C / O / L bus-state transitions and the rejection of invalid combinations."""
        # O (normal mode) open/close cycle.
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "C on an already-closed bus should be rejected")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Second O on an already-open bus should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # L (silent mode) open/close cycle.
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\a")
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Second L on an already-open bus should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # O and L cannot switch directly; must close first.
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\a")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "L while in normal mode should be rejected; must C first")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "O while in silent mode should be rejected; must C first")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Trailing characters are invalid.
        self.dut.send(b"O0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "O with trailing chars should be rejected")
        self.dut.send(b"L0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "L with trailing chars should be rejected")
        self.dut.send(b"C0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "C with trailing chars should be rejected")


    def test_S_command(self):
        """Sn[CR]: select a nominal bitrate preset (n in 0..8). Only valid when closed."""
        # Closed: S0..S8 are accepted, S9 is rejected.
        for idx in range(0, 10):
            cmd = ("S" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            if idx <= 8:
                self.assertEqual(self.dut.receive(), b"\r",
                                 f"S{idx} should be accepted when closed")
            else:
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"S{idx} (out of range) should be rejected")

        # Bitrate cannot be changed while the bus is open (normal mode).
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("S" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"S{idx} should be rejected while open in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Same for silent mode.
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("S" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"S{idx} should be rejected while open in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Invalid forms.
        self.dut.send(b"S\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Bare S should be rejected")
        self.dut.send(b"S00\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "S with 2 digits should be rejected")
        self.dut.send(b"SG\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "S with non-digit should be rejected")


    def test_s_command(self):
        """sddxxyyzz[CR] (long form): set nominal bitrate by raw BTR values."""
        # Valid form, closed: accepted.
        self.dut.send(b"s10460908\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Valid s command should be accepted when closed")

        # Rejected while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"s10460908\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "s should be rejected while open in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"s10460908\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "s should be rejected while open in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Boundary values within range.
        self.dut.send(b"s01010101\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Min in-range BTR values should be accepted")
        self.dut.send(b"sFFFF8080\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Max in-range BTR values should be accepted")

        # Each field out of range -> rejected.
        for bad in (b"s00460908", b"s10000908", b"s10460008", b"s10468108",
                    b"s10460900", b"s10460981"):
            self.dut.send(bad + b"\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Out-of-range BTR field should be rejected: {bad!r}")

        # Wrong length / non-hex character.
        self.dut.send(b"s1046090\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Short s command should be rejected")
        self.dut.send(b"s104609080\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Long s command should be rejected")
        self.dut.send(b"s0G460908\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "s with non-hex character should be rejected")


    def test_sxxyy_command(self):
        """sxxyy[CR] (short LAWICEL-compatible form): set nominal bitrate by BTR0/BTR1."""
        # Default LAWICEL: 125 kbps @ 87.5% sample point.
        self.dut.send(b"s031C\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "sxxyy should be accepted when closed")

        # Rejected while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"s031C\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "sxxyy should be rejected while open in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"s031C\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "sxxyy should be rejected while open in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Standard LAWICEL bit rates (all use BTR1=0x1C for 87.5% SP).
        for cmd, label in ((b"s311C", "10kbps"), (b"s181C", "20kbps"),
                           (b"s091C", "50kbps"), (b"s041C", "100kbps"),
                           (b"s011C", "250kbps"), (b"s001C", "500kbps"),
                           (b"s0016", "800kbps"), (b"s0014", "1Mbps")):
            self.dut.send(cmd + b"\r")
            self.assertEqual(self.dut.receive(), b"\r",
                             f"Standard LAWICEL preset {label} ({cmd!r}) should be accepted")

        # Boundary values (formulas always produce in-range results).
        self.dut.send(b"s0000\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "sxxyy with all-zero should be accepted")
        self.dut.send(b"sFFFF\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "sxxyy with all-ones should be accepted")

        # Sampling mode bit (MSB of yy) is ignored.
        self.dut.send(b"s031C\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"s039C\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "sxxyy with sample-mode bit set should still be accepted")

        # Wrong length / non-hex.
        self.dut.send(b"s031\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Short sxxyy should be rejected")
        self.dut.send(b"s031C0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Long sxxyy should be rejected")
        self.dut.send(b"s0G1C\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "sxxyy with non-hex character should be rejected")


    def test_Y_command(self):
        """Yn[CR]: select a data-phase bitrate preset (n in 0,1,2,4,5). Only valid when closed."""
        # Closed: Y0,Y1,Y2,Y4,Y5 accepted; others rejected.
        for idx in range(0, 10):
            cmd = ("Y" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            if idx in (0, 1, 2, 4, 5):
                self.assertEqual(self.dut.receive(), b"\r",
                                 f"Y{idx} should be accepted when closed")
            else:
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"Y{idx} (unsupported preset) should be rejected")

        # Rejected while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("Y" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Y{idx} should be rejected while open in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("Y" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Y{idx} should be rejected while open in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Invalid forms.
        self.dut.send(b"Y\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Bare Y should be rejected")
        self.dut.send(b"Y00\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Y with 2 digits should be rejected")
        self.dut.send(b"YG\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Y with non-digit should be rejected")


    def test_y_command(self):
        """yddxxyyzz[CR]: set data-phase bitrate by raw BTR values."""
        # Valid form, closed: accepted.
        self.dut.send(b"y021E0908\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Valid y command should be accepted when closed")

        # Rejected while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"y021E0908\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "y should be rejected while open in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"y021E0908\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "y should be rejected while open in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Boundary values within range.
        self.dut.send(b"y01010101\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Min in-range data BTR values should be accepted")
        self.dut.send(b"y20201010\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "Max in-range data BTR values should be accepted")

        # Each field out of range.
        for bad in (b"y001E0908", b"y211E0908", b"y02000908", b"y02210908",
                    b"y021E0008", b"y021E1108", b"y021E0900", b"y021E0911"):
            self.dut.send(bad + b"\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Out-of-range data BTR field should be rejected: {bad!r}")

        # Wrong length / non-hex.
        self.dut.send(b"y021E090\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Short y command should be rejected")
        self.dut.send(b"y021E09080\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Long y command should be rejected")
        self.dut.send(b"y0G1E0908\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "y with non-hex character should be rejected")


    def test_Z_command(self):
        """Zn[CR]: timestamp mode (0=off, 1=ms, 2=us). Z[CR] queries current time."""
        # Z[CR] is rejected while timestamp is disabled.
        self.dut.send(b"Z0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Z[CR] should be rejected while timestamp is disabled")

        # Z[CR] returns Z1xxxx[CR] in ms mode.
        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"Z1xxxx\r"),
                         f"Z[CR] reply in ms mode has unexpected length: {rx_data!r}")
        self.assertEqual(rx_data[0], b"Z"[0],
                         f"Z[CR] reply should start with 'Z': {rx_data!r}")

        # Z[CR] returns Z2xxxxxxxx[CR] in us mode.
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z\r")
        rx_data = self.dut.receive()
        self.assertEqual(len(rx_data), len(b"Z2xxxxxxxx\r"),
                         f"Z[CR] reply in us mode has unexpected length: {rx_data!r}")
        self.assertEqual(rx_data[0], b"Z"[0],
                         f"Z[CR] reply should start with 'Z': {rx_data!r}")

        # Closed: only Z0/Z1/Z2 are accepted.
        for idx in range(0, 10):
            cmd = ("Z" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            if idx in range(0, 3):
                self.assertEqual(self.dut.receive(), b"\r",
                                 f"Z{idx} should be accepted when closed")
            else:
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"Z{idx} (out of range) should be rejected")

        # Mode cannot be changed while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("Z" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Z{idx} should be rejected while open in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("Z" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Z{idx} should be rejected while open in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Invalid forms.
        # Note: bare Z[CR] is now valid (queries current timestamp).
        self.dut.send(b"Z00\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Z with 2 digits should be rejected")
        self.dut.send(b"ZG\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Z with non-digit should be rejected")


    def test_z_command(self):
        """zNxxx[CR]: detailed report-mode config. z[CR] queries detailed time."""
        # z[CR] returns the detailed time reply.
        self.dut.send(b"z\r")
        rx_data = self.dut.receive()
        self.assertGreaterEqual(len(rx_data), len(b"z\r"),
                                f"z[CR] reply too short: {rx_data!r}")
        self.assertEqual(rx_data[0], b"z"[0],
                         f"z[CR] reply should start with 'z': {rx_data!r}")

        # Closed: z0xxx..z2xxx accepted, others rejected.
        for idx in range(0, 10):
            cmd = ("z" + str(idx) + "000\r").encode()
            self.dut.send(cmd)
            if idx in range(0, 3):
                self.assertEqual(self.dut.receive(), b"\r",
                                 f"z{idx}000 should be accepted when closed")
            else:
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"z{idx}000 (out of range) should be rejected")

        # Rejected while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("z" + str(idx) + "000\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"z{idx}000 should be rejected while open in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("z" + str(idx) + "000\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"z{idx}000 should be rejected while open in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Wrong length / non-hex.
        self.dut.send(b"z0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "z with 1 char should be rejected")
        self.dut.send(b"z000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "z with 3 chars should be rejected")
        self.dut.send(b"z00000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "z with 5 chars should be rejected")
        self.dut.send(b"zG000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "z with non-hex should be rejected")


    def test_F_command(self):
        """F[CR]: read status flags. Only valid while the bus is open."""
        # Closed -> rejected.
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "F should be rejected while closed")

        # Normal mode -> Fxx[CR].
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F in normal mode should report F00 at idle")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Silent mode -> Fxx[CR].
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "F in silent mode should report F00 at idle")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Trailing characters are invalid.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"F0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "F with trailing chars should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_f_command(self):
        """f[CR]: read detailed status. Only valid while the bus is open."""
        # Closed -> rejected.
        self.dut.send(b"f\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "f should be rejected while closed")

        # Normal mode -> detailed string.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertGreaterEqual(len(rx_data), len(b"f\r"),
                                f"f reply too short: {rx_data!r}")
        self.assertEqual(rx_data[0], b"f"[0],
                         f"f reply should start with 'f': {rx_data!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Silent mode -> detailed string.
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"f\r")
        rx_data = self.dut.receive()
        self.assertGreaterEqual(len(rx_data), len(b"f\r"),
                                f"f reply too short: {rx_data!r}")
        self.assertEqual(rx_data[0], b"f"[0],
                         f"f reply should start with 'f': {rx_data!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Trailing characters are invalid.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"f0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "f with trailing chars should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_W_command(self):
        """Wn[CR]: filter mode (0=dual, 2=simple; W1 unsupported). Only valid when closed."""
        # Closed: W0 and W2 accepted, W1 and others rejected.
        for idx in range(0, 10):
            cmd = ("W" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            if idx in (0, 2):
                self.assertEqual(self.dut.receive(), b"\r",
                                 f"W{idx} should be accepted when closed")
            else:
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"W{idx} (unsupported / out of range) should be rejected")

        # Rejected while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("W" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"W{idx} should be rejected while open in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("W" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"W{idx} should be rejected while open in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Invalid forms.
        self.dut.send(b"W\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Bare W should be rejected")
        self.dut.send(b"W00\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "W with 2 digits should be rejected")
        self.dut.send(b"WG\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "W with non-digit should be rejected")


    def test_M_command(self):
        """Mxxxxxxxx[CR]: set filter Code. Only valid when closed; 8 hex digits required."""
        # Closed: any 8-hex value accepted.
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "M with min value should be accepted")
        self.dut.send(b"MFFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "M with max value should be accepted")

        # Rejected while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "M should be rejected while open in normal mode")
        self.dut.send(b"MFFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"\a")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"M00000000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "M should be rejected while open in silent mode")
        self.dut.send(b"MFFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"\a")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Wrong length / non-hex.
        self.dut.send(b"M0000000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Short M should be rejected")
        self.dut.send(b"M000000000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Long M should be rejected")
        self.dut.send(b"M0000000G\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "M with non-hex character should be rejected")


    def test_m_command(self):
        """mxxxxxxxx[CR]: set filter Mask. Only valid when closed; 8 hex digits required."""
        # Closed: any 8-hex value accepted.
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "m with min value should be accepted")
        self.dut.send(b"mFFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"\r",
                         "m with max value should be accepted")

        # Rejected while open.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "m should be rejected while open in normal mode")
        self.dut.send(b"mFFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"\a")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"m00000000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "m should be rejected while open in silent mode")
        self.dut.send(b"mFFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"\a")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Wrong length / non-hex.
        self.dut.send(b"m0000000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Short m should be rejected")
        self.dut.send(b"m000000000\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Long m should be rejected")
        self.dut.send(b"m0000000G\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "m with non-hex character should be rejected")


    def test_Q_command(self):
        """Qn[CR]: auto-startup mode (n in 0..2). Only valid while the bus is open."""
        # Closed: rejected.
        for idx in range(0, 10):
            cmd = ("Q" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Q{idx} should be rejected while closed")

        # Normal mode: Q0..Q2 accepted, others rejected.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("Q" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            if idx in (0, 1, 2):
                self.assertEqual(self.dut.receive(), b"\r",
                                 f"Q{idx} should be accepted while open in normal mode")
            else:
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"Q{idx} (out of range) should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Silent mode: same set accepted.
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for idx in range(0, 10):
            cmd = ("Q" + str(idx) + "\r").encode()
            self.dut.send(cmd)
            if idx in (0, 1, 2):
                self.assertEqual(self.dut.receive(), b"\r",
                                 f"Q{idx} should be accepted while open in silent mode")
            else:
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"Q{idx} (out of range) should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Invalid forms (while open).
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Q\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Bare Q should be rejected")
        self.dut.send(b"Q00\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Q with 2 digits should be rejected")
        self.dut.send(b"QG\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "Q with non-digit should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_send_commands(self):
        """r/t/d/b (STD) and R/T/D/B (EXT) frame-send commands: validity per bus state,
        minimum and maximum lengths, and rejection of malformed forms."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Closed -> all send commands rejected.
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"STD {cmd!r} should be rejected while closed")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"EXT {cmd!r} should be rejected while closed")

        # Normal mode -> sends ACKed with z[CR] / Z[CR].
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"STD {cmd!r} should be accepted in normal mode")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"EXT {cmd!r} should be accepted in normal mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Minimum / shortest valid frames.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd, body in (
            (b"r", b"0000"), (b"t", b"0000"), (b"d", b"0000"), (b"b", b"0000"),
        ):
            self.dut.send(cmd + body + b"\r")
            self.assertEqual(self.dut.receive(), b"z\r",
                             f"Minimum STD {cmd!r} frame should be accepted")
        for cmd, body in (
            (b"R", b"000000000"), (b"T", b"000000000"),
            (b"D", b"000000000"), (b"B", b"000000000"),
        ):
            self.dut.send(cmd + body + b"\r")
            self.assertEqual(self.dut.receive(), b"Z\r",
                             f"Minimum EXT {cmd!r} frame should be accepted")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Maximum / longest valid frames (DLC 0xF carries 8 / 64 byte data depending on type).
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"r7FFF\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "Maximum STD r frame should be accepted")
        self.dut.send(b"t7FFF" + b"FF" * 8 + b"\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "Maximum STD t frame (8 bytes) should be accepted")
        self.dut.send(b"d7FFF" + b"FF" * 64 + b"\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "Maximum STD d frame (64 bytes) should be accepted")
        self.dut.send(b"b7FFF" + b"FF" * 64 + b"\r")
        self.assertEqual(self.dut.receive(), b"z\r",
                         "Maximum STD b frame (64 bytes) should be accepted")
        self.dut.send(b"R1FFFFFFFF\r")
        self.assertEqual(self.dut.receive(), b"Z\r",
                         "Maximum EXT R frame should be accepted")
        self.dut.send(b"T1FFFFFFFF" + b"FF" * 8 + b"\r")
        self.assertEqual(self.dut.receive(), b"Z\r",
                         "Maximum EXT T frame (8 bytes) should be accepted")
        self.dut.send(b"D1FFFFFFFF" + b"FF" * 64 + b"\r")
        self.assertEqual(self.dut.receive(), b"Z\r",
                         "Maximum EXT D frame (64 bytes) should be accepted")
        self.dut.send(b"B1FFFFFFFF" + b"FF" * 64 + b"\r")
        self.assertEqual(self.dut.receive(), b"Z\r",
                         "Maximum EXT B frame (64 bytes) should be accepted")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Silent mode -> send commands are rejected (listen-only).
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"STD {cmd!r} should be rejected in silent mode")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"EXT {cmd!r} should be rejected in silent mode")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Too short / too long command lengths.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Too-short STD {cmd!r} should be rejected")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F10\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Trailing extra char on STD {cmd!r} should be rejected")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC8\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Too-short EXT {cmd!r} should be rejected")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC810\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Trailing extra char on EXT {cmd!r} should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Extra char after a maximum-length frame.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for tail in (b"r03FF0", b"t03FF" + b"00" * 8 + b"0",
                     b"d03FF" + b"00" * 64 + b"0", b"b03FF" + b"00" * 64 + b"0",
                     b"R0137FEC8F0", b"T0137FEC8F" + b"00" * 8 + b"0",
                     b"D0137FEC8F" + b"00" * 64 + b"0",
                     b"B0137FEC8F" + b"00" * 64 + b"0"):
            self.dut.send(tail + b"\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             "Max-length frame with one extra char should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Extra byte (two hex chars) after a maximum-length frame.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for tail in (b"r03FF00", b"t03FF" + b"00" * 8 + b"00",
                     b"d03FF" + b"00" * 64 + b"00", b"b03FF" + b"00" * 64 + b"00",
                     b"R0137FEC8F00", b"T0137FEC8F" + b"00" * 8 + b"00",
                     b"D0137FEC8F" + b"00" * 64 + b"00",
                     b"B0137FEC8F" + b"00" * 64 + b"00"):
            self.dut.send(tail + b"\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             "Max-length frame with one extra byte should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # DLC vs payload-length mismatch.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for prefix in (b"t03F4", b"d03F4", b"b03F4"):
            for pad in (b"00" * 3, b"00" * 5):
                self.dut.send(prefix + pad + b"\r")
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"DLC vs payload mismatch should be rejected: {prefix!r}+{len(pad)//2} bytes")
        for prefix in (b"T0137FEC84", b"D0137FEC84", b"B0137FEC84"):
            for pad in (b"00" * 3, b"00" * 5):
                self.dut.send(prefix + pad + b"\r")
                self.assertEqual(self.dut.receive(), b"\a",
                                 f"DLC vs payload mismatch should be rejected: {prefix!r}+{len(pad)//2} bytes")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Non-hex characters in the ID field.
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03FG\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"STD {cmd!r} with non-hex ID should be rejected")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC8G\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"EXT {cmd!r} with non-hex ID should be rejected")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # ID out of range (STD bit 11 set, or EXT bit 29 set).
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for bad in (b"r8000", b"t8000", b"d8000", b"b8000",
                    b"R200000000", b"T200000000", b"D200000000", b"B200000000"):
            self.dut.send(bad + b"\r")
            self.assertEqual(self.dut.receive(), b"\a",
                             f"Out-of-range ID should be rejected: {bad!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_unsupported_commands(self):
        """P, A, X, U: known LAWICEL commands that this firmware does not implement;
        they must all be rejected with [BELL]."""
        # P: Poll incoming FIFO.
        self.dut.send(b"P\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "P (unsupported) should be rejected")
        # A: Poll all pending frames.
        self.dut.send(b"A\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "A (unsupported) should be rejected")
        # X: Set Auto Poll / Send.
        self.dut.send(b"X0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "X (unsupported) should be rejected")
        # U: Set UART baud rate.
        self.dut.send(b"U0\r")
        self.assertEqual(self.dut.receive(), b"\a",
                         "U (unsupported) should be rejected")


if __name__ == "__main__":
    unittest.main()
