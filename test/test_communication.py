#!/usr/bin/env python3

import sys
import unittest

from device_under_test import DeviceUnderTest


# NOTE: This test requires TWO SLCAN devices wired on the same CAN bus.
# - DUT: default port (COM9 on Windows, /dev/ttyACM0 on Linux)
# - AUX: second device (COM8 on Windows, /dev/ttyACM1 on Linux)
# Both devices must be physically connected to each other via the CAN bus
# (CAN-H to CAN-H, CAN-L to CAN-L, with proper termination).
class CommunicationTestCase(unittest.TestCase):
    """End-to-end two-device communication test.

    DUT (default port: COM9 on Windows / /dev/ttyACM0 on Linux) and AUX
    (COM8 / /dev/ttyACM1) are wired on the same CAN bus.

    For every supported (S, Y) bitrate combination where Y >= S and both
    devices accept the setup, sends 50 frames from each side and verifies
    each side receives all expected frames in order with no bus errors.
    """

    FRAMES_PER_SIDE = 50
    # Frame mix per side: 15 classic + 15 FD-noBRS + 20 FD-BRS
    NUM_CLASSIC = 15
    NUM_FD_NO_BRS = 15
    NUM_FD_BRS = 20

    dut: DeviceUnderTest
    aux: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.aux = DeviceUnderTest()
        self.dut.open()                              # default port
        self.aux.open(port=self._get_aux_port())     # OS-specific AUX port
        self.dut.setup()
        self.aux.setup()


    def tearDown(self):
        self.dut.close()
        self.aux.close()


    @staticmethod
    def _get_aux_port():
        """Return the hardcoded AUX port for the current OS."""
        if sys.platform == "win32":
            return "COM8"
        if sys.platform.startswith("linux"):
            return "/dev/ttyACM1"
        return "XXX"    # TODO: put default AUX device name in the macOS


    def test_bidirectional_at_all_supported_bitrates(self):
        """Iterate over the full S=0..9 x Y=0..9 grid. Send 50 frames
        from each side and verify reception + F=00 on every combo
        that both devices accept.

        No bitrate-based pre-filter is applied: per the doc, every
        Y rate is >= 500 kbps which is >= every S rate (max S = 1 Mbps,
        practical Y range is 1 Mbps and above), so Y_kbps >= S_kbps
        always holds in practice and a Y < S filter would only mask
        legitimate test coverage. Unsupported indices (Y3 is N/A per
        doc, Y6-9 and S9 may not exist on a given firmware) are caught
        by the [BELL] response in _set_bitrate_or_skip and reported as
        skipTest.
        """
        for s in range(0, 10):
            for y in range(0, 10):
                with self.subTest(s=s, y=y):
                    self._run_one_combo(s, y)


    def _run_one_combo(self, s, y):
        """Run one (S, Y) combo: setup, send 50 frames each, verify, close.

        Setup is wrapped in skipTest so unsupported bitrates are skipped
        with a clear message rather than failing the subTest. The
        send/verify block is wrapped in try/finally so the CAN channel
        on both devices is always closed, even when an assertion fails
        — otherwise a leftover open channel would cause the very next
        subTest's S/Y commands to be rejected with [BELL].
        """
        # 1. Set S and Y on both devices; [BELL] -> skip this combo.
        self._set_bitrate_or_skip(self.dut, "DUT", b"S", s)
        self._set_bitrate_or_skip(self.aux, "AUX", b"S", s)
        self._set_bitrate_or_skip(self.dut, "DUT", b"Y", y)
        self._set_bitrate_or_skip(self.aux, "AUX", b"Y", y)

        # 2. Build frame lists.
        # DUT uses even IDs (0x000, 0x002, ..., 0x062);
        # AUX uses odd IDs  (0x001, 0x003, ..., 0x063).
        dut_frames = self._build_frames(start_id=0)
        aux_frames = self._build_frames(start_id=1)

        try:
            # 3. Open both devices in normal mode.
            for dev, name in [(self.dut, "DUT"), (self.aux, "AUX")]:
                dev.send(b"O\r")
                self.assertEqual(dev.receive(), b"\r",
                                 f"S{s}Y{y}: {name} failed to open in normal mode")

            # 4. Send all 50 frames from each side (interleaved push).
            for i in range(self.FRAMES_PER_SIDE):
                self.dut.send(dut_frames[i])
                self.aux.send(aux_frames[i])

            # 5. Drain both sides until the bus is quiet.
            dut_rx = self._drain_until_quiet(self.dut)
            aux_rx = self._drain_until_quiet(self.aux)

            # 6. Read F on both BEFORE asserting frame contents, so we
            # always record the bus-error state even when frame loss
            # would otherwise short-circuit the subTest at step 7.
            self.dut.send(b"F\r")
            dut_f = self.dut.receive()
            self.aux.send(b"F\r")
            aux_f = self.aux.receive()

            # 7. Collect both observations (frame loss and bus errors)
            # then fail with a single combined message.
            problems = []
            try:
                self._assert_received(dut_rx, aux_frames, "DUT", s, y)
            except AssertionError as e:
                problems.append(f"DUT rx: {e}")
            try:
                self._assert_received(aux_rx, dut_frames, "AUX", s, y)
            except AssertionError as e:
                problems.append(f"AUX rx: {e}")
            if dut_f != b"F00\r":
                problems.append(f"DUT F={dut_f!r}")
            if aux_f != b"F00\r":
                problems.append(f"AUX F={aux_f!r}")
            if problems:
                self.fail(f"S{s}Y{y}: " + " | ".join(problems))
        finally:
            # 8. Always close both channels. We drain the response but
            # do not assert on it; the goal is to leave both devices
            # in the closed state so the next subTest can re-configure
            # S/Y. C\r is idempotent on an already-closed channel.
            for dev in (self.dut, self.aux):
                dev.send(b"C\r")
                dev.receive()


    def _set_bitrate_or_skip(self, dev, dev_name, cmd_letter, n):
        """Try a bitrate command; skip this subTest if [BELL] is returned.

        Both S and Y must be tried separately on both devices, because
        either S or Y may individually be unsupported.
        """
        cmd = cmd_letter + str(n).encode() + b"\r"
        dev.send(cmd)
        r = dev.receive()
        if r == b"\a":
            self.skipTest(f"{cmd_letter.decode()}{n} not accepted by {dev_name}")
        self.assertEqual(r, b"\r",
                         f"{dev_name}: unexpected response to {cmd!r}: {r!r}")


    def _drain_until_quiet(self, dev):
        """Call dev.receive() repeatedly until it returns empty.

        The DUT's receive() returns as soon as it sees a single 20 ms
        gap, which at low bitrates can happen mid-burst. By calling
        again until one full receive() comes back empty, we make sure
        the bus has actually quieted down before we verify.
        """
        rx_data = b""
        while True:
            chunk = dev.receive()
            if not chunk:
                break
            rx_data += chunk
        return rx_data


    def _build_frames(self, start_id):
        """Build 50 frames with IDs start_id, start_id+2, ..., start_id+98.

        Mix: 15 t (classic) + 15 d (FD no BRS) + 20 b (FD BRS).
        Payload is 2 bytes holding the CAN ID value (for traceability).
        """
        frames = []
        for i in range(self.FRAMES_PER_SIDE):
            can_id = start_id + i * 2   # 0..98 step 2 or 1..99 step 2
            if i < self.NUM_CLASSIC:
                ftype = b"t"
            elif i < self.NUM_CLASSIC + self.NUM_FD_NO_BRS:
                ftype = b"d"
            else:
                ftype = b"b"
            frames.append(
                ftype
                + format(can_id, "03X").encode()
                + b"2"                                  # DLC = 2 bytes
                + format(can_id, "04X").encode()        # data = ID value (2 bytes hex)
                + b"\r"
            )
        return frames


    def _assert_received(self, rx_data, expected_frames, recipient, s, y):
        """Verify rx_data contains all expected_frames in order, after
        stripping the recipient's own z[CR] acks.

        Frame payloads are hex digits only, so [CR] only appears as a
        frame delimiter — splitting on [CR] is safe.
        """
        stripped = rx_data.replace(b"z\r", b"")
        received_msgs = [m for m in stripped.split(b"\r") if m]
        expected_msgs = [f.rstrip(b"\r") for f in expected_frames]
        self.assertEqual(received_msgs, expected_msgs,
                         f"S{s}Y{y}: {recipient} received frames mismatch")


if __name__ == "__main__":
    unittest.main()
