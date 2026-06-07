#!/usr/bin/env python3

import unittest

import time
from device_under_test import DeviceUnderTest


class InLoopbackTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_internal_loopback(self):
        """Verify lossless loopback for all 8 frame types.

        For each combination of:
          * frame type: data (t/T/d/D/b/B) or remote (r/R)
          * ID width: standard (lowercase) or extended (uppercase)
          * data phase: classic, CAN-FD without BRS, CAN-FD with BRS

        send both the minimum-length payload (single byte / no data)
        and the maximum-length payload (8 bytes for classic, 64 bytes
        for CAN-FD), and verify the looped-back frame matches
        byte-for-byte.
        """
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check loopback of shortest frames of each type
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r",
                             f"Short std loopback mismatch for {cmd!r}")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r",
                             f"Short ext loopback mismatch for {cmd!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Check loopback of longest frames of each type
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        tx_data = b"r03FF\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                         "Long remote std (r) loopback mismatch")
        tx_data = b"t03F80011223344556677\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                         "Long data std (t, 8 bytes) loopback mismatch")
        tx_data = b"d03FF" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                         "Long CAN-FD std no-BRS (d, 64 bytes) loopback mismatch")
        tx_data = b"b03FF" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"z\r" + tx_data,
                         "Long CAN-FD std BRS (b, 64 bytes) loopback mismatch")
        tx_data = b"R0137FEC8F\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data,
                         "Long remote ext (R) loopback mismatch")
        tx_data = b"T0137FEC880011223344556677\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data,
                         "Long data ext (T, 8 bytes) loopback mismatch")
        tx_data = b"D0137FEC8F" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data,
                         "Long CAN-FD ext no-BRS (D, 64 bytes) loopback mismatch")
        tx_data = b"B0137FEC8F" + b"00112233445566778899AABBCCDDEEFF" * 4 + b"\r"
        self.dut.send(tx_data)
        self.assertEqual(self.dut.receive(), b"Z\r" + tx_data,
                         "Long CAN-FD ext BRS (B, 64 bytes) loopback mismatch")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_tx_off_rx_on(self):
        """Verify reporting mode z0001: Tx event OFF, Rx frame ON.

        Each Tx command should ack with z[CR] (LAWICEL buffer-save
        response) and the looped-back frame should be reported as a
        standard Rx frame report.
        """
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check tx event is disabled and rx frame is enabled
        self.dut.send(b"z0001\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r",
                             f"z0001: expected Rx report for std {cmd!r}")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r",
                             f"z0001: expected Rx report for ext {cmd!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_tx_on_rx_off(self):
        """Verify reporting mode z0002: Tx event ON, Rx frame OFF.

        Each Tx command should ack with [CR] (bare carriage return)
        and emit a Tx event report (z<frame> / Z<frame>) without a
        separate Rx frame report.
        """
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check tx event is enabled and rx frame is disabled
        self.dut.send(b"z0002\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"\r" + b"z" + cmd + b"03F0\r",
                             f"z0002: expected Tx event for std {cmd!r}")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"\r" + b"Z" + cmd + b"0137FEC80\r",
                             f"z0002: expected Tx event for ext {cmd!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_esi_on(self):
        """Verify reporting mode z0013: ESI field included for CAN-FD only.

        Classic CAN frames (r/R/t/T) report without ESI; CAN-FD
        frames (d/D/b/B) report with ESI in both the Rx frame and
        the Tx event report. The relative order of Tx event and Rx
        frame is not guaranteed, so both orderings are accepted.

        ESI bit value verification (0 = error-active, 1 = error-passive)
        is out of scope here: internal loopback cannot drive the DUT
        into error-passive. That coverage is planned in test_dominant.py.
        """
        #self.dut.print_on = True
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Check CC frames are reported without ESI and FD frames with ESI (Rx frame and Tx event).
        # ESI bit value verification is tracked elsewhere; see method docstring.
        self.dut.send(b"z0013\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            if cmd in (b"r", b"t"):
                rx_data = self.dut.receive()
                self.assertEqual(len(rx_data), len(b"\r" + b"z" + cmd + b"03F0\r" + cmd + b"03F0\r"),
                                 f"z0013 CC std {cmd!r}: response length mismatch (ESI must be absent)")
                if rx_data[1] == b"z"[0]:
                    self.assertEqual(rx_data, b"\r" + b"z" + cmd + b"03F0\r" + cmd + b"03F0\r",
                                     f"z0013 CC std {cmd!r}: Tx-first order content mismatch")
                else:
                    self.assertEqual(rx_data, b"\r" + cmd + b"03F0\r" + b"z" + cmd + b"03F0\r",
                                     f"z0013 CC std {cmd!r}: Rx-first order content mismatch")
            else:
                rx_data = self.dut.receive()
                self.assertEqual(len(rx_data), len(b"\r" + b"z" + cmd + b"03F00\r" + cmd + b"03F00\r"),
                                 f"z0013 FD std {cmd!r}: response length mismatch (ESI must be present)")
                if rx_data[1] == b"z"[0]:
                    self.assertEqual(rx_data, b"\r" + b"z" + cmd + b"03F00\r" + cmd + b"03F00\r",
                                     f"z0013 FD std {cmd!r}: Tx-first order content mismatch")
                else:
                    self.assertEqual(rx_data, b"\r" + cmd + b"03F00\r" + b"z" + cmd + b"03F00\r",
                                     f"z0013 FD std {cmd!r}: Rx-first order content mismatch")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            if cmd in (b"R", b"T"):
                rx_data = self.dut.receive()
                self.assertEqual(len(rx_data), len(b"\r" + b"Z" + cmd + b"0137FEC80\r" + cmd + b"0137FEC80\r"),
                                 f"z0013 CC ext {cmd!r}: response length mismatch (ESI must be absent)")
                if rx_data[1] == b"Z"[0]:
                    self.assertEqual(rx_data, b"\r" + b"Z" + cmd + b"0137FEC80\r" + cmd + b"0137FEC80\r",
                                     f"z0013 CC ext {cmd!r}: Tx-first order content mismatch")
                else:
                    self.assertEqual(rx_data, b"\r" + cmd + b"0137FEC80\r" + b"Z" + cmd + b"0137FEC80\r",
                                     f"z0013 CC ext {cmd!r}: Rx-first order content mismatch")
            else:
                rx_data = self.dut.receive()
                self.assertEqual(len(rx_data), len(b"\r" + b"Z" + cmd + b"0137FEC800\r" + cmd + b"0137FEC800\r"),
                                 f"z0013 FD ext {cmd!r}: response length mismatch (ESI must be present)")
                if rx_data[1] == b"Z"[0]:
                    self.assertEqual(rx_data, b"\r" + b"Z" + cmd + b"0137FEC800\r" + cmd + b"0137FEC800\r",
                                     f"z0013 FD ext {cmd!r}: Tx-first order content mismatch")
                else:
                    self.assertEqual(rx_data, b"\r" + cmd + b"0137FEC800\r" + b"Z" + cmd + b"0137FEC800\r",
                                     f"z0013 FD ext {cmd!r}: Rx-first order content mismatch")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


class TimestampMsTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_format_when_off(self):
        """When timestamp mode is off (default), reports for all frame types
        carry no timestamp suffix."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r",
                             f"Frame {cmd!r} should be reported without timestamp")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r",
                             f"Frame {cmd!r} should be reported without timestamp")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_format_when_on(self):
        """With Z1, every report appends a 4-hex-digit ms timestamp."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            rx_data = self.dut.receive()
            self.assertEqual(len(rx_data), len(b"z\r" + cmd + b"03F0TTTT\r"),
                             f"Frame {cmd!r}: expected 4-char timestamp suffix, got {rx_data!r}")
            self.assertEqual(rx_data[:len(b"z\r" + cmd + b"03F0")], b"z\r" + cmd + b"03F0",
                             f"Frame {cmd!r}: body mismatch, got {rx_data!r}")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            rx_data = self.dut.receive()
            self.assertEqual(len(rx_data), len(b"Z\r" + cmd + b"0137FEC80TTTT\r"),
                             f"Frame {cmd!r}: expected 4-char timestamp suffix, got {rx_data!r}")
            self.assertEqual(rx_data[:len(b"Z\r" + cmd + b"0137FEC80")], b"Z\r" + cmd + b"0137FEC80",
                             f"Frame {cmd!r}: body mismatch, got {rx_data!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_mode_toggle(self):
        """Switching Z1 -> Z0 restores the no-timestamp format, confirming
        the mode setter is not sticky."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Turn on, then off.
        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r",
                             f"Frame {cmd!r} should be reported without timestamp after Z1->Z0")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r",
                             f"Frame {cmd!r} should be reported without timestamp after Z1->Z0")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_z_Z_mutual_exclusivity(self):
        """Issuing Z resets all z-command settings to their defaults.

        The z command note states: "settings made by z will be overwritten by
        the Z command or reset to default."

        Procedure:
        1. Configure with z2003 (microsecond timestamp, tx event on, rx on).
        2. Issue Z1 (millisecond timestamp).
        3. Open internal loopback and send a frame.

        Expected after Z1 (default reporting + ms timestamp):
        - Tx event disabled  -> buffer save response is z[CR]
        - Rx frame enabled   -> loopback frame is reported
        - Millisecond timestamp (4 hex chars), no microsecond (8 chars)

        Wrong if z2003 persisted:
        - Tx event on -> buffer save response is [CR]
        - Response would include separate tx event and rx frame reports
        - 8-char microsecond timestamps
        """
        #self.dut.print_on = True
        # Configure z2003: us timestamp, rx on, tx event on, ESI off
        self.dut.send(b"z2003\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Z1 must overwrite z settings and reset reporting to default
        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive() + self.dut.receive()

        # With Z1 + defaults: z[CR] (tx event off) + t03F0TTTT[CR] (ms ts, rx on)
        self.assertEqual(len(rx_data), len(b"z\rt03F0TTTT\r"),
                         "Z1 must override z2003: expected tx-event-off and ms timestamp (4 chars)")
        self.assertEqual(rx_data[:len(b"z\rt03F0")], b"z\rt03F0")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_basic_accuracy_30s(self):
        """Send two frames 30 s apart and confirm the reported ms timestamp
        difference matches the host-side sleep within 600 ms (2% of 30 s).
        The tolerance absorbs USB latency and OS scheduling jitter."""
        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # First frame -> grab baseline timestamp.
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        last_timestamp = rx_data[len(b"z\r" + b"t03F0"):len(b"z\r" + b"t03F0") + 4]
        last_time_ms = int(last_timestamp.decode(), 16)

        # Sleep across the gap we want to measure.
        sleep_time_ms = 30 * 1000
        time.sleep(sleep_time_ms / 1000.0)

        # Second frame -> compute device-side elapsed time.
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        crnt_timestamp = rx_data[len(b"z\r" + b"t03F0"):len(b"z\r" + b"t03F0") + 4]
        crnt_time_ms = int(crnt_timestamp.decode(), 16)
        if crnt_time_ms > last_time_ms:
            diff_time_ms = crnt_time_ms - last_time_ms
        else:
            diff_time_ms = (60000 + crnt_time_ms) - last_time_ms

        self.assertLess(abs(sleep_time_ms - diff_time_ms), 600,
                        f"device elapsed {diff_time_ms} ms differs from host sleep {sleep_time_ms} ms by >=600 ms")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_wraparound_milli(self):
        """The ms timestamp wraps to 0 at the documented boundary
        0xEA60 (60000 ms). Polls Z[CR] for current position, sleeps until
        just before the wrap, then loopbacks frames until the wrap is
        observed. Asserts the pre-wrap value is just below 0xEA60 and the
        post-wrap value is near 0."""
        WRAP_MS = 0xEA60  # 60000 ms

        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Query current position in the 60-second cycle.
        self.dut.send(b"Z\r")
        rx = self.dut.receive()
        self.assertEqual(len(rx), len(b"Z1xxxx\r"),
                         f"Z query reply has unexpected length, got: {rx!r}")
        current_ms = int(rx[2:6], 16)

        # Sleep until ~1 second before the wrap so the polling loop is short.
        ms_until_wrap = (WRAP_MS - current_ms) % WRAP_MS
        if ms_until_wrap > 1000:
            time.sleep((ms_until_wrap - 1000) / 1000.0)

        # Loopback frames at ~40 ms/frame until a wrap-around is detected.
        pre_wrap_ts = None
        post_wrap_ts = None
        wrap_detected = False
        for _ in range(200):  # ~8 s of polling
            self.dut.send(b"t03F0\r")
            rx = self.dut.receive()
            self.assertEqual(len(rx), len(b"z\rt03F0TTTT\r"),
                             f"Loopback frame reply has unexpected length, got: {rx!r}")
            ts_ms = int(rx[len(b"z\rt03F0"):len(b"z\rt03F0") + 4], 16)

            # A monotonic decrease across two consecutive samples is the wrap.
            if pre_wrap_ts is not None and ts_ms < pre_wrap_ts:
                post_wrap_ts = ts_ms
                wrap_detected = True
                break
            pre_wrap_ts = ts_ms

        self.assertTrue(wrap_detected,
                        "Millisecond timestamp wrap-around not detected within test window")
        self.assertGreaterEqual(pre_wrap_ts, WRAP_MS - 50,
                                f"Wrap occurred too early: last ts={pre_wrap_ts:#06x}")
        self.assertLess(pre_wrap_ts, WRAP_MS,
                        f"Pre-wrap ts must be < {WRAP_MS:#06x}, got {pre_wrap_ts:#06x}")
        self.assertLess(post_wrap_ts, 50,
                        f"Post-wrap ts should be near 0, got {post_wrap_ts:#06x}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_invalid_when_stalled(self):
        """When the main loop is stalled beyond the SOF-to-report design
        window (~20 ms), the ms timestamp must be reported as the 0xFFFF
        sentinel while the frame data itself remains intact.

        Uses the DEBUG-only stall command ~<HHHH>[CR] to block the main
        loop. CAN bitrate is set to 10 kbps so the loopback frame arrives
        during the stall, forcing a long SOF-to-report delay."""
        self.dut.send(b"S0\r")          # 10 kbps: frame arrives mid-stall.
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z1\r")          # ms timestamp on
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")           # open internal loopback
        self.assertEqual(self.dut.receive(), b"\r")

        # Enqueue Tx, give it 2 ms to start, then stall the loop for 40 ms.
        self.dut.send(b"T0137FEC880011223344556677\r")
        time.sleep(0.002)
        self.dut.send(b"~0028\r")

        # Drain output for >40 ms so the post-stall report is captured.
        rx_data = self.dut.receive() + self.dut.receive() + self.dut.receive()

        self.assertIn(b"T0137FEC880011223344556677FFFF\r", rx_data,
                      f"Expected ms sentinel FFFF in timestamp position, got: {rx_data!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_accuracy_milli(self):
        """Send two frames back-to-back at the slowest bitrate (10k/500k)
        and compare the reported ms timestamp delta against the expected
        inter-frame interval computed from CAN bit timing. Tolerance is
        1 ms (the ms-resolution rounding limit)."""
        self.dut.send(b"Z1\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"S0\r")    # 10 kbps nominal
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y0\r")    # 500 kbps data
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Two classic frames back-to-back. Data pattern 0x55/0xAA minimises
        # stuff-bit count for a predictable inter-frame interval.
        tx_frame = b"t55585555555555555555"
        self.dut.send(tx_frame + b"\r" + tx_frame + b"\r")
        time.sleep(0.1)
        rx_data = self.dut.receive()

        pos = 2 * len(b"z\r") + len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 4]
        pos = 2 * len(b"z\r") + len(tx_frame) + len(b"TTTT\r") + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 4]

        # Classic frame: 47 header bits + 64 data bits + 1(?) stuff bit, each at 100us @10kbps.
        time_exp_us = (int(timestamp_1st, 16) * 1000 + (47 + 8 * 8 + 1) * 100) % 60000000
        self.assertLess(abs(time_exp_us - int(timestamp_2nd, 16) * 1000), 1000,
                        f"Classic frame inter-frame interval mismatch: expected {time_exp_us} us, "
                        f"got {int(timestamp_2nd, 16) * 1000} us")

        # Two BRS FD frames back-to-back. Same intent: stuff-bit-minimal payload.
        tx_frame = b"B1555555585555555555555555"
        self.dut.send(tx_frame + b"\r" + tx_frame + b"\r")
        time.sleep(0.1)
        rx_data = self.dut.receive()

        pos = 2 * len(b"Z\r") + len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 4]
        pos = 2 * len(b"Z\r") + len(tx_frame) + len(b"TTTT\r") + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 4]

        # BRS FD frame: 49 nominal bits @100us + (8*8 data bits + 26 fd overhead + 5 + 2(?) stuff) at the data rate.
        time_exp_us = (int(timestamp_1st, 16) * 1000 + 49 * 100 + (8 * 8 + 26 + 5 + 2) * 2) % 60000000
        self.assertLess(abs(time_exp_us - int(timestamp_2nd, 16) * 1000), 1000,
                        f"BRS frame inter-frame interval mismatch: expected {time_exp_us} us, "
                        f"got {int(timestamp_2nd, 16) * 1000} us")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


class TimestampUsTestCase(unittest.TestCase):

    dut: DeviceUnderTest

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open()
        self.dut.setup()


    def tearDown(self):
        self.dut.close()


    def test_format_when_off(self):
        """When timestamp mode is off (default), reports for all frame types
        carry no timestamp suffix."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r",
                             f"Frame {cmd!r} should be reported without timestamp")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r",
                             f"Frame {cmd!r} should be reported without timestamp")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_format_when_on(self):
        """With Z2, every report appends an 8-hex-digit us timestamp."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            rx_data = self.dut.receive()
            self.assertEqual(len(rx_data), len(b"z\r" + cmd + b"03F0TTTTTTTT\r"),
                             f"Frame {cmd!r}: expected 8-char timestamp suffix, got {rx_data!r}")
            self.assertEqual(rx_data[:len(b"z\r" + cmd + b"03F0")], b"z\r" + cmd + b"03F0",
                             f"Frame {cmd!r}: body mismatch, got {rx_data!r}")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            rx_data = self.dut.receive()
            self.assertEqual(len(rx_data), len(b"Z\r" + cmd + b"0137FEC80TTTTTTTT\r"),
                             f"Frame {cmd!r}: expected 8-char timestamp suffix, got {rx_data!r}")
            self.assertEqual(rx_data[:len(b"Z\r" + cmd + b"0137FEC80")], b"Z\r" + cmd + b"0137FEC80",
                             f"Frame {cmd!r}: body mismatch, got {rx_data!r}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_mode_toggle(self):
        """Switching Z2 -> Z0 restores the no-timestamp format, confirming
        the mode setter is not sticky."""
        cmd_send_std = (b"r", b"t", b"d", b"b")
        cmd_send_ext = (b"R", b"T", b"D", b"B")

        # Turn on, then off.
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z0\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        for cmd in cmd_send_std:
            self.dut.send(cmd + b"03F0\r")
            self.assertEqual(self.dut.receive(), b"z\r" + cmd + b"03F0\r",
                             f"Frame {cmd!r} should be echoed without timestamp after Z2->Z0")
        for cmd in cmd_send_ext:
            self.dut.send(cmd + b"0137FEC80\r")
            self.assertEqual(self.dut.receive(), b"Z\r" + cmd + b"0137FEC80\r",
                             f"Frame {cmd!r} should be echoed without timestamp after Z2->Z0")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_basic_accuracy_30s(self):
        """Send two frames 30 s apart and confirm the reported us timestamp
        difference matches the host-side sleep within 600 ms (2% of 30 s).
        The tolerance absorbs USB latency and OS scheduling jitter."""
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # First frame -> grab baseline timestamp.
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        last_timestamp = rx_data[len(b"z\r" + b"t03F0"):len(b"z\r" + b"t03F0") + 8]
        last_time_us = int(last_timestamp.decode(), 16)

        # Sleep across the gap we want to measure.
        sleep_time_us = 30 * 1000 * 1000
        time.sleep(sleep_time_us / 1000.0 / 1000.0)

        # Second frame -> compute device-side elapsed time.
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()
        crnt_timestamp = rx_data[len(b"z\r" + b"t03F0"):len(b"z\r" + b"t03F0") + 8]
        crnt_time_us = int(crnt_timestamp.decode(), 16)
        if crnt_time_us > last_time_us:
            diff_time_us = crnt_time_us - last_time_us
        else:
            diff_time_us = (3600000000 + crnt_time_us) - last_time_us

        self.assertLess(abs(sleep_time_us - diff_time_us), 600 * 1000,
                        f"device elapsed {diff_time_us} us differs from host sleep {sleep_time_us} us by >=600 ms")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    # test_timestamp_wraparound_micro
    # Testing wrap around of micro timestamp will take about one hour.
    # We will verify this in long_time_test.py rather than here.


    def test_timestamp_same_stamp(self):
        """In internal-loopback mode a single transmitted frame produces both
        a Tx event report and an Rx frame report. Both share the same SOF
        moment, so their us timestamps must match within ±1 us.

        The ±1 us tolerance absorbs the TIM2/TIM3 phase jitter introduced by
        two separate gen_get_timestamp_us_from_tim3() calls (one for the
        Tx event, one for the Rx frame) — see test_timestamp_accuracy_micro
        for the same quantization effect.
        """
        # z2003: us timestamp + Tx event + Rx frame reporting all enabled.
        self.dut.send(b"z2003\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"t03F0\r")
        rx_data = self.dut.receive()

        # The order of Tx event and Rx frame reports is not guaranteed; pick
        # the timestamps based on which one came first.
        if rx_data[1] == b"z"[0]:
            tx_timestamp = rx_data[len(b"\rzt03F0"):len(b"\rzt03F0") + 8]
            rx_timestamp = rx_data[len(b"\rzt03F0TTTTTTTT\rt03F0"):len(b"\rzt03F0TTTTTTTT\rt03F0") + 8]
        else:
            tx_timestamp = rx_data[len(b"\rt03F0TTTTTTTT\rzt03F0"):len(b"\rt03F0TTTTTTTT\rzt03F0") + 8]
            rx_timestamp = rx_data[len(b"\rt03F0"):len(b"\rt03F0") + 8]
        tx_us = int(tx_timestamp, 16)
        rx_us = int(rx_timestamp, 16)
        self.assertAlmostEqual(tx_us, rx_us, delta=1,
                               msg=f"Tx event and Rx frame share one SOF, so timestamps "
                                   f"must match within ±1 us. tx={tx_us}, rx={rx_us}")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_consistency(self):
        """The us timestamp returned by the Z[CR] query and the one attached
        to a frame report must come from the same internal source. Sends
        both back-to-back at 1 Mbps and checks the gap stays within a
        plausible budget (frame TX ~50 us + one main-loop cycle ~100 us)."""
        self.dut.send(b"S8\r")           # 1 Mbps nominal: minimise frame TX time.
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"z2001\r")        # us timestamp + Rx frame reporting.
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")
        # The two commands are sent in one write to minimise the gap between
        # them on the device side. Responses can arrive across two receive()
        # calls due to USB buffering; concatenate to be safe.
        self.dut.send(b"Z\rt03F0\r")
        rx_data = self.dut.receive() + self.dut.receive()
        last_timestamp = rx_data[len(b"Z2"):len(b"Z2") + 8]
        last_time_us = int(last_timestamp.decode(), 16)
        crnt_timestamp = rx_data[len(b"Z2XXXXXXXX\rz\rt03F0"):len(b"Z2XXXXXXXX\rz\rt03F0") + 8]
        crnt_time_us = int(crnt_timestamp.decode(), 16)
        if crnt_time_us > last_time_us:
            diff_time_us = crnt_time_us - last_time_us
        else:
            diff_time_us = (3600000000 + crnt_time_us) - last_time_us

        self.assertLess(diff_time_us, 200,
                        f"Z query and frame timestamp must share one source; "
                        f"diff={diff_time_us} us exceeds budget (~150 us)")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_invalid_when_stalled(self):
        """When the main loop is stalled beyond the SOF-to-report design
        window (~20 ms), the us timestamp must be reported as the
        0xFFFFFFFF sentinel while the frame data itself remains intact.

        Uses the DEBUG-only stall command ~<HHHH>[CR] to block the main
        loop. CAN bitrate is set to 10 kbps so the loopback frame arrives
        during the stall, forcing a long SOF-to-report delay."""
        self.dut.send(b"S0\r")          # 10 kbps: frame arrives mid-stall.
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Z2\r")          # us timestamp on
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")           # open internal loopback
        self.assertEqual(self.dut.receive(), b"\r")

        # Enqueue Tx, give it 2 ms to start, then stall the loop for 40 ms.
        self.dut.send(b"T0137FEC880011223344556677\r")
        time.sleep(0.002)
        self.dut.send(b"~0028\r")

        # Drain output for >40 ms so the post-stall report is captured.
        rx_data = self.dut.receive() + self.dut.receive() + self.dut.receive()

        self.assertIn(b"T0137FEC880011223344556677FFFFFFFF\r", rx_data,
                      f"Expected us sentinel FFFFFFFF in timestamp position, got: {rx_data!r}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


    def test_timestamp_accuracy_micro(self):
        """Send 2 and 20 consecutive frames at the slowest bitrate (10k/500k)
        and compare the reported us timestamp delta against the expected
        inter-frame interval computed from CAN bit timing, for both classic
        and BRS FD frames. Tolerance is +/-1 us, which accounts for the
        TIM2/TIM3 independent-clock phase quantisation."""
        self.dut.send(b"Z2\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"S0\r")    # 10 kbps nominal
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"Y0\r")    # 500 kbps data
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"=\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # --- 2 classic frames ---
        # Data pattern 0x55/0xAA minimises stuff bits for a predictable interval.
        tx_frame = b"t55585555555555555555"
        self.dut.send(tx_frame + b"\r" + tx_frame + b"\r")
        time.sleep(0.1)
        rx_data = self.dut.receive()

        pos = 2 * len(b"z\r") + len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 8]
        pos = 2 * len(b"z\r") + len(tx_frame) + len(b"TTTTTTTT\r") + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 8]

        # Classic frame: 47 header bits + 64 data bits + 1(?) stuff bit, each at 100us @10kbps.
        time_exp_us = (int(timestamp_1st, 16) + (47 + 8 * 8 + 1) * 100) % 3600000000
        self.assertAlmostEqual(time_exp_us, int(timestamp_2nd, 16), delta=1,
                               msg=f"2-classic interval mismatch: expected {time_exp_us}, "
                                   f"got {int(timestamp_2nd, 16)}")

        # --- 2 BRS FD frames ---
        tx_frame = b"B1555555585555555555555555"
        self.dut.send(tx_frame + b"\r" + tx_frame + b"\r")
        time.sleep(0.1)
        rx_data = self.dut.receive()

        pos = 2 * len(b"Z\r") + len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 8]
        pos = 2 * len(b"Z\r") + len(tx_frame) + len(b"TTTTTTTT\r") + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 8]

        # BRS FD frame: 49 nominal bits @100us + (64 data + 26 fd overhead + 5 + 2(?) stuff) at data rate.
        time_exp_us = (int(timestamp_1st, 16) + 49 * 100 + (8 * 8 + 26 + 5 + 2) * 2) % 3600000000
        self.assertAlmostEqual(time_exp_us, int(timestamp_2nd, 16), delta=1,
                               msg=f"2-BRS interval mismatch: expected {time_exp_us}, "
                                   f"got {int(timestamp_2nd, 16)}")

        # --- 20 classic frames: 1us jitter averages out over 19 intervals ---
        tx_frame = b"t55585555555555555555"
        for _ in range(0, 20):
            self.dut.send(tx_frame + b"\r")
        time.sleep(0.5)
        rx_data = self.dut.receive()
        # All Tx-event ACK prefixes arrive first; strip them to walk frames cleanly.
        rx_data = rx_data.replace(b"z\r", b"")

        pos = len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 8]
        pos = 19 * (len(tx_frame) + len(b"TTTTTTTT\r")) + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 8]

        time_exp_us = (int(timestamp_1st, 16) + 19 * (47 + 8 * 8 + 1) * 100) % 3600000000
        self.assertAlmostEqual(time_exp_us, int(timestamp_2nd, 16), delta=1,
                               msg=f"20-classic accumulated interval mismatch: expected {time_exp_us}, "
                                   f"got {int(timestamp_2nd, 16)}")

        # --- 20 BRS FD frames ---
        tx_frame = b"B1555555585555555555555555"
        for _ in range(0, 20):
            self.dut.send(tx_frame + b"\r")
        time.sleep(0.5)
        rx_data = self.dut.receive()
        rx_data = rx_data.replace(b"Z\r", b"")

        pos = len(tx_frame)
        timestamp_1st = rx_data[pos : pos + 8]
        pos = 19 * (len(tx_frame) + len(b"TTTTTTTT\r")) + len(tx_frame)
        timestamp_2nd = rx_data[pos : pos + 8]

        time_exp_us = (int(timestamp_1st, 16) + 19 * (49 * 100 + (8 * 8 + 26 + 5 + 2) * 2)) % 3600000000
        self.assertAlmostEqual(time_exp_us, int(timestamp_2nd, 16), delta=1,
                               msg=f"20-BRS accumulated interval mismatch: expected {time_exp_us}, "
                                   f"got {int(timestamp_2nd, 16)}")

        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")


if __name__ == "__main__":
    unittest.main()
