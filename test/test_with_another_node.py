#!/usr/bin/env python3

import re
import sys
import time
import unittest

from device_under_test import DeviceUnderTest


# NOTE: All tests in this file require another device (aux) wired to the DUT
#       on the same CAN bus, with no other node on the bus. The tests open
#       and drive the aux themselves, so no manual setup of the aux is needed.
#       Some tests need a DEBUG build on the aux (the aux opens with + or -),
#       and are skipped otherwise. So, unlike the other tests, the aux uses
#       the default port of the DUT, where the DEBUG build is connected in the
#       other tests, and the DUT uses the other port. The DUT needs no DEBUG
#       build, but should run the firmware under test.
if sys.platform == "win32":
    DUT_PORT = "COM8"
    AUX_PORT = "COM9"
else:
    DUT_PORT = "/dev/ttyACM1"
    AUX_PORT = "/dev/ttyACM0"


class AuxTestCase(unittest.TestCase):
    """Base class: opens the DUT and the aux, and sets both to the defaults."""

    dut: DeviceUnderTest
    aux: DeviceUnderTest
    aux_debug_required = False

    def setUp(self):
        self.dut = DeviceUnderTest()
        self.dut.open(DUT_PORT)
        self.dut.setup()
        self.aux = DeviceUnderTest()
        self.aux.open(AUX_PORT)
        self.aux.setup()
        if self.aux_debug_required and not self.aux.debug_build:
            self.aux.close()
            self.dut.close()
            self.skipTest("Requires a DEBUG build on the aux (+ and - modes)")


    def tearDown(self):
        self.aux.send(b"C\r")
        self.aux.receive()
        self.aux.close()
        self.dut.send(b"C\r")
        self.dut.receive()
        self.dut.close()


    def open_aux(self, mode: bytes):
        """Enable the Tx event report only, then open the aux with mode."""
        self.aux.send(b"z0002\r")   # no rx, tx event only
        self.assertEqual(self.aux.receive(), b"\r")
        self.aux.send(mode)
        self.assertEqual(self.aux.receive(), b"\r")


class ListenOnlyTestCase(AuxTestCase):
    """The DUT opened with L (listen only mode).

    The DUT does not ACK in listen only mode, so the aux sends with
    + (external loopback) when the frames must succeed: the aux then
    ignores the missing ACK and transmits each frame only once.
    In + mode, the aux does not see the bus at all (the FDCAN disregards
    its Rx pin), so it cannot see what the DUT does on the bus. The tests
    that check it use - (normal mode, no retransmission) on the aux.
    """

    aux_debug_required = True

    def aux_recent_err_code(self) -> bytes:
        """Return the recent error code shown by f on the aux."""
        self.aux.send(b"f\r")
        match = re.search(rb"recent_err_code=(\w+)", self.aux.receive())
        self.assertIsNotNone(match, "Aux should show recent_err_code in f")
        return match.group(1)


    def test_receive_in_listen_only(self):
        """Every frame type is received in listen only mode.

        The aux sends the shortest and the longest frame of each of the
        8 types with + (external loopback), so each frame succeeds without
        an ACK. The aux must report the Tx event, and the DUT must report
        the same frame.
        """
        #self.dut.print_on = True
        data = b"00112233445566778899AABBCCDDEEFF" * 4
        frames = (
            b"r03F0\r", b"t03F0\r", b"d03F0\r", b"b03F0\r",
            b"R0137FEC80\r", b"T0137FEC80\r", b"D0137FEC80\r", b"B0137FEC80\r",
            b"r03FF\r", b"t03F80011223344556677\r",
            b"d03FF" + data + b"\r", b"b03FF" + data + b"\r",
            b"R0137FEC8F\r", b"T0137FEC880011223344556677\r",
            b"D0137FEC8F" + data + b"\r", b"B0137FEC8F" + data + b"\r",
        )

        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.open_aux(b"+\r")

        for frame in frames:
            tx_event = (b"Z" if frame[:1].isupper() else b"z") + frame
            self.aux.send(frame)
            self.assertEqual(self.aux.receive(), b"\r" + tx_event,
                             f"Aux should report the Tx event of {frame!r}")
            self.assertEqual(self.dut.receive(), frame,
                             f"DUT should receive {frame!r} in listen only mode")


    def test_no_ack_in_listen_only(self):
        """The DUT does not ACK in listen only mode.

        The aux sends with - (no retransmission), so a frame without an ACK
        fails once and gives no Tx event. First the DUT opens with O and
        ACKs the frame, which confirms the wiring. Then the DUT opens with L:
        the frame must fail with the bus error (F80) on the aux, and the DUT
        must not receive it, since the aux destroys it with an error frame
        at the ACK delimiter.
        """
        #self.dut.print_on = True
        frame = b"t03F21122\r"
        self.open_aux(b"-\r")

        # Normal mode: ACKed by the DUT
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.aux.send(frame)
        self.assertEqual(self.aux.receive(), b"\r" + b"z" + frame,
                         "Aux should report the Tx event (ACKed by the DUT in normal mode)")
        self.assertEqual(self.dut.receive(), frame,
                         "DUT should receive the frame in normal mode")
        self.aux.send(b"F\r")
        self.assertEqual(self.aux.receive(), b"F00\r",
                         "Aux should see no bus error while the DUT ACKs")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Listen only mode: no ACK
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.aux.send(frame)
        self.assertEqual(self.aux.receive(), b"\r",
                         "Aux should give no Tx event (no ACK from the DUT in listen only mode)")
        self.assertEqual(self.dut.receive(), b"",
                         "DUT should not receive the frame destroyed by the ACK error")
        self.aux.send(b"F\r")
        self.assertEqual(self.aux.receive(), b"F80\r",
                         "Aux should see the bus error (ACK error)")


    def test_no_error_frame_in_listen_only(self):
        """The DUT sends no error frame in listen only mode.

        The DUT runs at Y5 (5 Mbps data) while the aux stays at Y2 (default),
        so the DUT cannot receive the data phase of b (BRS) frames. The aux
        sends a b frame with - (no retransmission), and the frame fails in
        both modes of the DUT. The error code shown by f on the aux tells
        how it failed:
          - O (normal mode, as a reference): the DUT destroys the frame with
            an error frame, so the aux sees an error other than the ACK error
            (BIT1 on the bench).
          - L (listen only mode): the aux sends the frame to the end and sees
            only the missing ACK (_ACK).
        The reference confirms that the aux can see the error frame of the
        DUT. The error code in f is unspecified in general, but it is used
        here to check how the frame failed.
        """
        #self.dut.print_on = True
        frame = b"b03F21122\r"
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Normal mode (reference): the error frame of the DUT destroys the frame
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.open_aux(b"-\r")
        self.aux.send(frame)
        self.assertEqual(self.aux.receive(), b"\r",
                         "Aux should give no Tx event (the frame fails at the DUT)")
        self.assertEqual(self.dut.receive(), b"",
                         "DUT should not receive the frame (data phase too slow)")
        self.aux.send(b"F\r")
        self.assertEqual(self.aux.receive(), b"F80\r",
                         "Aux should see the bus error")
        code = self.aux_recent_err_code()
        self.assertNotIn(code, (b"_ACK", b"NONE"),
                         f"Aux should see the error frame of the DUT in normal mode, not {code!r}")
        self.aux.send(b"C\r")   # reopen later to reset the error codes
        self.assertEqual(self.aux.receive(), b"\r")
        self.dut.send(b"C\r")
        self.assertEqual(self.dut.receive(), b"\r")

        # Listen only mode: no error frame, so only the ACK is missing
        self.dut.send(b"L\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.open_aux(b"-\r")
        self.aux.send(frame)
        self.assertEqual(self.aux.receive(), b"\r",
                         "Aux should give no Tx event (no ACK from the DUT)")
        self.assertEqual(self.dut.receive(), b"",
                         "DUT should not receive the frame (data phase too slow)")
        self.aux.send(b"F\r")
        self.assertEqual(self.aux.receive(), b"F80\r",
                         "Aux should see the bus error (ACK error)")
        self.assertEqual(self.aux_recent_err_code(), b"_ACK",
                         "Aux should see only the ACK error (no error frame from the DUT)")


class ReceiveErrorTestCase(AuxTestCase):
    """F bit 7 (BUS_ERROR) with receive errors mixed with successful receptions.

    Setup: the DUT runs at Y5 (5 Mbps data) and opens with O, while the aux
    stays at Y2 (default) and opens with - (no retransmission). B (BRS) frames
    from the aux fail at the DUT in the data phase, as the DUT cannot follow
    the slower data bit rate, and the DUT destroys them with an error frame.
    t frames use only the nominal phase, so the DUT receives and ACKs them.
    DUT REC: up per failed frame, -1 per successful frame.
    Only F is checked for the flag; the error code in f is unspecified.
    """

    aux_debug_required = True

    def test_receive_error_mixed_with_success(self):
        """The aux repeats [B x 1, t x 8], and the DUT reads F after each
        group. Each group must give F80 at the DUT and nothing else, although
        its REC goes back down with the t frames. The aux must also give F80,
        which confirms that the B frame failed. The DUT must receive every t
        frame and no B frame, and the error counters must be back to 0 at the
        end (as shown by f).
        """
        #self.dut.print_on = True
        aux_rx = b""
        aux_rx_exp = b""
        dut_rx = b""
        dut_rx_exp = b""
        self.dut.send(b"Y5\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.open_aux(b"-\r")

        for grp in range(0, 16):
            frame = b"B" + format(grp, "08X").encode() + b"2" + format(grp, "04X").encode() + b"\r"
            aux_rx_exp += b"\r"     # no Tx event
            self.aux.send(frame)
            # Wait until the B frame has failed. With no retransmission (DAR), the
            # frames added just after a failure are not sent at all. See
            # BusErrorFlagTestCase in test_with_receiver.py for the details.
            time.sleep(0.005)
            for i in range(0, 8):
                num = grp * 8 + i
                frame = b"t" + format(num, "03X").encode() + b"2" + format(num, "04X").encode() + b"\r"
                aux_rx_exp += b"\r" + b"z" + frame
                dut_rx_exp += frame
                self.aux.send(frame)
                time.sleep(0.001)
            aux_rx += self.aux.receive()
            dut_rx += self.dut.receive()
            self.dut.send(b"F\r")
            self.assertEqual(self.dut.receive(), b"F80\r",
                             f"Expected F80 (BUS_ERROR only) at the DUT after group {grp} of [B x 1, t x 8]")
            self.aux.send(b"F\r")
            self.assertEqual(self.aux.receive(), b"F80\r",
                             f"Expected F80 at the aux after group {grp} (the B frame should fail)")

        # [CR] and Tx event may swap, so remove all [CR].
        self.assertEqual(aux_rx.replace(b"\r", b""), aux_rx_exp.replace(b"\r", b""),
                         "Every t frame should give its Tx event at the aux")
        self.assertEqual(dut_rx, dut_rx_exp,
                         "DUT should receive every t frame and no B frame")

        self.dut.send(b"f\r")
        self.assertIsNotNone(re.search(rb"err_cnt_tx_rx=\[0x00, 0x00\]", self.dut.receive()),
                             "DUT error counters should be back to 0 after the t frames")


class ArbitrationTestCase(AuxTestCase):
    """Arbitration loss is not an error: F bit 6 (arbitration lost) is not
    supported, and no other bit may be raised."""

    def test_arbitration_loss(self):
        """The DUT and the aux queue many frames at once, with even IDs on the
        DUT and odd IDs on the aux, so both nodes start their frames together
        and arbitrate. Both must give F00.

        The order on the bus confirms the arbitration: while both nodes have
        frames pending, the lower ID wins every time, so after the first frames
        of the node that starts first, the IDs must be in ascending order with
        the two nodes taking turns. S2 (50 kbps) makes each frame long
        compared with the delay between the writes to the two devices.
        """
        #self.dut.print_on = True
        num = 64
        for dev in (self.dut, self.aux):
            dev.send(b"S2\r")
            self.assertEqual(dev.receive(), b"\r")
        self.dut.send(b"z0003\r")   # rx and tx event
        self.assertEqual(self.dut.receive(), b"\r")
        self.dut.send(b"O\r")
        self.assertEqual(self.dut.receive(), b"\r")
        self.open_aux(b"O\r")

        dut_tx = b"".join(b"t" + format(2 * i, "03X").encode() + b"1" + format(i, "02X").encode() + b"\r"
                          for i in range(0, num))
        aux_tx = b"".join(b"t" + format(2 * i + 1, "03X").encode() + b"1" + format(i, "02X").encode() + b"\r"
                          for i in range(0, num))
        self.dut.send(dut_tx)
        self.aux.send(aux_tx)
        time.sleep(0.5)     # 2 * num frames of about 1.2 ms each
        dut_rx = self.dut.receive()
        aux_rx = self.aux.receive()

        # Order on the bus, seen by the DUT: its own Tx events (zt...) and the
        # frames received from the aux (t...)
        ids = []
        for line in dut_rx.split(b"\r"):
            if line[:1] == b"z":
                line = line[1:]
            if line[:1] == b"t":
                ids.append(int(line[1:4], 16))
        self.assertEqual(sorted(ids), list(range(0, 2 * num)),
                         "Every frame should appear on the bus exactly once")
        self.assertEqual(len(re.findall(rb"zt", aux_rx)), num,
                         "Every frame of the aux should give its Tx event")

        # Skip the frames sent by the first node alone, before the other one starts
        start = 0
        while start < len(ids) and ids[start] % 2 == ids[0] % 2:
            start += 1
        contended = ids[start:]
        self.assertEqual(contended, sorted(contended),
                         "Lower ID should win the arbitration every time")
        turns = sum(1 for a, b in zip(contended, contended[1:]) if a % 2 != b % 2)
        self.assertGreaterEqual(turns, num,
                                "The two nodes should take turns (frames should be sent together)")

        self.dut.send(b"F\r")
        self.assertEqual(self.dut.receive(), b"F00\r",
                         "Expected F00 at the DUT (arbitration loss is not an error)")
        self.aux.send(b"F\r")
        self.assertEqual(self.aux.receive(), b"F00\r",
                         "Expected F00 at the aux (arbitration loss is not an error)")


if __name__ == "__main__":
    unittest.main()
